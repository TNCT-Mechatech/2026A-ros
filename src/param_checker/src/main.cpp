#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_packets.hpp"

#include "debug_msgs/msg/feedback.hpp"
#include "debug_msgs/msg/pid_gain.hpp"

#include <chrono>
#include <vector>

class ParamCheck : public rclcpp::Node {
   public:
    ParamCheck () : Node ("param_check") {
        serial_path_ = declare_parameter<std::string> ("serial_path", "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.2");
        timer_ptr    = this->create_wall_timer (10ms, std::bind (&ParamCheck::timer_callback, this));

        for (int i = 0; i < 4; i++) feedback_.push_back (create_publisher<debug_msgs::msg::Feedback> ("feedback", 10));

        open_serial ();

        command_frame_id_  = declare_parameter<int> ("command_frame_id", 0);
        feedback_frame_id_ = declare_parameter<int> ("feedback_frame_id", 1);

        serial_->add_frame (command_frame_id_, &command_msg_);
        serial_->add_frame (feedback_frame_id_, &feedback_msg_);

        target_wheel_rps_.reserve (4);
    }

    ~ParamCheck () override {
        send_stop ();

        if (serial_ != nullptr) {
            serial_->rm_frame (command_frame_id_);
            serial_->rm_frame (feedback_frame_id_);
        }

        delete serial_;
        delete dev_;
    }

    void timer_callback () {
        serial_->update ();

        if (feedback_msg_.was_updated ()) {
            for (size_t i = 0; i < feedback_.size (); i++) {
                debug_msgs::msg::Feedback msg;

                msg.output = feedback_msg_.data.output[i];
                target_rps = target_wheel_rps[i] current_rps = feedback_msg_.data.measured_wheel_rps[i];
                encoder_count                                = feedback_msg_.data.encoder_count[i];
                duty                                         = feedback_msg_.data.duty[i];
                dir                                          = feedback_msg_.data.dir[i];

                feedback_[i]->publish (msg);
            }
        }
    }

   private:
    void open_serial () {
        /*
          SerialBridge側のボーレート指定。
          READMEの例に合わせて B9600 にしてある。
          速くしたい場合は B115200 などに変更する。
        */
        dev_    = new LinuxHardwareSerial (serial_path_.c_str (), B115200);
        serial_ = new SerialBridge (dev_);
    }

    std::vector<float> target_wheel_rps_;

    std::string serial_path_;

    rclcpp::TimerBase::SharedPtr timer_ptr;

    std::vector<rclcpp::Publisher<debug_msgs::msg::Feedback>::SharedPtr>   feedback_;
    std::vector<rclcpp::Subscription<debug_msgs::msg::PidGain>::SharedPtr> pid_gain_;

    int command_frame_id_;
    int feedback_frame_id_;

    SerialDev    *dev_{nullptr};
    SerialBridge *serial_{nullptr};

    sb::Message<MotorCommandPacket>  command_msg_;
    sb::Message<MotorFeedbackPacket> feedback_msg_;
};

int main (int argc, char **argv) {
    rclcpp::init (argc, argv);

    try {
        rclcpp::spin (std::make_shared<ParamCheck> ());
    } catch (const std::exception &e) {
        std::cerr << "param_check error: " << e.what () << std::endl;
    }

    rclcpp::shutdown ();
    return 0;
}