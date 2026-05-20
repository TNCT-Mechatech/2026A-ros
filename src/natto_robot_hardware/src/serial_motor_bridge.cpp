#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <SerialBridge.hpp>
#include <LinuxHardwareSerial.hpp>
#include <Message.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "natto_robot_hardware/serial_packets.hpp"

using namespace std::chrono_literals;

class SerialMotorBridge : public rclcpp::Node
{
public:
  SerialMotorBridge()
  : Node("serial_motor_bridge")
  {
    serial_path_ = declare_parameter<std::string>("serial_path", "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.2");
    command_topic_ = declare_parameter<std::string>("command_topic", "/wheel_command");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");

    input_unit_ = declare_parameter<std::string>("input_unit", "radps");
    max_rps_ = declare_parameter<double>("max_rps", 2.0);
    timeout_ms_ = declare_parameter<int>("timeout_ms", 300);

    command_frame_id_ = declare_parameter<int>("command_frame_id", 0);
    feedback_frame_id_ = declare_parameter<int>("feedback_frame_id", 1);

    std::vector<std::string> default_names = {
      "wheel_0", "wheel_1", "wheel_2", "wheel_3"
    };
    wheel_names_ = declare_parameter<std::vector<std::string>>("wheel_names", default_names);

    open_serial();

    for (double & rps : last_position_rad_) {
      rps = 0.0;
    }

    serial_->add_frame(command_frame_id_, &command_msg_);
    serial_->add_frame(feedback_frame_id_, &feedback_msg_);

    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      command_topic_,
      10,
      std::bind(&SerialMotorBridge::command_callback, this, std::placeholders::_1)
    );

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      10
    );

    timer_ = create_wall_timer(
      10ms,
      std::bind(&SerialMotorBridge::timer_callback, this)
    );

    last_command_time_ = now();
    last_feedback_time_ = now();

    RCLCPP_INFO(get_logger(), "serial_motor_bridge started");
    RCLCPP_INFO(get_logger(), "serial_path: %s", serial_path_.c_str());
    RCLCPP_INFO(get_logger(), "command_topic: %s", command_topic_.c_str());
    RCLCPP_INFO(get_logger(), "joint_state_topic: %s", joint_state_topic_.c_str());
  }

  ~SerialMotorBridge() override
  {
    send_stop();

    if (serial_ != nullptr) {
      serial_->rm_frame(command_frame_id_);
      serial_->rm_frame(feedback_frame_id_);
    }

    delete serial_;
    delete dev_;
  }

private:
  void open_serial()
  {
    /*
      SerialBridge側のボーレート指定。
      READMEの例に合わせて B9600 にしてある。
      速くしたい場合は B115200 などに変更する。
    */
    dev_ = new LinuxHardwareSerial(serial_path_.c_str(), B115200);
    serial_ = new SerialBridge(dev_);
  }

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_command_time_ = now();

    const size_t count = std::min<size_t>(4, msg->velocity.size());

    for (size_t i = 0; i < 4; ++i) {
      command_msg_.data.target_wheel_rps[i] = 0.0f;
    }

    for (size_t i = 0; i < count; ++i) {
      double rps = 0.0;

      if (input_unit_ == "radps") {
        rps = msg->velocity[i] / (2.0 * M_PI);
      } else if (input_unit_ == "rps") {
        rps = msg->velocity[i];
      } else {
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "invalid input_unit: %s",
          input_unit_.c_str()
        );
        send_stop();
        return;
      }

      rps = std::clamp(rps, -max_rps_, max_rps_);
      command_msg_.data.target_wheel_rps[i] = static_cast<float>(rps);
    }

    serial_->write(command_frame_id_);
  }

  void timer_callback()
  {
    serial_->update();

    const auto elapsed_ms = (now() - last_command_time_).nanoseconds() / 1000000;

    if (elapsed_ms > timeout_ms_) {
      send_stop();
    }

    if (feedback_msg_.was_updated()) {
      last_feedback_time_ = now();
      publish_joint_state();
    }
  }

  void publish_joint_state()
  {
    sensor_msgs::msg::JointState msg;

    msg.header.stamp = now();
    msg.name = wheel_names_;

    msg.position.resize(4);
    msg.velocity.resize(4);
    msg.effort.resize(4);

    const double dt = 0.01;

    for (size_t i = 0; i < 4; ++i) {
      const double rps = feedback_msg_.data.measured_wheel_rps[i];
      const double radps = rps * 2.0 * M_PI;

      last_position_rad_[i] += radps * dt;

      msg.position[i] = last_position_rad_[i];
      msg.velocity[i] = radps;
      msg.effort[i] = 0.0;
    }

    joint_state_pub_->publish(msg);
  }

  void send_stop()
  {
    for (size_t i = 0; i < 4; ++i) {
      command_msg_.data.target_wheel_rps[i] = 0.0f;
    }

    if (serial_ != nullptr) {
      serial_->write(command_frame_id_);
    }
  }

  std::string serial_path_;
  std::string command_topic_;
  std::string joint_state_topic_;
  std::string input_unit_;

  double max_rps_;
  int timeout_ms_;
  int command_frame_id_;
  int feedback_frame_id_;

  std::vector<std::string> wheel_names_;

  SerialDev * dev_{nullptr};
  SerialBridge * serial_{nullptr};

  sb::Message<MotorCommandPacket> command_msg_;
  sb::Message<MotorFeedbackPacket> feedback_msg_;

  double last_position_rad_[4]{};

  rclcpp::Time last_command_time_;
  rclcpp::Time last_feedback_time_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<SerialMotorBridge>());
  } catch (const std::exception & e) {
    std::cerr << "serial_motor_bridge error: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}