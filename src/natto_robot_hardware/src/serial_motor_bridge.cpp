#include "natto_robot_hardware/serial_packets.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include <LinuxHardwareSerial.hpp>
#include <Message.hpp>
#include <SerialBridge.hpp>

#include "debug_msgs/msg/feedback.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class SerialMotorBridge : public rclcpp::Node {
   public:
    SerialMotorBridge () : Node ("serial_motor_bridge") {
        serial_path_       = declare_parameter<std::string> ("serial_path", "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.2:1.2");
        command_topic_     = declare_parameter<std::string> ("command_topic", "/command_joint_states");
        joint_state_topic_ = declare_parameter<std::string> ("joint_state_topic", "/joint_states");

        pid_gain_config_path_ = declare_parameter<std::string> ("gain_config_path", "/home/minimechatech/2026A-ros/examples/natto_nhk2025_sample/config/pid_gain.yaml");

        stop_mode_ = declare_parameter<bool> ("stop_mode", false);

        command_mode_                = declare_parameter<std::string> ("command_mode", "closed_loop_velocity");
        input_unit_                  = declare_parameter<std::string> ("input_unit", "radps");
        max_rps_                     = declare_parameter<double> ("max_rps", 4.0);
        open_loop_full_scale_rps_    = declare_parameter<double> ("open_loop_full_scale_rps", 4.0);
        open_loop_output_limit_      = declare_parameter<double> ("open_loop_output_limit", 1.0);
        open_loop_slew_rate_per_sec_ = declare_parameter<double> ("open_loop_slew_rate_per_sec", 1.0);
        timeout_ms_                  = declare_parameter<int64_t> ("timeout_ms", 300);
        reconnect_interval_ms_       = declare_parameter<int64_t> ("reconnect_interval_ms", 1000);

        command_frame_id_  = declare_parameter<int64_t> ("command_frame_id", 0);
        feedback_frame_id_ = declare_parameter<int64_t> ("feedback_frame_id", 1);

        std::vector<std::string> default_names = {"mecanum_wheel_fl", "mecanum_wheel_fr", "mecanum_wheel_rl", "mecanum_wheel_rr"};
        wheel_names_                           = declare_parameter<std::vector<std::string>> ("wheel_names", default_names);
        wheel_polarities_                      = declare_parameter<std::vector<int64_t>> ("wheel_polarities", {1, 1, 1, 1});
        encoder_polarities_                    = declare_parameter<std::vector<int64_t>> ("encoder_polarities", {1, 1, 1, 1});

        if (wheel_polarities_.size () != 4) {
            throw std::invalid_argument ("wheel_polarities must contain exactly 4 values");
        }
        for (const int64_t polarity : wheel_polarities_) {
            if (polarity != 1 && polarity != -1) {
                throw std::invalid_argument ("wheel_polarities values must be either 1 or -1");
            }
        }

        if (encoder_polarities_.size () != 4) {
            throw std::invalid_argument ("encoder_polarities must contain exactly 4 values");
        }

        for (const int64_t polarity : encoder_polarities_) {
            if (polarity != 1 && polarity != -1) {
                throw std::invalid_argument ("encoder_polarities values must be either 1 or -1");
            }
        }

        pid = declare_parameter<bool> ("pid", true);

        if (command_mode_ != "closed_loop_velocity" && command_mode_ != "open_loop_normalized") {
            throw std::invalid_argument ("command_mode must be either closed_loop_velocity or open_loop_normalized");
        }
        if (command_mode_ == "closed_loop_velocity" && !pid) {
            throw std::invalid_argument ("closed_loop_velocity requires pid=true");
        }
        if (command_mode_ == "open_loop_normalized" && pid) {
            throw std::invalid_argument ("open_loop_normalized requires pid=false");
        }
        if (input_unit_ != "radps" && input_unit_ != "rps") {
            throw std::invalid_argument ("input_unit must be either radps or rps");
        }
        if (!std::isfinite (max_rps_) || max_rps_ <= 0.0) {
            throw std::invalid_argument ("max_rps must be finite and greater than zero");
        }
        if (!std::isfinite (open_loop_full_scale_rps_) || open_loop_full_scale_rps_ <= 0.0) {
            throw std::invalid_argument ("open_loop_full_scale_rps must be finite and greater than zero");
        }
        if (!std::isfinite (open_loop_output_limit_) || open_loop_output_limit_ <= 0.0 || open_loop_output_limit_ > 1.0) {
            throw std::invalid_argument ("open_loop_output_limit must be finite and in (0, 1]");
        }
        if (!std::isfinite (open_loop_slew_rate_per_sec_) || open_loop_slew_rate_per_sec_ <= 0.0) {
            throw std::invalid_argument ("open_loop_slew_rate_per_sec must be finite and greater than zero");
        }

        for (double &position : last_position_rad_) {
            position = 0.0;
        }

        command_sub_ = create_subscription<sensor_msgs::msg::JointState> (command_topic_, rclcpp::SensorDataQoS (), std::bind (&SerialMotorBridge::command_callback, this, std::placeholders::_1));

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState> (joint_state_topic_, rclcpp::SensorDataQoS ());

        for (size_t i = 0; i < 4; ++i) {
            feedback_pubs_.push_back (create_publisher<debug_msgs::msg::Feedback> ("feedback" + std::to_string (i), 10));
        }

        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray> ("/diagnostics", 10);

        feedback_timeout_ms_ = declare_parameter<int64_t> ("feedback_timeout_ms", 200);

        max_feedback_dt_sec_ = declare_parameter<double> ("max_feedback_dt_sec", 0.1);

        last_feedback_stamp_ = now ();

        direction[0] = true;
        direction[1] = true;
        direction[2] = true;
        direction[3] = true;

        std::fill (std::begin (kp), std::end (kp), 0.0f);
        std::fill (std::begin (ki), std::end (ki), 0.0f);
        std::fill (std::begin (kd), std::end (kd), 0.0f);

        if (pid) {
            YAML::Node config = YAML::LoadFile (pid_gain_config_path_);

            for (int i = 0; i < 4; i++) {
                kp[i] = config["wheel" + std::to_string (i)]["kp"].as<float> ();
                ki[i] = config["wheel" + std::to_string (i)]["ki"].as<float> ();
                kd[i] = config["wheel" + std::to_string (i)]["kd"].as<float> ();
            }
        }

        initialize_command_packet ();

        last_command_time_      = now ();
        last_feedback_time_     = now ();
        last_reconnect_attempt_ = now ();

        try_open_serial ();

        timer_ = create_wall_timer (10ms, std::bind (&SerialMotorBridge::timer_callback, this));

        RCLCPP_INFO (get_logger (), "serial_motor_bridge started");
        RCLCPP_INFO (get_logger (), "serial_path: %s", serial_path_.c_str ());
        RCLCPP_INFO (get_logger (), "command_topic: %s", command_topic_.c_str ());
        RCLCPP_INFO (get_logger (), "joint_state_topic: %s", joint_state_topic_.c_str ());
        RCLCPP_INFO (get_logger (), "input_unit: %s", input_unit_.c_str ());
        RCLCPP_INFO (get_logger (), "command_mode: %s", command_mode_.c_str ());
        RCLCPP_INFO (get_logger (), "serial command unit: %s", pid ? "rps" : "normalized output [-1, 1]");
        RCLCPP_INFO (get_logger (), "serial feedback unit: rps");
    }

    ~SerialMotorBridge () override {
        send_stop ();
        close_serial ();
    }

   private:
    void close_serial () {
        try {
            if (serial_ != nullptr) {
                serial_->rm_frame (static_cast<SerialBridge::frame_id> (command_frame_id_));
                serial_->rm_frame (static_cast<SerialBridge::frame_id> (feedback_frame_id_));
            }
        } catch (...) {
        }

        delete serial_;
        serial_ = nullptr;

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#endif
        delete dev_;
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
        dev_ = nullptr;

        serial_connected_ = false;
    }

    bool try_open_serial () {
        close_serial ();

        try {
            // LinuxHardwareSerial exits the process when open(2) fails, so check first and
            // turn the normal disconnected state into a recoverable exception.
            if (access (serial_path_.c_str (), R_OK | W_OK) != 0) {
                throw std::runtime_error ("serial device is unavailable: " + serial_path_ + ": " + std::strerror (errno));
            }

            dev_    = new LinuxHardwareSerial (serial_path_.c_str (), B230400);
            serial_ = new SerialBridge (dev_);

            serial_->add_frame (static_cast<SerialBridge::frame_id> (command_frame_id_), &command_msg_);
            serial_->add_frame (static_cast<SerialBridge::frame_id> (feedback_frame_id_), &feedback_msg_);

            serial_connected_         = true;
            feedback_received_        = false;
            feedback_timeout_latched_ = false;

            set_zero_command ();
            if (!write_command ()) {
                throw std::runtime_error ("failed to send the initial zero command");
            }

            RCLCPP_INFO (get_logger (), "Serial connection established: %s", serial_path_.c_str ());
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR (get_logger (), "Serial connection failed: %s", e.what ());
        } catch (...) {
            RCLCPP_ERROR (get_logger (), "Serial connection failed with an unknown error");
        }

        close_serial ();
        return false;
    }

    bool write_command () {
        if (!serial_connected_ || serial_ == nullptr) {
            return false;
        }

        try {
            const int result = serial_->write (static_cast<SerialBridge::frame_id> (command_frame_id_));
            if (result < 0) {
                throw std::runtime_error (std::string ("write failed: ") + std::strerror (errno));
            }
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR (get_logger (), "Serial write failed: %s", e.what ());
        } catch (...) {
            RCLCPP_ERROR (get_logger (), "Serial write failed with an unknown error");
        }

        close_serial ();
        return false;
    }

    void initialize_command_packet () {
        for (size_t i = 0; i < 4; ++i) {
            command_msg_.data.mc[i].target_wheel_rps = 0.0f;
            command_msg_.data.mc[i].kp               = pid ? kp[i] : 0.0f;
            command_msg_.data.mc[i].ki               = pid ? ki[i] : 0.0f;
            command_msg_.data.mc[i].kd               = 0.0f;
            command_msg_.data.mc[i].set_direction    = direction[i];
            command_msg_.data.mc[i].pid_state        = pid;
            command_msg_.data.mc[i].stop_mode        = stop_mode_;
        }
        last_open_loop_outputs_.fill (0.0);
    }

    void set_zero_command () {
        for (size_t i = 0; i < 4; ++i) {
            command_msg_.data.mc[i].target_wheel_rps = 0.0f;
        }
        last_open_loop_outputs_.fill (0.0);
    }

    void command_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
        const auto command_time = now ();
        const auto command_dt   = std::clamp ((command_time - last_command_time_).seconds (), 0.0, 0.1);
        last_command_time_      = command_time;

        std::array<double, 4> wheel_rps{};

        for (size_t i = 0; i < std::min<size_t> (4, wheel_names_.size ()); ++i) {
            double velocity = 0.0;
            if (!get_command_velocity (*msg, i, velocity)) {
                RCLCPP_WARN_THROTTLE (get_logger (), *get_clock (), 1000, "command message has no velocity for %s", wheel_names_[i].c_str ());
                continue;
            }

            if (!std::isfinite (velocity)) {
                RCLCPP_ERROR_THROTTLE (get_logger (), *get_clock (), 1000, "non-finite command velocity for %s", wheel_names_[i].c_str ());
                send_stop ();
                return;
            }

            wheel_rps[i] = input_unit_ == "radps" ? velocity / (2.0 * M_PI) : velocity;
        }

        if (command_mode_ == "closed_loop_velocity") {
            for (size_t i = 0; i < 4; ++i) {
                const double rps                         = std::clamp (wheel_rps[i], -max_rps_, max_rps_);
                command_msg_.data.mc[i].target_wheel_rps = static_cast<float> (rps * static_cast<double> (wheel_polarities_[i]));
            }
        } else {
            double largest_abs_rps = 0.0;
            for (const double rps : wheel_rps) {
                largest_abs_rps = std::max (largest_abs_rps, std::abs (rps));
            }

            const double normalization_scale = std::max (open_loop_full_scale_rps_, largest_abs_rps);
            const double max_delta           = open_loop_slew_rate_per_sec_ * command_dt;

            std::array<double, 4> desired_outputs{};
            double                largest_output_change = 0.0;
            for (size_t i = 0; i < 4; ++i) {
                desired_outputs[i]    = open_loop_output_limit_ * wheel_rps[i] / normalization_scale * static_cast<double> (wheel_polarities_[i]);
                largest_output_change = std::max (largest_output_change, std::abs (desired_outputs[i] - last_open_loop_outputs_[i]));
            }

            double slew_scale = 1.0;
            if (largest_output_change > max_delta && largest_output_change > 0.0) {
                slew_scale = max_delta / largest_output_change;
            }

            for (size_t i = 0; i < 4; ++i) {
                const double output                      = last_open_loop_outputs_[i] + (desired_outputs[i] - last_open_loop_outputs_[i]) * slew_scale;
                last_open_loop_outputs_[i]               = std::clamp (output, -1.0, 1.0);
                command_msg_.data.mc[i].target_wheel_rps = static_cast<float> (last_open_loop_outputs_[i]);
            }
        }

        write_command ();
    }

    bool get_command_velocity (const sensor_msgs::msg::JointState &msg, const size_t wheel_index, double &velocity) const {
        if (msg.name.size () == msg.velocity.size ()) {
            const auto name_it = std::find (msg.name.begin (), msg.name.end (), wheel_names_[wheel_index]);
            if (name_it != msg.name.end ()) {
                const size_t index = static_cast<size_t> (std::distance (msg.name.begin (), name_it));
                velocity           = msg.velocity[index];
                return true;
            }
        }

        if (wheel_index < msg.velocity.size ()) {
            velocity = msg.velocity[wheel_index];
            return true;
        }

        return false;
    }

    void timer_callback () {
        const auto current_time = now ();

        if (!serial_connected_) {
            const auto elapsed_ms = (current_time - last_reconnect_attempt_).nanoseconds () / 1000000;

            if (elapsed_ms >= reconnect_interval_ms_) {
                last_reconnect_attempt_ = current_time;
                try_open_serial ();
            }

            publish_diagnostics ();
            return;
        }

        try {
            serial_->update ();
        } catch (const std::exception &e) {
            RCLCPP_ERROR (get_logger (), "Serial update failed: %s", e.what ());
            close_serial ();
            publish_diagnostics ();
            return;
        } catch (...) {
            RCLCPP_ERROR (get_logger (), "Serial update failed with an unknown error");
            close_serial ();
            publish_diagnostics ();
            return;
        }

        const auto command_elapsed_ms = (current_time - last_command_time_).nanoseconds () / 1000000;

        if (command_elapsed_ms > timeout_ms_) {
            send_stop ();
        }

        if (feedback_msg_.was_updated ()) {
            double dt = 0.0;

            if (feedback_received_) {
                dt = (current_time - last_feedback_stamp_).seconds ();
            }

            last_feedback_stamp_      = current_time;
            last_feedback_time_       = current_time;
            feedback_received_        = true;
            feedback_timeout_latched_ = false;

            publish_joint_state (dt, current_time);
            publish_feedback ();
        }

        if (feedback_received_) {
            const auto feedback_elapsed_ms = (current_time - last_feedback_stamp_).nanoseconds () / 1000000;

            if (feedback_elapsed_ms > feedback_timeout_ms_) {
                if (!feedback_timeout_latched_) {
                    RCLCPP_ERROR (get_logger (), "Motor feedback timed out: %ld ms", feedback_elapsed_ms);

                    send_stop ();
                    feedback_timeout_latched_ = true;
                }
            }
        }

        publish_diagnostics ();
    }

    void publish_diagnostics () {
        diagnostic_msgs::msg::DiagnosticArray array;
        const auto                            current_time = now ();
        array.header.stamp                                 = current_time;

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name        = "natto_robot_hardware/serial_motor_bridge";
        status.hardware_id = serial_path_;

        const double command_age  = (current_time - last_command_time_).seconds ();
        const double feedback_age = feedback_received_ ? (current_time - last_feedback_stamp_).seconds () : -1.0;

        if (!serial_connected_) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Serial disconnected";
        } else if (feedback_timeout_latched_) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Feedback timeout";
        } else if (command_age > static_cast<double> (timeout_ms_) / 1000.0) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Command timeout";
        } else {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "OK";
        }

        auto add_value = [&status] (const std::string &key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue key_value;
            key_value.key   = key;
            key_value.value = value;
            status.values.push_back (key_value);
        };

        add_value ("connected", serial_connected_ ? "true" : "false");
        add_value ("command_age_sec", std::to_string (command_age));
        add_value ("feedback_age_sec", std::to_string (feedback_age));
        add_value ("pid_enabled", pid ? "true" : "false");
        add_value ("command_mode", command_mode_);
        if (command_mode_ == "open_loop_normalized") {
            add_value ("open_loop_output_limit", std::to_string (open_loop_output_limit_));
            for (size_t i = 0; i < 4; ++i) {
                add_value ("open_loop_command_" + std::to_string (i), std::to_string (command_msg_.data.mc[i].target_wheel_rps));
            }
        }

        array.status.push_back (status);
        diagnostics_pub_->publish (array);
    }

    void publish_joint_state (double dt, const rclcpp::Time &stamp) {
        sensor_msgs::msg::JointState msg;

        msg.header.stamp = stamp;
        msg.name         = wheel_names_;

        msg.position.resize (4);
        msg.velocity.resize (4);
        msg.effort.resize (4);

        const bool integrate = std::isfinite (dt) && dt > 0.0 && dt <= max_feedback_dt_sec_;

        for (size_t i = 0; i < 4; ++i) {
            const double rps   = feedback_msg_.data.mfd[i].measured_wheel_rps * static_cast<double> (encoder_polarities_[i]);
            const double radps = rps * 2.0 * M_PI;

            if (integrate) {
                last_position_rad_[i] += radps * dt;
            }

            msg.position[i] = last_position_rad_[i];
            msg.velocity[i] = radps;
            msg.effort[i]   = 0.0;
        }

        joint_state_pub_->publish (msg);
    }

    void publish_feedback () {
        for (size_t i = 0; i < feedback_pubs_.size (); ++i) {
            const auto &feedback = feedback_msg_.data.mfd[i];

            debug_msgs::msg::Feedback msg;
            msg.output        = feedback.output;
            msg.current_rps   = feedback.measured_wheel_rps;
            msg.encoder_count = feedback.encoder_count;
            msg.duty          = feedback.duty;
            msg.dir           = feedback.dir;
            msg.target_rps    = command_msg_.data.mc[i].target_wheel_rps;

            feedback_pubs_[i]->publish (msg);
        }
    }

    void send_stop () {
        set_zero_command ();
        write_command ();
    }

    std::string serial_path_;
    std::string command_topic_;
    std::string command_mode_;
    std::string joint_state_topic_;
    std::string input_unit_;
    std::string pid_gain_config_path_;

    bool direction[4];
    bool pid;
    bool stop_mode_;

    float kp[4];
    float ki[4];
    float kd[4];

    double  open_loop_full_scale_rps_;
    double  open_loop_output_limit_;
    double  open_loop_slew_rate_per_sec_;
    double  max_rps_;
    int64_t timeout_ms_;
    int64_t reconnect_interval_ms_;
    int64_t command_frame_id_;
    int64_t feedback_frame_id_;

    double  max_feedback_dt_sec_;
    int64_t feedback_timeout_ms_;

    bool feedback_received_{false};
    bool feedback_timeout_latched_{false};
    bool serial_connected_{false};

    std::vector<std::string> wheel_names_;
    std::vector<int64_t>     wheel_polarities_;
    std::vector<int64_t>     encoder_polarities_;

    LinuxHardwareSerial *dev_{nullptr};
    SerialBridge        *serial_{nullptr};

    sb::Message<MotorCommandPacket>  command_msg_;
    sb::Message<MotorFeedbackPacket> feedback_msg_;

    std::array<double, 4> last_open_loop_outputs_{};
    double                last_position_rad_[4]{};

    rclcpp::Time last_command_time_;
    rclcpp::Time last_feedback_time_;
    rclcpp::Time last_feedback_stamp_;
    rclcpp::Time last_reconnect_attempt_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr        command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr           joint_state_pub_;
    std::vector<rclcpp::Publisher<debug_msgs::msg::Feedback>::SharedPtr> feedback_pubs_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr  diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr                                         timer_;
};

int main (int argc, char **argv) {
    rclcpp::init (argc, argv);

    try {
        rclcpp::spin (std::make_shared<SerialMotorBridge> ());
    } catch (const std::exception &e) {
        std::cerr << "serial_motor_bridge error: " << e.what () << std::endl;
    }

    rclcpp::shutdown ();
    return 0;
}
