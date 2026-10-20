#include "natto_robot_hardware/pid_gain_path.hpp"
#include "natto_robot_hardware/serial_initialization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include <LinuxHardwareSerial.hpp>
#include <Message.hpp>
#include <SerialBridge.hpp>
#include <natto_system_monitor/serial_restart.hpp>
#include <natto_system_monitor/serial_watchdog.hpp>

#include "debug_msgs/msg/feedback.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

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
using namespace natto_serial;

class SerialMotorBridge : public rclcpp::Node {
   public:
    SerialMotorBridge () : Node ("serial_motor_bridge") {
        serial_path_       = declare_parameter<std::string> ("serial_path", "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF565653756687235135-if02");
        command_topic_     = declare_parameter<std::string> ("command_topic", "/command_joint_states");
        joint_state_topic_ = declare_parameter<std::string> ("joint_state_topic", "/joint_states");

        pid_gain_config_path_ = natto_robot_hardware::resolve_pid_gain_path (declare_parameter<std::string> ("gain_config_path", ""));
        set_parameter (rclcpp::Parameter ("gain_config_path", pid_gain_config_path_));
        RCLCPP_INFO (get_logger (), "PID gain config path: %s", pid_gain_config_path_.c_str ());

        stop_mode_ = declare_parameter<bool> ("stop_mode", false);

        command_mode_                = declare_parameter<std::string> ("command_mode", "closed_loop_velocity");
        input_unit_                  = declare_parameter<std::string> ("input_unit", "radps");
        max_rps_                     = declare_parameter<double> ("max_rps", 4.0);
        open_loop_full_scale_rps_    = declare_parameter<double> ("open_loop_full_scale_rps", 4.0);
        open_loop_output_limit_      = declare_parameter<double> ("open_loop_output_limit", 1.0);
        open_loop_slew_rate_per_sec_ = declare_parameter<double> ("open_loop_slew_rate_per_sec", 1.0);
        timeout_ms_                  = declare_parameter<int64_t> ("timeout_ms", 300);
        reconnect_interval_ms_       = declare_parameter<int64_t> ("reconnect_interval_ms", 1000);

        if (reconnect_interval_ms_ <= 0) throw std::invalid_argument ("reconnect_interval_ms must be positive");


        std::vector<std::string> default_names = {"mecanum_wheel_fl", "mecanum_wheel_fr", "mecanum_wheel_rl", "mecanum_wheel_rr"};
        wheel_names_                           = declare_parameter<std::vector<std::string>> ("wheel_names", default_names);
        wheel_polarities_                      = declare_parameter<std::vector<int64_t>> ("wheel_polarities", {-1, 1, -1, 1});
        encoder_polarities_                    = declare_parameter<std::vector<int64_t>> ("encoder_polarities", {-1, -1, -1, -1});

        if (wheel_names_.size() != MOTORS) throw std::invalid_argument("wheel_names must contain four names");

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

        declare_initialization (*this, frames_, pid ? 1 : 0, stop_mode_, max_rps_, {-1, -1, -1, -1});

        command_sub_ = create_subscription<sensor_msgs::msg::JointState> (command_topic_, rclcpp::SensorDataQoS (), std::bind (&SerialMotorBridge::command_callback, this, std::placeholders::_1));

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState> (joint_state_topic_, rclcpp::SensorDataQoS ());

        for (size_t i = 0; i < 4; ++i) {
            feedback_pubs_.push_back (create_publisher<debug_msgs::msg::Feedback> ("feedback" + std::to_string (i), 10));
        }

        switch_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray> ("switch_feedback", 10);

        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray> ("/diagnostics", 10);

        feedback_timeout_ms_ = declare_parameter<int64_t> ("feedback_timeout_ms", 200);
        watchdog_            = std::make_unique<Watchdog> (
            std::chrono::milliseconds (feedback_timeout_ms_), std::chrono::milliseconds (declare_parameter<int64_t> ("startup_feedback_timeout_ms", 1000)), std::chrono::milliseconds (timeout_ms_),
            declare_parameter<int> ("feedback_recovery_count", 3));

        restart_policy_ = std::make_unique<natto_system_monitor::SerialRestartPolicy> (
            declare_parameter<bool> ("auto_restart", false), declare_parameter<int> ("restart_failure_count", 1), std::chrono::milliseconds (declare_parameter<int64_t> ("restart_timeout_ms", 1000)),
            std::chrono::milliseconds (declare_parameter<int64_t> ("restart_recovery_ms", 3000)));

        // Retain the existing parameter for launch compatibility; raw counts need no dt.
        declare_parameter<double> ("max_feedback_dt_sec", 0.1);


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
        last_reconnect_attempt_ = Watchdog::Clock::now ();

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

        serial_connected_       = false;
        last_reconnect_attempt_ = Watchdog::Clock::now ();
        set_zero_command ();
    }

    bool try_open_serial () {
        close_serial ();

        ++reconnect_attempts_;
        try {
            dev_    = new LinuxHardwareSerial (serial_path_.c_str (), B230400);
            serial_ = new SerialBridge (dev_);

            initialize_command_packet ();
            frames_.reset_received ();
            frames_.register_all (*serial_);
            feedback_seen_.fill (false);
            feedback_cohort_.fill (false);
            frames_.initialize (*serial_);

            serial_connected_  = true;
            watchdog_->reset (Watchdog::Clock::now ());
            watchdog_state_ = Watchdog::State::WaitingFeedback;

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

        if (!frames_.configuration.ready() || watchdog_->check (Watchdog::Clock::now ()) != Watchdog::State::Ready) {
            set_zero_command ();
        }
        try {
            for (size_t i = 0; i < MOTORS; ++i) {
                if (pid) SerialFrames::write (*serial_, CMD_VEL_FRAME_ID_BASE + i, frames_.velocity[i]);
                else SerialFrames::write (*serial_, CMD_DUTY_FRAME_ID_BASE + i, frames_.duty[i]);
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
        for (size_t i = 0; i < MOTORS; ++i) {
            auto &packet = frames_.init[i].data;
            if (!std::isfinite(kp[i]) || !std::isfinite(ki[i]) || !std::isfinite(kd[i]))
                throw std::invalid_argument("velocity PID gains must be finite");
            packet.vel_kp = pid ? kp[i] : 0.0f;
            packet.vel_ki = pid ? ki[i] : 0.0f;
            // Keep derivative control disabled as in the original hardware bridge.
            // YAML kd remains available for calibration/storage, but is not sent.
            packet.vel_kd = 0.0f;
        }
        set_zero_command ();
    }

    void set_zero_command () {
        for (size_t i = 0; i < MOTORS; ++i) {
            frames_.velocity[i].data.target_motor_rps = 0.0f;
            frames_.duty[i].data.duty = 0.0f;
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
                send_stop ();
                return;
            }

            if (!std::isfinite (velocity)) {
                RCLCPP_ERROR_THROTTLE (get_logger (), *get_clock (), 1000, "non-finite command velocity for %s", wheel_names_[i].c_str ());
                send_stop ();
                return;
            }

            wheel_rps[i] = input_unit_ == "radps" ? velocity / (2.0 * M_PI) : velocity;
        }

        const bool neutral = std::all_of (wheel_rps.begin (), wheel_rps.end (), [] (double value) { return value == 0.0; });
        if (!frames_.configuration.ready()) {
            send_stop();
            return;
        }
        if (!watchdog_->command (Watchdog::Clock::now (), neutral)) {
            send_stop ();
            return;
        }

        if (command_mode_ == "closed_loop_velocity") {
            for (size_t i = 0; i < 4; ++i) {
                const double rps                         = std::clamp (wheel_rps[i], -max_rps_, max_rps_);
                frames_.velocity[i].data.target_motor_rps = static_cast<float> (rps * static_cast<double> (wheel_polarities_[i]));
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
                frames_.duty[i].data.duty = static_cast<float> (last_open_loop_outputs_[i]);
                frames_.velocity[i].data.target_motor_rps = static_cast<float> (wheel_rps[i] * wheel_polarities_[i]);
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
        const auto restart_time          = Watchdog::Clock::now ();
        const auto restart_state         = watchdog_->check (restart_time);
        const bool communication_healthy = serial_connected_ && restart_state != Watchdog::State::WaitingFeedback && restart_state != Watchdog::State::FeedbackTimeout && restart_state != Watchdog::State::Recovering;
        if (restart_policy_->observe (restart_time, communication_healthy)) {
            RCLCPP_ERROR (get_logger (), "Persistent serial communication failure: requesting process restart");
            send_stop ();
            close_serial ();
            rclcpp::shutdown ();
            return;
        }
        const auto current_time = now ();
        const auto steady_time  = Watchdog::Clock::now ();

        if (!serial_connected_) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds> (steady_time - last_reconnect_attempt_).count ();

            if (elapsed_ms >= reconnect_interval_ms_) {
                last_reconnect_attempt_ = steady_time;
                try_open_serial ();
            }

            publish_diagnostics ();
            return;
        }

        DrainResult received;
        try {
            received = frames_.drain (*serial_);
            frames_.service_initialization(*serial_);
            invalid_packets_ += received.invalid;
            unregistered_frames_ += received.unregistered;
            drain_budget_hits_ += received.budget_exhausted;
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

        const auto received_time = Watchdog::Clock::now ();
        if (received.configuration_lost) {
            watchdog_->reset(received_time);
            feedback_seen_.fill(false);
            feedback_cohort_.fill(false);
            set_zero_command();
            RCLCPP_WARN(get_logger(), "STM configuration changed/reset; stopping and reinitializing");
        }
        bool any_feedback = false;
        for (size_t i = 0; i < MOTORS; ++i) {
            if (received.motor_updated[i]) {
                feedback_seen_[i] = feedback_cohort_[i] = true;
                any_feedback = true;
            }
        }
        // A single live motor or switch must not hide a missing motor.
        if (std::all_of(feedback_cohort_.begin(), feedback_cohort_.end(), [](bool v) { return v; })) {
            watchdog_->feedback (received_time);
            feedback_cohort_.fill (false);
        }
        if (any_feedback) {
            if (frames_.configuration.ready() && std::all_of(feedback_seen_.begin(), feedback_seen_.end(), [](bool v) { return v; }))
                publish_joint_state (current_time);
            publish_feedback (received.motor_updated);
        }
        if (received.switches_updated) {
            std_msgs::msg::UInt8MultiArray msg;
            for (bool state : frames_.switches.data.state) msg.data.push_back(state);
            switch_pub_->publish(msg);
        }

        const auto state = watchdog_->check (received_time);
        if (state != watchdog_state_) {
            RCLCPP_WARN (get_logger (), "Serial watchdog: %s", Watchdog::name (state));
            watchdog_state_ = state;
        }
        if (state != Watchdog::State::Ready) send_stop ();
        if (state == Watchdog::State::FeedbackTimeout) close_serial ();

        publish_diagnostics ();
    }

    void publish_diagnostics () {
        diagnostic_msgs::msg::DiagnosticArray array;
        const auto                            current_time = now ();
        array.header.stamp                                 = current_time;

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name        = "natto_robot_hardware/serial_motor_bridge";
        status.hardware_id = serial_path_;

        const auto   steady_time  = Watchdog::Clock::now ();
        const auto   state        = watchdog_->check (steady_time);
        const double command_age  = watchdog_->command_age (steady_time);
        const double feedback_age = watchdog_->feedback_age (steady_time);

        if (frames_.configuration.failed()) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = frames_.configuration.error();
        } else if (!serial_connected_) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Serial disconnected";
        } else if (state == Watchdog::State::FeedbackTimeout) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Feedback timeout";
        } else if (!frames_.configuration.ready()) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = frames_.configuration.state();
        } else if (state != Watchdog::State::Ready) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = Watchdog::name (state);
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

        add_value ("invalid_packets", std::to_string(invalid_packets_));
        add_value ("unregistered_frames", std::to_string(unregistered_frames_));
        add_value ("drain_budget_hits", std::to_string(drain_budget_hits_));
        add_value ("stm_comm_timeout_ms", std::to_string(COMM_TIMEOUT_MS));
        add_value ("reconnect_attempts", std::to_string (reconnect_attempts_));
        add_value ("watchdog_state", Watchdog::name (state));
        add_value ("connected", serial_connected_ ? "true" : "false");
        add_value ("command_age_sec", std::to_string (command_age));
        add_value ("feedback_age_sec", std::to_string (feedback_age));
        add_value ("pid_enabled", pid ? "true" : "false");
        add_value ("command_mode", command_mode_);
        add_value("configuration_state", frames_.configuration.state());
        add_value("configuration_ready", frames_.configuration.ready() ? "true" : "false");
        add_value("session_id", std::to_string(frames_.configuration.session_id()));
        for (size_t i = 0; i < MOTORS; ++i) {
            const auto suffix = std::to_string(i);
            add_value("expected_config_id_" + suffix, std::to_string(frames_.configuration.config_id(i)));
            add_value("applied_session_id_" + suffix, std::to_string(frames_.configuration.applied_session_id(i)));
            add_value("applied_config_id_" + suffix, std::to_string(frames_.configuration.applied_config_id(i)));
            add_value("configuration_confirmed_" + suffix, frames_.configuration.confirmed(i) ? "true" : "false");
            add_value("initialization_attempts_" + suffix, std::to_string(frames_.configuration.attempts(i)));
            add_value("initialization_result_" + suffix, std::to_string(frames_.configuration.result(i)));
        }
        if (command_mode_ == "open_loop_normalized") {
            add_value ("open_loop_output_limit", std::to_string (open_loop_output_limit_));
            for (size_t i = 0; i < 4; ++i) {
                add_value ("open_loop_command_" + std::to_string (i), std::to_string (frames_.duty[i].data.duty));
            }
        }

        array.status.push_back (status);
        diagnostics_pub_->publish (array);
    }

    void publish_joint_state (const rclcpp::Time &stamp) {
        sensor_msgs::msg::JointState msg;

        msg.header.stamp = stamp;
        msg.name         = wheel_names_;

        msg.position.resize (4);
        msg.velocity.resize (4);
        msg.effort.resize (4);


        for (size_t i = 0; i < 4; ++i) {
            const auto &feedback = frames_.feedback[i].data;
            const double polarity = static_cast<double>(wheel_polarities_[i]);
            const double radps = feedback.measured_wheel_rps * polarity * 2.0 * M_PI;
            const auto &init = frames_.init[i].data;
            const double cpr = static_cast<double>(init.encoder_ppr) * 4.0 * init.gear_rate;
            // STM corrects the encoder for its PID motor coordinates. Convert
            // back to ROS wheel coordinates with the inverse command polarity.
            msg.position[i] = static_cast<double>(feedback.encoder_raw_count) / cpr * polarity * 2.0 * M_PI;
            msg.velocity[i] = radps;
            msg.effort[i]   = 0.0;
        }

        joint_state_pub_->publish (msg);
    }

    void publish_feedback (const std::array<bool, MOTORS> &updated) {
        for (size_t i = 0; i < feedback_pubs_.size (); ++i) {
            if (!updated[i]) continue;
            const auto &feedback = frames_.feedback[i].data;

            debug_msgs::msg::Feedback msg;
            msg.output        = feedback.output;
            msg.current_rps   = feedback.measured_wheel_rps;
            msg.encoder_count = feedback.encoder_count;
            msg.duty          = feedback.duty;
            msg.dir           = feedback.dir;
            msg.target_rps    = frames_.velocity[i].data.target_motor_rps;

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

    int64_t feedback_timeout_ms_;

    using Watchdog = natto_system_monitor::SerialWatchdog;
    std::unique_ptr<Watchdog>                                  watchdog_;
    std::unique_ptr<natto_system_monitor::SerialRestartPolicy> restart_policy_;
    Watchdog::State                                            watchdog_state_{Watchdog::State::WaitingFeedback};
    bool                                                       serial_connected_{false};

    std::vector<std::string> wheel_names_;
    std::vector<int64_t>     wheel_polarities_;
    std::vector<int64_t>     encoder_polarities_;

    LinuxHardwareSerial *dev_{nullptr};
    SerialBridge        *serial_{nullptr};

    SerialFrames frames_;
    std::array<bool, MOTORS> feedback_seen_{};
    std::array<bool, MOTORS> feedback_cohort_{};
    uint64_t invalid_packets_{0}, unregistered_frames_{0}, drain_budget_hits_{0};

    std::array<double, 4> last_open_loop_outputs_{};

    rclcpp::Time   last_command_time_;
    Watchdog::Time last_reconnect_attempt_;
    uint64_t       reconnect_attempts_{0};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr        command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr           joint_state_pub_;
    std::vector<rclcpp::Publisher<debug_msgs::msg::Feedback>::SharedPtr> feedback_pubs_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr  diagnostics_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr switch_pub_;
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
