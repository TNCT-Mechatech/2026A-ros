#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "natto_robot_hardware/pid_gain_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <natto_robot_hardware/serial_initialization.hpp>
#include "vrft_auto_tuner.hpp"
#include "param_checker/measurement_guard.hpp"
#include "param_checker/velocity_gain_store.hpp"
#include <memory>
#include <poll.h>
#include <thread>
#include "yaml-cpp/yaml.h"

#include "debug_msgs/msg/feedback.hpp"
#include "debug_msgs/msg/pid_gain.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int64.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std::literals::chrono_literals;
using namespace natto_serial;

class ParamCheck : public rclcpp::Node {
   public:
    ParamCheck () : Node ("param_check") {
        serial_path_          = declare_parameter<std::string> ("serial_path", "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF565653756687235135-if02");
        pid_gain_output_path_ = natto_robot_hardware::resolve_pid_gain_path (declare_parameter<std::string> ("gain_output_path", ""));
        set_parameter (rclcpp::Parameter ("gain_output_path", pid_gain_output_path_));
        const auto output_directory = std::filesystem::path (pid_gain_output_path_).parent_path ();
        vrft_csv_path_              = declare_parameter<std::string> ("vrft_csv_path", (output_directory / "vrft_param_checker.csv").string ());
        serial_diagnostic_csv_path_ = declare_parameter<std::string> ("serial_diagnostic_csv_path", (output_directory / "vrft_param_checker_serial.csv").string ());
        RCLCPP_INFO (get_logger (), "PID gain output path: %s", pid_gain_output_path_.c_str ());

        stop_mode_ = declare_parameter<bool> ("stop_mode", false);

        initial_kp_ = declare_parameter<double> ("initial_kp", 0.001);
        initial_ki_ = declare_parameter<double> ("initial_ki", 1.8);
        initial_kd_ = declare_parameter<double> ("initial_kd", 0.0);

        if (!std::isfinite(initial_kp_) || !std::isfinite(initial_ki_) || !std::isfinite(initial_kd_))
            throw std::invalid_argument("initial PID gains must be finite");

        max_wheel_rps_         = declare_parameter<double> ("max_wheel_rps", 1.5);
        measurement_seconds_   = declare_parameter<double> ("measurement_seconds", 15.0);
        response_time_seconds_ = declare_parameter<double> ("response_time_seconds", 0.3);
        sample_time_seconds_   = declare_parameter<double> ("sample_time_seconds", 0.01);
        joy_deadband_          = declare_parameter<double> ("joy_deadband", 0.05);

        axis_x_    = static_cast<int> (declare_parameter<int> ("axis_x", 0));
        axis_y_    = static_cast<int> (declare_parameter<int> ("axis_y", 1));
        axis_turn_ = static_cast<int> (declare_parameter<int> ("axis_turn", 2));

        target_motor_ = declare_parameter<int> ("target_motor_index", -1);
        inspect_switches_ = declare_parameter<bool>("inspect_switches", false);
        if (inspect_switches_ && target_motor_ < 0)
            throw std::invalid_argument("switch inspection requires a target_motor_index");
        max_duty_ = declare_parameter<double> ("measurement_max_duty", 0.1);
        single_max_rps_ = declare_parameter<double> ("measurement_max_rps", 0.03);
        invert_axis_ = declare_parameter<bool> ("invert_axis", false);
        minimum_samples_ = declare_parameter<int> ("minimum_samples", 20);
        minimum_fit_ = declare_parameter<double> ("minimum_fit_r_squared", 0.7);
        if (target_motor_ < -1 || target_motor_ >= static_cast<int>(MOTORS) ||
            !std::isfinite(max_duty_) || max_duty_ <= 0 || max_duty_ > 1 ||
            !std::isfinite(single_max_rps_) || single_max_rps_ <= 0 ||
            !std::isfinite(measurement_seconds_) || measurement_seconds_ <= 0 ||
            !std::isfinite(response_time_seconds_) || response_time_seconds_ <= 0 ||
            !std::isfinite(sample_time_seconds_) || sample_time_seconds_ <= 0 ||
            !std::isfinite(joy_deadband_) || joy_deadband_ < 0 || joy_deadband_ >= 1 ||
            !std::isfinite(minimum_fit_) || minimum_fit_ < 0 || minimum_fit_ > 1 ||
            minimum_samples_ < 3 || axis_y_ < 0)
            throw std::invalid_argument("invalid single-motor measurement settings");
        const bool pid_enabled = declare_parameter<bool> ("pid", false);
        if (target_motor_ >= 0 && pid_enabled)
            throw std::invalid_argument("single-motor PI identification requires pid=false (bounded Duty input)");
        pid_state_.fill (pid_enabled);

        declare_initialization (*this, frames_, pid_enabled ? 1 : 0, stop_mode_,
                                max_wheel_rps_, {-1, 1, -1, 1});
        command_timeout_ = std::chrono::milliseconds(declare_parameter<int>("timeout_ms", 300));
        if (command_timeout_.count() <= 0) throw std::invalid_argument("timeout_ms must be positive");
        const auto feedback_timeout = std::chrono::milliseconds(declare_parameter<int>("feedback_timeout_ms", 300));
        const auto travel = declare_parameter<int64_t>("measurement_range_counts", 0);
        const int switch_index = declare_parameter<int>("limit_switch_index", -1);
        const bool switch_active = declare_parameter<bool>("limit_switch_active_state", true);
        if (target_motor_ >= 0) {
            guard_ = std::make_unique<param_checker::MeasurementGuard>(
                std::chrono::steady_clock::now(), feedback_timeout, travel, switch_index, switch_active);
            auto &init = frames_.init[target_motor_].data;
            single_max_rps_ = std::min(single_max_rps_, static_cast<double>(init.max_motor_rps));
            init.max_motor_rps = static_cast<float>(single_max_rps_);
            // Measure in a continuous coordinate; no automatic origin resetting.
            for (auto &packet : frames_.init) {
                packet.data.pid_state = 0;
                packet.data.reset_encoder_origin = false;
                packet.data.reset_encoder_origin_with_switch = false;
            }
            for (const auto &csv : {vrft_csv_path_, serial_diagnostic_csv_path_}) {
                if (!csv.empty() && std::filesystem::absolute(csv).lexically_normal() == pid_gain_output_path_)
                    throw std::invalid_argument("CSV path must differ from gain YAML path");
            }
            RCLCPP_INFO(get_logger(), "Single motor %d: axis=%d, max Duty=%.3f, saved speed limit=%.4f rps",
                        target_motor_, axis_y_, max_duty_, single_max_rps_);
            RCLCPP_INFO(get_logger(), "Encoder calibration: motor=%d, PPR=%u, gear=%.9g, counts/rev=%.9g, polarity=%d",
                        target_motor_, static_cast<unsigned>(init.encoder_ppr), init.gear_rate,
                        static_cast<double>(init.encoder_ppr) * 4 * init.gear_rate,
                        init.set_direction ? -1 : 1);
        }

        for (size_t i = 0; i < vrft_.size (); i++) {
            feedback_.push_back (create_publisher<debug_msgs::msg::Feedback> ("feedback" + std::to_string (i), 10));
            encoder_raw_count_.push_back(create_publisher<std_msgs::msg::Int64>(
                "encoder_raw_count" + std::to_string(i), 10));
            pid_gain_.push_back (create_publisher<debug_msgs::msg::PidGain> ("pid_gain" + std::to_string (i), 10));
            vrft_[i].timeStep (static_cast<float> (sample_time_seconds_));
            vrft_[i].responseTime (static_cast<float> (response_time_seconds_));
        }

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy> ("controller/joy", 10, std::bind (&ParamCheck::joy_callback, this, std::placeholders::_1));

        open_serial ();
        frames_.register_all (*serial_);

        target_wheel_rps_.resize (4, 0.0f);
        start_measurement ();

        timer_ptr_ = create_wall_timer (10ms, std::bind (&ParamCheck::timer_callback, this));
    }

    ~ParamCheck () override {
        for (int i = 0; i < (target_motor_ >= 0 ? 5 : 1); ++i) {
            send_stop ();
            if (target_motor_ >= 0) std::this_thread::sleep_for(10ms);
        }

        serial_.reset();
        dev_.reset();
    }

   private:
    enum class Phase {
        Measure,
        Diagnose,
        Finished,
    };

    struct SerialDiagnosticSample {
        size_t timer_index;
        double elapsed_sec;
        double timer_dt_sec;
        double update_duration_sec;
        int    update_result;
        bool   feedback_updated;
        size_t command_write_count;
        size_t command_writes_since_last_tick;
        int    last_command_write_result;
    };

    void start_measurement () {
        reset_vrft_samples ();
        apply_initial_gain ();
        phase_start_time_       = now ();
        countdown_seconds_left_ = -1;
        phase_                  = Phase::Measure;

        if (inspect_switches_) {
            RCLCPP_INFO(get_logger(), "Input inspection: all motors held at Duty 0. Compare switch inputs 0..3 and /encoder_raw_count0..3 for encoders. Ctrl+C exits; gains are not changed.");
            return;
        }
        RCLCPP_INFO (get_logger (), "VRFT measurement started: initial PID=(%.4f, %.4f, %.4f), duration=%.2fs", initial_kp_, initial_ki_, initial_kd_, measurement_seconds_);
        if (!vrft_csv_path_.empty ()) {
            RCLCPP_INFO (get_logger (), "VRFT CSV output path: %s", vrft_csv_path_.c_str ());
        }
        if (!serial_diagnostic_csv_path_.empty ()) {
            RCLCPP_INFO (get_logger (), "Serial diagnostic CSV output path: %s", serial_diagnostic_csv_path_.c_str ());
        }
        if (target_motor_ < 0) publish_countdown ();
        else RCLCPP_INFO(get_logger(), "Waiting for target feedback and neutral Joy before measurement");
    }

    void reset_vrft_samples () {
        for (size_t i = 0; i < vrft_.size (); i++) {
            vrft_[i].clear ();
            vrft_sample_times_[i].clear ();
            vrft_target_rps_samples_[i].clear ();
        }

        serial_diagnostic_samples_.clear ();
        timer_sample_index_                = 0;
        previous_timer_elapsed_sec_        = 0.0;
        has_previous_timer_sample_         = false;
        command_write_count_               = 0;
        last_recorded_command_write_count_ = 0;
        last_command_write_result_         = 0;
    }

    void apply_initial_gain () {
        for (size_t i = 0; i < MOTORS; ++i) {
            auto &init = frames_.init[i].data;
            init.vel_kp = initial_kp_;
            init.vel_ki = initial_ki_;
            init.vel_kd = target_motor_ >= 0 ? 0.0 : initial_kd_;
        }
        if (serial_) frames_.initialize (*serial_);
        send_stop ();
    }

    void write_targets () {
        if (!serial_) return;
        if (!frames_.configuration.ready())
            std::fill(target_wheel_rps_.begin(), target_wheel_rps_.end(), 0.0f);
        for (size_t i = 0; i < MOTORS; ++i) {
            if (target_motor_ >= 0 && (i != static_cast<size_t>(target_motor_) ||
                !single_started_ || !neutral_seen_)) {
                frames_.duty[i].data.duty = 0.0f;
                last_command_write_result_ = SerialFrames::write(*serial_, CMD_DUTY_FRAME_ID_BASE+i, frames_.duty[i]);
            } else if (pid_state_[i]) {
                frames_.velocity[i].data.target_motor_rps = target_wheel_rps_[i];
                last_command_write_result_ = SerialFrames::write (*serial_, CMD_VEL_FRAME_ID_BASE+i, frames_.velocity[i]);
            } else {
                frames_.duty[i].data.duty = std::clamp(target_wheel_rps_[i], -1.0f, 1.0f);
                last_command_write_result_ = SerialFrames::write (*serial_, CMD_DUTY_FRAME_ID_BASE+i, frames_.duty[i]);
            }
            ++command_write_count_;
        }
    }

    void send_stop () {
        std::fill(target_wheel_rps_.begin(), target_wheel_rps_.end(), 0.0f);
        try { write_targets (); }
        catch (const std::exception &e) { RCLCPP_ERROR(get_logger(), "Stop write failed: %s", e.what()); }
    }

    void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (!frames_.configuration.ready()) { send_stop(); return; }
        if (phase_ != Phase::Measure) {
            return;
        }

        if (target_motor_ >= 0) {
            joy_valid_ = static_cast<size_t>(axis_y_) < msg->axes.size() && std::isfinite(msg->axes[axis_y_]);
            axis_value_ = joy_valid_ ? std::clamp(read_axis(*msg, axis_y_), -1.0f, 1.0f) : 0.0f;
            if (invert_axis_) axis_value_ = -axis_value_;
            if (!joy_valid_) neutral_seen_ = false;
            last_joy_time_ = std::chrono::steady_clock::now();
            return; // The timer is the single sender for this mode.
        }
        const float x    = read_axis (*msg, axis_x_);
        const float y    = -1 * read_axis (*msg, axis_y_);
        const float turn = read_axis (*msg, axis_turn_);

        set_mecanum_target (x, y, turn);
        last_joy_time_ = std::chrono::steady_clock::now();
        try { write_targets (); }
        catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Command write failed: %s", e.what());
            send_stop();
        }
    }

    float read_axis (const sensor_msgs::msg::Joy &msg, int axis) const {
        if (axis < 0 || static_cast<size_t> (axis) >= msg.axes.size ()) {
            return 0.0f;
        }

        const float value = msg.axes[static_cast<size_t> (axis)];
        if (!std::isfinite(value) || std::fabs (value) < joy_deadband_) {
            return 0.0f;
        }

        return value;
    }

    void set_mecanum_target (float x, float y, float turn) {
        std::array<float, 4> raw = {
            -1 * (y + x + turn),
            y - x - turn,
            -1 * (y - x + turn),
            y + x - turn,
        };

        float max_abs = 1.0f;
        for (const auto value : raw) {
            max_abs = std::max (max_abs, std::fabs (value));
        }

        for (size_t i = 0; i < target_wheel_rps_.size (); i++) {
            if (pid_state_[i]) {
                target_wheel_rps_[i] = raw[i] / max_abs * static_cast<float> (max_wheel_rps_);
            } else {
                target_wheel_rps_[i] = std::clamp(raw[i], -1.0f, 1.0f);  // signed duty
            }
            // if (i != 1) target_wheel_rps_[i] = 0.0f;  // 1のみでテスト
            // if (i == 1) target_wheel_rps_[i] = 0.0f;
        }
    }

    void timer_callback () {
        if (serial_ == nullptr || phase_ == Phase::Finished) {
            return;
        }

        if (target_motor_ >= 0) {
            single_motor_tick();
            return;
        }
        const auto callback_start   = now ();
        const auto received = frames_.drain (*serial_);
        if (!confirm_configuration(received)) return;
        const int update_result = received.status;
        const auto update_end       = now ();
        const bool feedback_updated = std::any_of(received.motor_updated.begin(), received.motor_updated.end(), [](bool v) { return v; });
        // Keep the ROS command deadline and STM 300ms timeout as independent stops.
        if (std::chrono::steady_clock::now() - last_joy_time_ >= command_timeout_) send_stop();

        if (phase_ == Phase::Measure) {
            record_serial_diagnostic (callback_start, update_end, update_result, feedback_updated);
        }

        if (feedback_updated) {
            publish_feedback (received.motor_updated);

            if (phase_ == Phase::Measure) {
                record_vrft_sample (received.motor_updated);
            }
        }

        if (phase_ == Phase::Measure) {
            publish_countdown ();

            if ((now () - phase_start_time_).seconds () >= measurement_seconds_) {
                finish_diagnosis ();
            }
        }
    }

    void record_serial_diagnostic (const rclcpp::Time &callback_start, const rclcpp::Time &update_end, int update_result, bool feedback_updated) {
        const double elapsed_sec                    = (callback_start - phase_start_time_).seconds ();
        const double timer_dt_sec                   = has_previous_timer_sample_ ? elapsed_sec - previous_timer_elapsed_sec_ : 0.0;
        const size_t command_writes_since_last_tick = command_write_count_ - last_recorded_command_write_count_;

        serial_diagnostic_samples_.push_back ({
            timer_sample_index_,
            elapsed_sec,
            timer_dt_sec,
            (update_end - callback_start).seconds (),
            update_result,
            feedback_updated,
            command_write_count_,
            command_writes_since_last_tick,
            last_command_write_result_,
        });

        timer_sample_index_++;
        previous_timer_elapsed_sec_        = elapsed_sec;
        has_previous_timer_sample_         = true;
        last_recorded_command_write_count_ = command_write_count_;
    }

    static const char *serial_update_status (int result) {
        switch (result) {
            case 0:
                return "updated";
            case -1:
                return "no_complete_packet";
            case -2:
                return "invalid_packet";
            case -3:
                return "unregistered_frame_id";
            default:
                return "unknown";
        }
    }

    void publish_countdown () {
        const double elapsed = target_motor_ >= 0 ?
            std::chrono::duration<double>(std::chrono::steady_clock::now() - single_started_at_).count() :
            (now () - phase_start_time_).seconds();
        const double remaining = measurement_seconds_ - elapsed;
        const int    seconds_left = std::max (0, static_cast<int> (std::ceil (remaining)));

        if (seconds_left == countdown_seconds_left_) {
            return;
        }

        countdown_seconds_left_ = seconds_left;
        RCLCPP_INFO (get_logger (), "VRFT measurement countdown: %ds remaining", seconds_left);
    }

    void publish_feedback (const std::array<bool, MOTORS> &updated) {
        for (size_t i = 0; i < feedback_.size (); i++) {
            if (!updated[i]) continue;
            debug_msgs::msg::Feedback msg;

            msg.output        = frames_.feedback[i].data.output;
            msg.current_rps   = frames_.feedback[i].data.measured_wheel_rps;
            msg.encoder_count = frames_.feedback[i].data.encoder_count;
            msg.duty          = frames_.feedback[i].data.duty;
            msg.dir           = frames_.feedback[i].data.dir;
            msg.target_rps    = target_wheel_rps_[i];

            feedback_[i]->publish (msg);
            std_msgs::msg::Int64 raw;
            raw.data = frames_.feedback[i].data.encoder_raw_count;
            encoder_raw_count_[i]->publish(raw);
        }
    }

    void record_vrft_sample (const std::array<bool, MOTORS> &updated) {
        const double elapsed_sec = target_motor_ >= 0 ?
            std::chrono::duration<double>(std::chrono::steady_clock::now() - single_started_at_).count() :
            (now () - phase_start_time_).seconds ();

        for (size_t i = 0; i < vrft_.size (); i++) {
            if (!updated[i] || (target_motor_ >= 0 && i != static_cast<size_t>(target_motor_))) continue;
            const float input  = signed_by_dir (frames_.feedback[i].data.duty, frames_.feedback[i].data.dir);
            const float output = frames_.feedback[i].data.measured_wheel_rps;

            if (std::isfinite (input) && std::isfinite (output)) {
                vrft_[i].update (input, output, static_cast<float> (elapsed_sec));
                vrft_sample_times_[i].push_back (elapsed_sec);
                vrft_target_rps_samples_[i].push_back (target_wheel_rps_[i]);
            }
        }
    }

    static float signed_by_dir (float value, bool dir) {
        return std::fabs (value) * (dir ? -1.0f : 1.0f);
    }

    bool confirm_configuration(const DrainResult &received) {
        frames_.service_initialization(*serial_);
        if (configuration_started_ && received.configuration_lost) {
            send_stop();
            throw std::runtime_error("STM configuration changed during measurement; gains not saved");
        }
        if (!frames_.configuration.ready()) { send_stop(); return false; }
        if (!configuration_started_) {
            configuration_started_ = true;
            phase_start_time_ = now();
            if (guard_) guard_->start_feedback_deadline(std::chrono::steady_clock::now());
            RCLCPP_INFO(get_logger(), "All STM configurations confirmed: session_id=%u config_id=%u",
                frames_.configuration.session_id(), frames_.configuration.config_id(0));
        }
        return true;
    }

    void single_motor_tick() {
        pollfd descriptor{dev_->diagnostic_fd(), POLLIN, 0};
        if (::poll(&descriptor, 1, 0) < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
            throw std::runtime_error("serial disconnected during velocity measurement");
        const auto instant = std::chrono::steady_clock::now();
        const auto received = frames_.drain(*serial_);
        if (!confirm_configuration(received)) return;
        if (inspect_switches_) {
            publish_feedback(received.motor_updated);
            if (received.switches_updated) {
                std::array<bool, MOTORS> inputs{};
                for (size_t i = 0; i < MOTORS; ++i) inputs[i] = frames_.switches.data.state[i];
                if (!switch_inspection_seen_ || inputs != inspected_switches_) {
                    RCLCPP_INFO(get_logger(), "Switch inputs: [0]=%s [1]=%s [2]=%s [3]=%s",
                        inputs[0] ? "true" : "false", inputs[1] ? "true" : "false",
                        inputs[2] ? "true" : "false", inputs[3] ? "true" : "false");
                    inspected_switches_ = inputs;
                    switch_inspection_seen_ = true;
                }
            }
            send_stop();
            return;
        }
        if (received.motor_updated[target_motor_]) {
            const auto &data = frames_.feedback[target_motor_].data;
            if (!std::isfinite(data.measured_wheel_rps) || !std::isfinite(data.duty))
                throw std::runtime_error("non-finite target motor feedback");
            guard_->motor(instant, data.encoder_raw_count);
            if (std::abs(data.measured_wheel_rps) > single_max_rps_)
                throw std::runtime_error("measurement speed limit reached; reduce Duty input");
        }
        if (received.switches_updated && guard_->switch_index() >= 0)
            guard_->switches(instant, frames_.switches.data.state[guard_->switch_index()]);
        guard_->check(instant);
        const bool joy_fresh = joy_valid_ && instant - last_joy_time_ < command_timeout_;
        if (!joy_fresh) neutral_seen_ = false;
        if (guard_->ready() && joy_fresh && axis_value_ == 0) neutral_seen_ = true;
        if (!single_started_ && guard_->ready() && neutral_seen_) {
            single_started_ = true;
            phase_start_time_ = now();
            single_started_at_ = instant;
            RCLCPP_INFO(get_logger(), "Single velocity measurement started: motor=%d, duration=%.2fs", target_motor_, measurement_seconds_);
        }
        publish_feedback(received.motor_updated);
        if (single_started_) {
            record_vrft_sample(received.motor_updated);
            publish_countdown();
            if (std::chrono::duration<double>(instant - single_started_at_).count() >= measurement_seconds_) {
                send_stop();
                finish_single_motor();
                return;
            }
        }
        std::fill(target_wheel_rps_.begin(), target_wheel_rps_.end(), 0.0f);
        if (single_started_ && neutral_seen_ && joy_fresh)
            target_wheel_rps_[target_motor_] = axis_value_ * static_cast<float>(max_duty_);
        write_targets();
    }

    void finish_single_motor() {
        phase_ = Phase::Diagnose;
        const auto result = vrft_[target_motor_].getPI();
        const float kp = static_cast<float>(result.kp), ki = static_cast<float>(result.ki);
        const bool valid = std::isfinite(kp) && kp > 0 && std::isfinite(ki) && ki >= 0 &&
            result.used_samples >= static_cast<size_t>(minimum_samples_) &&
            std::isfinite(result.r_squared) && result.r_squared >= minimum_fit_;
        bool saved = false;
        if (valid) {
            try {
                const auto &init = frames_.init[target_motor_].data;
                param_checker::save_velocity_gain(pid_gain_output_path_, target_motor_, kp, ki,
                    response_time_seconds_, single_max_rps_, result.r_squared,
                    init.encoder_ppr, init.gear_rate, init.set_direction ? -1 : 1);
                saved = true;
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Velocity gain save failed: %s", e.what());
            }
        }
        std::array<std::array<float, 3>, 4> gains{};
        gains[target_motor_] = {kp, ki, 0};
        write_vrft_csv(gains);
        if (saved) {
            debug_msgs::msg::PidGain msg; msg.kp = kp; msg.ki = ki; msg.kd = 0;
            pid_gain_[target_motor_]->publish(msg);
            RCLCPP_INFO(get_logger(), "Velocity PI saved: wheel%d kp=%.9g ki=%.9g kd=0, R^2=%.4f, samples=%zu, YAML=%s. Retune target position P.",
                        target_motor_, result.kp, result.ki, result.r_squared, result.used_samples, pid_gain_output_path_.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "Velocity PI not saved: kp=%.9g ki=%.9g, R^2=%.4f, samples=%zu. Existing YAML unchanged.",
                        result.kp, result.ki, result.r_squared, result.used_samples);
        }
        phase_ = Phase::Finished;
        send_stop();
        timer_ptr_->cancel();
        rclcpp::shutdown();
    }

    void finish_diagnosis () {
        phase_ = Phase::Diagnose;

        YAML::Node config;

        RCLCPP_INFO (get_logger (), "VRFT measurement finished. Diagnosing PID gains.");

        std::array<std::array<float, 3>, 4> gains = {};

        for (size_t i = 0; i < vrft_.size (); i++) {
            const auto gain = vrft_[i].getPID ();
            gains[i]        = gain;

            debug_msgs::msg::PidGain msg;
            msg.kp                                     = gain[0];
            msg.ki                                     = gain[1];
            msg.kd                                     = gain[2];
            config["wheel" + std::to_string (i)]["kp"] = gain[0];
            config["wheel" + std::to_string (i)]["ki"] = gain[1];
            config["wheel" + std::to_string (i)]["kd"] = gain[2];
            pid_gain_[i]->publish (msg);
        }
        bool gains_saved = false;
        try {
            std::filesystem::create_directories (std::filesystem::path (pid_gain_output_path_).parent_path ());
            std::ofstream fout;
            fout.exceptions (std::ios::failbit | std::ios::badbit);
            fout.open (pid_gain_output_path_);
            fout << config;
            fout.close ();
            gains_saved = true;
            RCLCPP_INFO (get_logger (), "PID gains saved: %s", pid_gain_output_path_.c_str ());
        } catch (const std::exception &e) {
            RCLCPP_ERROR (get_logger (), "Failed to save PID gains to %s: %s", pid_gain_output_path_.c_str (), e.what ());
        }

        write_vrft_csv (gains);
        write_serial_diagnostic_csv ();

        phase_ = Phase::Finished;
        send_stop ();

        if (timer_ptr_ != nullptr) {
            timer_ptr_->cancel ();
        }

        log_gain_summary (gains, gains_saved);

        if (gains_saved) {
            RCLCPP_INFO (get_logger (), "Diagnosis completed. Shutting down param_checker.");
        } else {
            RCLCPP_ERROR (get_logger (), "Diagnosis finished, but PID gains were not saved. Shutting down param_checker.");
        }
        rclcpp::shutdown ();
    }

    void log_gain_summary (const std::array<std::array<float, 3>, 4> &gains, bool gains_saved) {
        const auto format_gain = [] (float value) {
            std::ostringstream text;
            const float magnitude = std::fabs (value);
            // Keep small or large gains readable without rounding them to zero.
            if ((magnitude > 0.0f && magnitude < 1e-6f) || magnitude >= 1e6f)
                text << std::scientific;
            else
                text << std::fixed;
            text << std::setprecision (6) << value;
            return text.str ();
        };

        std::ostringstream summary;
        summary << "\nPIDゲインの決定値 (VRFT)\n"
                << std::left << std::setw (8) << "Motor" << " | "
                << std::right << std::setw (14) << "kp" << " | "
                << std::setw (14) << "ki" << " | " << std::setw (14) << "kd" << " | 判定\n"
                << std::string (76, '-') << "\n";

        bool needs_attention = !gains_saved;
        for (size_t i = 0; i < gains.size (); ++i) {
            const auto &gain = gains[i];
            const bool finite = std::all_of (gain.begin (), gain.end (), [] (float v) { return std::isfinite (v); });
            const bool all_zero = std::all_of (gain.begin (), gain.end (), [] (float v) { return v == 0.0f; });
            const bool negative = std::any_of (gain.begin (), gain.end (), [] (float v) { return v < 0.0f; });
            const char *status = !finite ? "非有限値（要確認）" : all_zero ? "算出失敗 / データ不足" : negative ? "負のゲイン（極性を確認）" : "OK";
            needs_attention = needs_attention || !finite || all_zero || negative;
            summary << std::left << std::setw (8) << ("wheel" + std::to_string (i)) << " | ";
            for (float value : gain) summary << std::right << std::setw (14) << format_gain (value) << " | ";
            summary << status << "\n";
        }
        summary << std::string (76, '-') << "\n"
                << "PID YAML: " << pid_gain_output_path_ << (gains_saved ? " （保存済み）" : " （保存失敗）") << "\n"
                << "kd列は計算・保存値です。通常運転の車輪ブリッジは vel_kd=0 を送信します。";
        const auto text = summary.str ();
        if (needs_attention) RCLCPP_WARN (get_logger (), "%s", text.c_str ());
        else RCLCPP_INFO (get_logger (), "%s", text.c_str ());
    }

    void write_vrft_csv (const std::array<std::array<float, 3>, 4> &gains) {
        if (vrft_csv_path_.empty ()) {
            RCLCPP_INFO (get_logger (), "VRFT CSV output disabled because vrft_csv_path is empty.");
            return;
        }

        try {
            std::filesystem::create_directories(std::filesystem::absolute(vrft_csv_path_).parent_path());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "VRFT CSV directory failed: %s", e.what());
            return;
        }
        std::ofstream csv (vrft_csv_path_);
        if (!csv.is_open ()) {
            RCLCPP_ERROR (get_logger (), "Failed to open VRFT CSV output: %s", vrft_csv_path_.c_str ());
            return;
        }

        csv << std::fixed << std::setprecision (6);
        csv << "wheel,sample_index,elapsed_sec,vrft_time_sec,sample_dt_sec,"
            << (target_motor_ >= 0 ? "target_duty" : "target_rps")
            << ",input_duty,output_rps,virtual_reference,virtual_error,kp_coefficient,ki_coefficient,kd_coefficient,used_for_regression,result_kp,result_ki,result_kd\n";

        auto write_optional_float = [&csv] (bool enabled, float value) {
            if (enabled) {
                csv << value;
            }
        };

        size_t rows = 0;
        for (size_t wheel = 0; wheel < vrft_.size (); wheel++) {
            const auto samples = vrft_[wheel].diagnosticSamples ();

            for (const auto &sample : samples) {
                const size_t index       = static_cast<size_t> (sample.index);
                const double elapsed_sec = index < vrft_sample_times_[wheel].size () ? vrft_sample_times_[wheel][index] : sample.time;
                const float  target_rps  = index < vrft_target_rps_samples_[wheel].size () ? vrft_target_rps_samples_[wheel][index] : 0.0f;

                csv << wheel << ",";
                csv << sample.index << ",";
                csv << elapsed_sec << ",";
                csv << sample.time << ",";
                csv << sample.sample_dt << ",";
                csv << target_rps << ",";
                csv << sample.input << ",";
                csv << sample.output << ",";
                write_optional_float (sample.has_virtual_values, sample.virtual_reference);
                csv << ",";
                write_optional_float (sample.has_virtual_values, sample.virtual_error);
                csv << ",";
                write_optional_float (sample.used_for_regression, sample.kp_coefficient);
                csv << ",";
                write_optional_float (sample.used_for_regression, sample.ki_coefficient);
                csv << ",";
                write_optional_float (sample.used_for_regression, target_motor_ >= 0 ? 0.0f : sample.kd_coefficient);
                csv << ",";
                csv << (sample.used_for_regression ? 1 : 0) << ",";
                csv << gains[wheel][0] << ",";
                csv << gains[wheel][1] << ",";
                csv << gains[wheel][2] << "\n";
                rows++;
            }
        }

        if (!csv.good ()) {
            RCLCPP_ERROR (get_logger (), "Failed while writing VRFT CSV output: %s", vrft_csv_path_.c_str ());
            return;
        }

        RCLCPP_INFO (get_logger (), "VRFT CSV written: %s (%zu rows)", vrft_csv_path_.c_str (), rows);
    }

    void write_serial_diagnostic_csv () {
        if (serial_diagnostic_csv_path_.empty ()) {
            RCLCPP_INFO (get_logger (), "Serial diagnostic CSV output disabled because serial_diagnostic_csv_path is empty.");
            return;
        }

        std::ofstream csv (serial_diagnostic_csv_path_);
        if (!csv.is_open ()) {
            RCLCPP_ERROR (get_logger (), "Failed to open serial diagnostic CSV output: %s", serial_diagnostic_csv_path_.c_str ());
            return;
        }

        csv << std::fixed << std::setprecision (6);
        csv << "timer_index,elapsed_sec,timer_dt_sec,update_duration_sec,serial_update_result,serial_update_status,feedback_updated,command_write_count,command_writes_since_last_tick,last_command_write_result\n";

        size_t updated_count      = 0;
        size_t feedback_count     = 0;
        size_t no_packet_count    = 0;
        size_t invalid_count      = 0;
        size_t unregistered_count = 0;
        size_t unknown_count      = 0;

        for (const auto &sample : serial_diagnostic_samples_) {
            csv << sample.timer_index << ",";
            csv << sample.elapsed_sec << ",";
            csv << sample.timer_dt_sec << ",";
            csv << sample.update_duration_sec << ",";
            csv << sample.update_result << ",";
            csv << serial_update_status (sample.update_result) << ",";
            csv << (sample.feedback_updated ? 1 : 0) << ",";
            csv << sample.command_write_count << ",";
            csv << sample.command_writes_since_last_tick << ",";
            csv << sample.last_command_write_result << "\n";

            feedback_count += sample.feedback_updated ? 1 : 0;
            switch (sample.update_result) {
                case 0:
                    updated_count++;
                    break;
                case -1:
                    no_packet_count++;
                    break;
                case -2:
                    invalid_count++;
                    break;
                case -3:
                    unregistered_count++;
                    break;
                default:
                    unknown_count++;
                    break;
            }
        }

        if (!csv.good ()) {
            RCLCPP_ERROR (get_logger (), "Failed while writing serial diagnostic CSV output: %s", serial_diagnostic_csv_path_.c_str ());
            return;
        }

        RCLCPP_INFO (
            get_logger (), "Serial diagnostic CSV written: %s (%zu timer ticks, update=%zu, feedback=%zu, no_packet=%zu, invalid=%zu, unregistered=%zu, unknown=%zu)", serial_diagnostic_csv_path_.c_str (), serial_diagnostic_samples_.size (),
            updated_count, feedback_count, no_packet_count, invalid_count, unregistered_count, unknown_count);
    }

    void open_serial () {
        dev_ = std::make_unique<LinuxHardwareSerial>(serial_path_.c_str(), B230400);
        if (dev_->diagnostic_fd() < 0) throw std::runtime_error("serial open failed");
        serial_ = std::make_unique<SerialBridge>(dev_.get());
    }

    bool inspect_switches_{false}, switch_inspection_seen_{false};
    std::array<bool, MOTORS> inspected_switches_{};
    int target_motor_{-1}, minimum_samples_{20};
    double max_duty_{0.1}, single_max_rps_{0.03}, minimum_fit_{0.7};
    float axis_value_{0};
    bool invert_axis_{false}, joy_valid_{false}, neutral_seen_{false}, single_started_{false};
    std::chrono::steady_clock::time_point single_started_at_{};
    std::unique_ptr<param_checker::MeasurementGuard> guard_;

    std::vector<float>  target_wheel_rps_;
    std::array<VRFT, 4> vrft_;

    std::array<std::vector<double>, 4> vrft_sample_times_;
    std::array<std::vector<float>, 4>  vrft_target_rps_samples_;

    std::string serial_path_;
    std::string vrft_csv_path_;
    std::string serial_diagnostic_csv_path_;
    std::string pid_gain_output_path_;

    double initial_kp_;
    double initial_ki_;
    double initial_kd_;
    double max_wheel_rps_;
    double measurement_seconds_;
    double response_time_seconds_;
    double sample_time_seconds_;
    double joy_deadband_;

    std::array<bool, 4> pid_state_{};

    std::vector<SerialDiagnosticSample> serial_diagnostic_samples_;
    size_t                              timer_sample_index_{0};
    double                              previous_timer_elapsed_sec_{0.0};
    bool                                has_previous_timer_sample_{false};
    size_t                              command_write_count_{0};
    size_t                              last_recorded_command_write_count_{0};
    int                                 last_command_write_result_{0};

    bool stop_mode_;

    int axis_x_;
    int axis_y_;
    int axis_turn_;

    Phase phase_{Phase::Finished};
    int   countdown_seconds_left_{-1};

    rclcpp::Time                 phase_start_time_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;

    std::vector<rclcpp::Publisher<debug_msgs::msg::Feedback>::SharedPtr> feedback_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr> encoder_raw_count_;
    std::vector<rclcpp::Publisher<debug_msgs::msg::PidGain>::SharedPtr>  pid_gain_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_{nullptr};


    std::unique_ptr<LinuxHardwareSerial> dev_;
    std::unique_ptr<SerialBridge> serial_;

    SerialFrames frames_;
    bool configuration_started_{false};
    std::chrono::milliseconds command_timeout_{300};
    std::chrono::steady_clock::time_point last_joy_time_{};
};

int main (int argc, char **argv) {
    rclcpp::init (argc, argv);

    int result = 0;
    try {
        rclcpp::spin (std::make_shared<ParamCheck> ());
    } catch (const std::exception &e) {
        std::cerr << "param_check error: " << e.what () << std::endl;
        result = 1;
    }

    if (rclcpp::ok ()) {
        rclcpp::shutdown ();
    }

    return result;
}
