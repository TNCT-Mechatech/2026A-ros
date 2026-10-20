#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "natto_robot_hardware/pid_gain_path.hpp"
#include "natto_robot_hardware/serial_initialization.hpp"
#include "param_checker/measurement_guard.hpp"
#include "param_checker/position_vrft.hpp"
#include "yaml-cpp/yaml.h"

#include "debug_msgs/msg/pid_gain.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int64.hpp"

#include <poll.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using namespace natto_serial;

class PositionCheck : public rclcpp::Node {
   public:
    PositionCheck () : Node ("param_check") {
        operation_ = declare_parameter<std::string> ("position_operation", "tune");
        if (operation_ != "tune" && operation_ != "verify") {
            throw std::invalid_argument ("position_operation must be tune or verify");
        }
        motor_ = declare_parameter<int> ("target_motor_index", 0);
        if (motor_ < 0 || motor_ >= static_cast<int> (MOTORS)) {
            throw std::invalid_argument ("target_motor_index must be 0..3");
        }
        gain_path_        = natto_robot_hardware::resolve_pid_gain_path (declare_parameter<std::string> ("gain_output_path", ""));
        const auto source = declare_parameter<std::string> ("gain_input_path", "");
        gain_input_path_  = source.empty () ? gain_path_ : natto_robot_hardware::resolve_pid_gain_path (source);
        csv_path_         = declare_parameter<std::string> ("vrft_csv_path", (std::filesystem::path (gain_path_).parent_path () / ("vrft_position_wheel" + std::to_string (motor_) + ".csv")).string ());
        if (!csv_path_.empty ()) {
            const auto csv = std::filesystem::absolute (csv_path_).lexically_normal ();
            if (csv == gain_path_ || csv == gain_input_path_) {
                throw std::invalid_argument ("vrft_csv_path must differ from gain YAML paths");
            }
        }
        serial_path_               = declare_parameter<std::string> ("serial_path", "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF565653756687235135-if02");
        duration_                  = positive ("measurement_seconds", 30.0);
        tau_                       = positive ("position_response_time_seconds", 2.0);
        filter_window_             = positive ("position_filter_window_seconds", 0.1);
        max_rps_                   = positive ("position_measurement_max_rps", 0.03);
        range_counts_              = declare_parameter<int64_t> ("position_range_counts", 1000);
        axis_                      = declare_parameter<int> ("axis_y", 1);
        invert_axis_               = declare_parameter<bool> ("invert_axis", false);
        deadband_                  = declare_parameter<double> ("joy_deadband", 0.05);
        minimum_fit_               = declare_parameter<double> ("minimum_fit_r_squared", 0.7);
        minimum_samples_           = declare_parameter<int> ("minimum_samples", 20);
        const int timeout          = declare_parameter<int> ("timeout_ms", 300);
        const int feedback_timeout = declare_parameter<int> ("feedback_timeout_ms", 300);
        if (range_counts_ <= 0 || range_counts_ > 1000000000 || axis_ < 0 || !std::isfinite (deadband_) || deadband_ < 0.0 || deadband_ >= 1.0 || !std::isfinite (minimum_fit_) || minimum_fit_ < 0.0 || minimum_fit_ > 1.0 || minimum_samples_ < 3 ||
            timeout <= 0 || feedback_timeout <= 0) {
            throw std::invalid_argument ("invalid position range, axis, fit or timeout setting");
        }
        command_timeout_         = std::chrono::milliseconds (timeout);
        feedback_timeout_        = std::chrono::milliseconds (feedback_timeout);
        const auto shared_travel = declare_parameter<int64_t> ("measurement_range_counts", 0);
        const auto travel        = declare_parameter<int64_t> ("position_measurement_range_counts", shared_travel);
        const int  switch_index  = declare_parameter<int> ("limit_switch_index", -1);
        const bool switch_active = declare_parameter<bool> ("limit_switch_active_state", true);
        require_neutral_         = declare_parameter<bool> ("require_neutral_on_start", false);
        if (travel < 0 || switch_index < -1 || switch_index >= static_cast<int> (MOTORS)) throw std::invalid_argument ("invalid measurement travel/switch limit");
        if (operation_ == "verify" && travel > 0 && range_counts_ >= travel) throw std::invalid_argument ("position_range_counts must be smaller than position_measurement_range_counts");
        const bool stop_mode = declare_parameter<bool> ("stop_mode", false);
        declare_initialization (*this, frames_, 0, stop_mode, 1.5, {-1, 1, -1, 1});
        load_gains ();  // Fail before opening hardware if the inner loop is unavailable.
        auto &init  = frames_.init[motor_].data;
        init.vel_kp = velocity_kp_;
        init.vel_ki = velocity_ki_;
        init.vel_kd = 0.0f;
        init.pos_kp = position_kp_;
        init.pos_ki = init.pos_kd = 0.0f;
        init.max_motor_rps        = max_rps_;
        init.pid_state            = 1;
        // Start at zero velocity, not at a stale absolute position on the STM.
        for (size_t i = 0; i < MOTORS; ++i) {
            frames_.init[i].data.vel_kd = 0.0f;
            frames_.init[i].data.pos_ki = frames_.init[i].data.pos_kd = 0.0f;
        }
        tuner_     = std::make_unique<param_checker::PositionVrft> (param_checker::PositionVrft::Config{tau_, 4.0 * init.encoder_ppr * init.gear_rate, filter_window_});
        gain_pub_  = create_publisher<debug_msgs::msg::PidGain> ("position_pid_gain" + std::to_string (motor_), 10);
        count_pub_ = create_publisher<std_msgs::msg::Int64> ("encoder_raw_count" + std::to_string (motor_), 10);
        joy_       = create_subscription<sensor_msgs::msg::Joy> ("controller/joy", 10, [this] (const sensor_msgs::msg::Joy::SharedPtr msg) {
            if (!frames_.configuration.ready ()) {
                axis_value_ = 0.0;
                joy_valid_  = false;
                return;
            }
            if (axis_ >= static_cast<int> (msg->axes.size ()) || !std::isfinite (msg->axes[axis_])) {
                axis_value_ = 0.0;
                joy_valid_  = false;
                return;
            }
            axis_value_ = std::clamp (static_cast<double> (msg->axes[axis_]), -1.0, 1.0);
            if (std::abs (axis_value_) < deadband_) {
                axis_value_ = 0.0;
            }
            if (invert_axis_) {
                axis_value_ = -axis_value_;
            }
            last_joy_  = Clock::now ();
            joy_valid_ = true;
        });
        device_    = std::make_unique<LinuxHardwareSerial> (serial_path_.c_str (), B230400);
        if (device_->diagnostic_fd () < 0) {
            throw std::runtime_error ("serial open failed");
        }
        serial_ = std::make_unique<SerialBridge> (device_.get ());
        frames_.register_all (*serial_);
        frames_.initialize (*serial_);
        write_command (false);
        opened_ = Clock::now ();
        guard_  = std::make_unique<param_checker::MeasurementGuard> (opened_, feedback_timeout_, travel, switch_index, switch_active);
        timer_  = create_wall_timer (10ms, [this] { tick (); });
        RCLCPP_INFO (
            get_logger (),
            "Position %s: wheel%d, inner PI=(%.6g, %.6g, 0), limit=%.6g rps (%.3f counts/s), "
            "verify range=+/- %lld counts, travel limit=+/- %lld counts, response=%.3fs. Waiting for feedback.",
            operation_.c_str (), motor_, velocity_kp_, velocity_ki_, max_rps_, max_rps_ * 4.0 * init.encoder_ppr * init.gear_rate, static_cast<long long> (range_counts_), static_cast<long long> (travel), tau_);
        if (operation_ == "tune")
            RCLCPP_INFO (
                get_logger (),
                "Position P identification uses the saved velocity PI; Joy controls velocity, not position. "
                "Response-model fit: %.3fs count windows, first 60%% training / last 40%% validation.",
                filter_window_);
    }

    ~PositionCheck () override {
        for (int i = 0; i < 5; ++i) {
            stop ();
            std::this_thread::sleep_for (10ms);
        }
    }

   private:
    using Clock = std::chrono::steady_clock;
    double positive (const std::string &name, double fallback) {
        const double value = declare_parameter<double> (name, fallback);
        if (!std::isfinite (value) || value <= 0.0 || value > std::numeric_limits<float>::max ()) {
            throw std::invalid_argument (name + " must be finite and positive");
        }
        return value;
    }

    static double gain (const YAML::Node &node, const char *key) {
        if (!node[key]) {
            throw std::runtime_error (std::string ("missing gain: ") + key);
        }
        const double value = node[key].as<double> ();
        if (!std::isfinite (value) || value < 0.0 || value > std::numeric_limits<float>::max ()) {
            throw std::runtime_error (std::string ("invalid gain: ") + key);
        }
        return value;
    }

    void load_gains () {
        const auto config = YAML::LoadFile (gain_input_path_);
        const auto wheel  = config["wheel" + std::to_string (motor_)];
        if (!wheel || !wheel.IsMap ()) {
            throw std::runtime_error ("Run velocity tuning first: missing wheel gains");
        }
        const YAML::Node velocity = wheel["velocity"] ? wheel["velocity"] : wheel;
        velocity_kp_              = gain (velocity, "kp");
        velocity_ki_              = gain (velocity, "ki");
        if (velocity_kp_ <= 0.0) {
            throw std::runtime_error ("velocity kp must be positive");
        }
        if (velocity["kd"]) {
            const double stored_kd = velocity["kd"].as<double> ();
            if (!std::isfinite (stored_kd)) {
                throw std::runtime_error ("velocity kd must be finite");
            }
            if (stored_kd != 0.0) {
                RCLCPP_WARN (get_logger (), "Saved velocity kd is ignored; hardware kd is fixed to 0.");
            }
        }
        if (wheel["velocity"]) {
            require_units (velocity, "rps", "duty");
            if (!velocity["controller"] || velocity["controller"].as<std::string> () != "PI") {
                throw std::runtime_error ("velocity.controller must be PI");
            }
        }
        if (wheel["encoder"]) {
            const auto  encoder = wheel["encoder"];
            const auto &init    = frames_.init[motor_].data;
            if (encoder["pulses_per_revolution"].as<int> () != init.encoder_ppr || std::abs (encoder["gear_ratio"].as<double> () - init.gear_rate) > 1e-5 || (encoder["polarity"] && encoder["polarity"].as<int> () != (init.set_direction ? -1 : 1)))
                throw std::runtime_error ("velocity encoder calibration changed; repeat velocity tuning");
        }
        max_rps_ = std::min (max_rps_, static_cast<double> (frames_.init[motor_].data.max_motor_rps));
        if (velocity["max_rps"]) {
            const double limit = gain (velocity, "max_rps");
            if (limit <= 0.0) {
                throw std::runtime_error ("saved max_rps must be positive");
            }
            max_rps_ = std::min (max_rps_, limit);
        }
        if (operation_ == "tune" && velocity["response_time_seconds"]) {
            const double inner_tau = gain (velocity, "response_time_seconds");
            if (inner_tau <= 0.0 || tau_ <= inner_tau) {
                throw std::runtime_error ("position response time must exceed velocity response time");
            }
            if (tau_ < 3.0 * inner_tau) {
                RCLCPP_WARN (get_logger (), "Position response time is below 3x the velocity response time.");
            }
        }
        if (operation_ == "verify") {
            const auto position = wheel["position"];
            require_units (position, "encoder_count", "rps");
            if (!position["controller"] || position["controller"].as<std::string> () != "P") {
                throw std::runtime_error ("position.controller must be P; run position tuning first");
            }
            position_kp_ = gain (position, "kp");
            if (position_kp_ <= 0.0 || gain (position, "ki") != 0.0 || gain (position, "kd") != 0.0) {
                throw std::runtime_error ("position gains must be Kp>0, Ki=Kd=0");
            }
            if (position["max_rps"]) {
                const double limit = gain (position, "max_rps");
                if (limit <= 0.0) {
                    throw std::runtime_error ("position max_rps must be positive");
                }
                max_rps_ = std::min (max_rps_, limit);
            }
            // A count-based gain cannot be reused after changing encoder scale/sign.
            if (position["encoder_ppr"] && position["gear_rate"] && position["encoder_polarity"]) {
                const auto &init = frames_.init[motor_].data;
                if (position["encoder_ppr"].as<int> () != init.encoder_ppr || std::abs (position["gear_rate"].as<double> () - init.gear_rate) > 1e-5 || position["encoder_polarity"].as<int> () != (init.set_direction ? -1 : 1)) {
                    throw std::runtime_error ("encoder calibration changed; repeat position tuning");
                }
            }
        }
    }

    static void require_units (const YAML::Node &node, const char *input, const char *output) {
        if (!node || !node["input_unit"] || !node["output_unit"] || node["input_unit"].as<std::string> () != input || node["output_unit"].as<std::string> () != output) {
            throw std::runtime_error ("gain file has missing or incompatible controller units");
        }
    }

    void write_command (bool active) {
        active = active && frames_.configuration.ready ();
        for (size_t i = 0; i < MOTORS; ++i) {
            if (i == static_cast<size_t> (motor_) && active && operation_ == "verify") {
                const long double target = static_cast<long double> (origin_) + std::round (axis_value_ * range_counts_);
                if (target < std::numeric_limits<int64_t>::min () || target > std::numeric_limits<int64_t>::max ()) {
                    throw std::runtime_error ("position target overflows int64");
                }
                frames_.position[i].data.target_pos = static_cast<int64_t> (target);
                SerialFrames::write (*serial_, CMD_POS_FRAME_ID_BASE + i, frames_.position[i]);
            } else if (i == static_cast<size_t> (motor_) && operation_ == "tune") {
                frames_.velocity[i].data.target_motor_rps = active ? axis_value_ * max_rps_ : 0.0;
                SerialFrames::write (*serial_, CMD_VEL_FRAME_ID_BASE + i, frames_.velocity[i]);
                last_input_rps_ = frames_.velocity[i].data.target_motor_rps;
            } else {
                frames_.duty[i].data.duty = 0.0f;
                SerialFrames::write (*serial_, CMD_DUTY_FRAME_ID_BASE + i, frames_.duty[i]);
            }
        }
    }

    void stop () noexcept {
        if (!serial_) {
            return;
        }
        try {
            // tune stops in velocity mode; verify disengages position hold with zero duty.
            write_command (false);
        } catch (const std::exception &e) {
            RCLCPP_ERROR (get_logger (), "Position stop failed: %s", e.what ());
        }
    }

    void tick () {
        pollfd descriptor{device_->diagnostic_fd (), POLLIN, 0};
        if (::poll (&descriptor, 1, 0) < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            throw std::runtime_error ("serial disconnected during position measurement");
        }
        const auto instant = Clock::now ();
        if (finishing_) {
            stop ();
            // Repeat zero commands over several STM cycles when leaving position mode.
            // STM rejects two command families received in the same control cycle.
            if (instant - finished_at_ >= 50ms) {
                timer_->cancel ();
                rclcpp::shutdown ();
            }
            return;
        }
        const auto received = frames_.drain (*serial_);
        frames_.service_initialization (*serial_);
        if (configuration_started_ && received.configuration_lost) {
            stop ();
            throw std::runtime_error ("STM configuration changed during position measurement; gains not saved");
        }
        if (!frames_.configuration.ready ()) {
            stop ();
            return;
        }
        if (!configuration_started_) {
            configuration_started_ = true;
            guard_->start_feedback_deadline (instant);
            RCLCPP_INFO (get_logger (), "All STM configurations confirmed: session_id=%u config_id=%u", frames_.configuration.session_id (), frames_.configuration.config_id (0));
        }
        if (received.motor_updated[motor_]) {
            const auto &data = frames_.feedback[motor_].data;
            if (!std::isfinite (data.measured_wheel_rps)) {
                throw std::runtime_error ("non-finite motor feedback");
            }
            guard_->motor (instant, data.encoder_raw_count);
            last_feedback_ = instant;
            std_msgs::msg::Int64 count;
            count.data = data.encoder_raw_count;
            count_pub_->publish (count);
            if (operation_ == "tune" && started_) {
                tuner_->update (last_input_rps_, data.encoder_raw_count, std::chrono::duration<double> (instant - started_at_).count ());
            }
        }
        if (received.switches_updated && guard_->switch_index () >= 0) guard_->switches (instant, frames_.switches.data.state[guard_->switch_index ()]);
        guard_->check (instant);
        const bool joy_fresh = joy_valid_ && instant - last_joy_ < command_timeout_;
        if (!joy_fresh) neutral_seen_ = false;
        if (guard_->ready () && joy_fresh && axis_value_ == 0) neutral_seen_ = true;
        if (!started_ && guard_->ready () && (!require_neutral_ || neutral_seen_)) {
            started_    = true;
            started_at_ = instant;
            origin_     = frames_.feedback[motor_].data.encoder_raw_count;
            RCLCPP_INFO (get_logger (), "Position %s started: origin=%lld counts, duration=%.2fs", operation_.c_str (), static_cast<long long> (origin_), duration_);
        }
        if (started_ && std::chrono::duration<double> (instant - started_at_).count () >= duration_) {
            stop ();
            if (operation_ == "tune") {
                finish ();
            } else {
                RCLCPP_INFO (get_logger (), "Position verification finished; motor stopped.");
            }
            finishing_   = true;
            finished_at_ = instant;
            return;
        }
        const bool active = started_ && guard_->ready () && joy_fresh && (!require_neutral_ || neutral_seen_);
        write_command (active);  // 100 Hz refresh, with independent Joy/feedback deadlines.
    }

    bool save_gain (const param_checker::PositionVrft::Result &result) {
        const auto destination = std::filesystem::path (gain_path_);
        const auto temporary   = destination.string () + ".position.tmp";
        try {
            // Preserve legacy flat velocity gains and every other wheel's settings.
            YAML::Node config = YAML::LoadFile (std::filesystem::exists (destination) ? gain_path_ : gain_input_path_);
            if (gain_input_path_ != gain_path_ && std::filesystem::exists (destination)) {
                const auto source     = YAML::LoadFile (gain_input_path_);
                const auto wheel_name = "wheel" + std::to_string (motor_);
                // The saved cascade must contain the inner gains used for identification.
                config[wheel_name] = YAML::Clone (source[wheel_name]);
            }
            auto position                              = config["wheel" + std::to_string (motor_)]["position"];
            position["controller"]                     = "P";
            position["kp"]                             = result.kp;
            position["ki"]                             = 0.0;
            position["kd"]                             = 0.0;
            position["input_unit"]                     = "encoder_count";
            position["output_unit"]                    = "rps";
            position["response_time_seconds"]          = tau_;
            position["max_rps"]                        = max_rps_;
            position["fit_r_squared"]                  = result.r_squared;
            position["identification_method"]          = "velocity_response_model_v1";
            position["training_fit_r_squared"]         = result.training_r_squared;
            position["validation_fit_r_squared"]       = result.validation_r_squared;
            position["raw_vrft_fit_r_squared"]         = result.raw_r_squared;
            position["velocity_response_time_seconds"] = result.velocity_response_s;
            position["velocity_delay_seconds"]         = result.delay_s;
            position["velocity_steady_gain"]           = result.velocity_gain;
            position["filter_window_seconds"]          = filter_window_;
            position["fit_windows"]                    = result.used_samples;
            const auto &init                           = frames_.init[motor_].data;
            position["encoder_ppr"]                    = init.encoder_ppr;
            position["gear_rate"]                      = static_cast<double> (init.gear_rate);
            position["encoder_polarity"]               = init.set_direction ? -1 : 1;
            std::filesystem::create_directories (destination.parent_path ());
            {
                std::ofstream stream (temporary);
                stream << std::setprecision (17) << config << '\n';
                stream.flush ();
                if (!stream) {
                    throw std::runtime_error ("gain YAML write failed");
                }
            }
            std::filesystem::rename (temporary, destination);
            return true;
        } catch (const std::exception &e) {
            std::error_code ignored;
            std::filesystem::remove (temporary, ignored);
            RCLCPP_ERROR (get_logger (), "Position gain save failed: %s", e.what ());
            return false;
        }
    }

    void save_csv () {
        if (csv_path_.empty ()) {
            return;
        }
        try {
            const auto parent = std::filesystem::absolute (csv_path_).parent_path ();
            std::filesystem::create_directories (parent);
            std::ofstream stream (csv_path_);
            stream << "wheel,time_sec,encoder_raw_count,relative_position_count,input_rps,virtual_error_count,used_for_regression,"
                      "model_mean_input_rps,model_measured_rps,model_predicted_rps,model_residual_rps,model_partition\n";
            stream << std::setprecision (17);
            const auto &models      = tuner_->model_samples ();
            size_t      model_index = 0, source_index = 0;
            for (const auto &sample : tuner_->samples ()) {
                stream << motor_ << ',' << sample.time << ',' << sample.raw_count << ',' << sample.relative_count << ',' << sample.input_rps << ',' << sample.virtual_error << ',' << sample.used;
                if (model_index < models.size () && models[model_index].source_index == source_index) {
                    const auto &m = models[model_index++];
                    stream << ',' << m.input_rps << ',' << m.measured_rps << ',' << m.predicted_rps << ',' << m.measured_rps - m.predicted_rps << ',' << (m.training ? "training" : "validation");
                } else
                    stream << ",,,,,";
                stream << '\n';
                ++source_index;
            }
            stream.flush ();
            if (!stream) {
                throw std::runtime_error ("CSV write failed");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR (get_logger (), "Position CSV save failed: %s", e.what ());
        }
    }

    void finish () {
        const auto  result      = tuner_->result ();
        const float hardware_kp = static_cast<float> (result.kp);
        const bool  valid =
            result.model_valid && std::isfinite (result.kp) && hardware_kp > 0.0f && std::isfinite (hardware_kp) && result.used_samples >= static_cast<size_t> (minimum_samples_) && std::isfinite (result.r_squared) && result.r_squared >= minimum_fit_;
        const bool saved = valid && save_gain (result);
        save_csv ();
        if (saved) {
            debug_msgs::msg::PidGain msg;
            msg.kp = hardware_kp;
            msg.ki = msg.kd = 0.0f;
            gain_pub_->publish (msg);
        }
        std::ostringstream summary;
        summary << "\n位置PIDゲインの決定値 (速度応答モデル / P制御)\n"
                << "対象     | Kp [rps/count] | Ki | Kd | 判定\n"
                << "wheel" << motor_ << "   | " << std::scientific << std::setprecision (6) << result.kp << "   | 0  | 0  | " << (saved ? "保存済み" : "保存しませんでした") << '\n'
                << "適合度 R^2: " << std::fixed << std::setprecision (4) << result.r_squared << " （必要値 " << minimum_fit_ << "）、有効窓数: " << result.used_samples << '\n'
                << "モデル R^2: 学習=" << result.training_r_squared << ", 検証=" << result.validation_r_squared << "、旧VRFT R^2=" << result.raw_r_squared << '\n'
                << "推定速度応答: 時定数=" << result.velocity_response_s << "s, 遅延=" << result.delay_s << "s, 定常ゲイン=" << result.velocity_gain << '\n'
                << "モデル判定: " << result.reason << '\n'
                << "速度PI: Kp=" << velocity_kp_ << ", Ki=" << velocity_ki_ << ", Kd=0\n"
                << "PID YAML: " << gain_path_ << '\n';
        if (!valid) {
            summary << "測定の前半・後半とも入力を正負に変化させ、モデル判定・検証R^2を確認してください。既存のYAMLは更新していません。\n";
        }
        if (saved) {
            RCLCPP_INFO (get_logger (), "%s", summary.str ().c_str ());
        } else {
            RCLCPP_WARN (get_logger (), "%s", summary.str ().c_str ());
        }
    }

    std::string                                            operation_, gain_path_, gain_input_path_, csv_path_, serial_path_;
    int                                                    motor_{0}, axis_{1}, minimum_samples_{20};
    int64_t                                                range_counts_{1000}, origin_{0};
    double                                                 duration_{30}, tau_{2}, max_rps_{0.03}, deadband_{0.05}, minimum_fit_{0.7};
    double                                                 filter_window_{0.1};
    double                                                 velocity_kp_{0}, velocity_ki_{0}, position_kp_{0}, axis_value_{0}, last_input_rps_{0};
    bool                                                   invert_axis_{false}, joy_valid_{false}, started_{false}, finishing_{false};
    bool                                                   require_neutral_{false}, neutral_seen_{false};
    std::unique_ptr<param_checker::MeasurementGuard>       guard_;
    Clock::time_point                                      opened_{}, started_at_{}, last_joy_{}, last_feedback_{}, finished_at_{};
    std::chrono::milliseconds                              command_timeout_{300}, feedback_timeout_{300};
    SerialFrames                                           frames_;
    bool                                                   configuration_started_{false};
    std::unique_ptr<LinuxHardwareSerial>                   device_;
    std::unique_ptr<SerialBridge>                          serial_;
    std::unique_ptr<param_checker::PositionVrft>           tuner_;
    rclcpp::TimerBase::SharedPtr                           timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_;
    rclcpp::Publisher<debug_msgs::msg::PidGain>::SharedPtr gain_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr     count_pub_;
};

int main (int argc, char **argv) {
    rclcpp::init (argc, argv);
    int result = 0;
    try {
        rclcpp::spin (std::make_shared<PositionCheck> ());
    } catch (const std::exception &e) {
        std::cerr << "position_checker error: " << e.what () << '\n';
        result = 1;
    }
    if (rclcpp::ok ()) {
        rclcpp::shutdown ();
    }
    return result;
}
