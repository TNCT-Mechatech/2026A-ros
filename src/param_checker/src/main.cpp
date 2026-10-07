#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "natto_robot_hardware/pid_gain_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <natto_robot_hardware/serial_initialization.hpp>
#include "vrft_auto_tuner.hpp"
#include "yaml-cpp/yaml.h"

#include "debug_msgs/msg/feedback.hpp"
#include "debug_msgs/msg/pid_gain.hpp"
#include "sensor_msgs/msg/joy.hpp"

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
        measurement_seconds_   = declare_parameter<double> ("measurement_seconds", 10.0);
        response_time_seconds_ = declare_parameter<double> ("response_time_seconds", 0.3);
        sample_time_seconds_   = declare_parameter<double> ("sample_time_seconds", 0.01);
        joy_deadband_          = declare_parameter<double> ("joy_deadband", 0.05);

        axis_x_    = static_cast<int> (declare_parameter<int> ("axis_x", 0));
        axis_y_    = static_cast<int> (declare_parameter<int> ("axis_y", 1));
        axis_turn_ = static_cast<int> (declare_parameter<int> ("axis_turn", 2));

        const bool pid_enabled = declare_parameter<bool> ("pid", false);
        pid_state_.fill (pid_enabled);

        declare_initialization (*this, frames_, pid_enabled ? 1 : 0, stop_mode_,
                                max_wheel_rps_, {-1, 1, -1, 1});
        command_timeout_ = std::chrono::milliseconds(declare_parameter<int>("timeout_ms", 300));
        if (command_timeout_.count() <= 0) throw std::invalid_argument("timeout_ms must be positive");

        for (size_t i = 0; i < vrft_.size (); i++) {
            feedback_.push_back (create_publisher<debug_msgs::msg::Feedback> ("feedback" + std::to_string (i), 10));
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
        send_stop ();

        delete serial_;
        delete dev_;
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

        RCLCPP_INFO (get_logger (), "VRFT measurement started: initial PID=(%.4f, %.4f, %.4f), duration=%.2fs", initial_kp_, initial_ki_, initial_kd_, measurement_seconds_);
        if (!vrft_csv_path_.empty ()) {
            RCLCPP_INFO (get_logger (), "VRFT CSV output path: %s", vrft_csv_path_.c_str ());
        }
        if (!serial_diagnostic_csv_path_.empty ()) {
            RCLCPP_INFO (get_logger (), "Serial diagnostic CSV output path: %s", serial_diagnostic_csv_path_.c_str ());
        }
        publish_countdown ();
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
            init.vel_kd = initial_kd_;
        }
        if (serial_) frames_.initialize (*serial_);
        send_stop ();
    }

    void write_targets () {
        if (!serial_) return;
        for (size_t i = 0; i < MOTORS; ++i) {
            if (pid_state_[i]) {
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
        if (phase_ != Phase::Measure) {
            return;
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

        const auto callback_start   = now ();
        const auto received = frames_.drain (*serial_);
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
        const double remaining    = measurement_seconds_ - (now () - phase_start_time_).seconds ();
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
        }
    }

    void record_vrft_sample (const std::array<bool, MOTORS> &updated) {
        const double elapsed_sec = (now () - phase_start_time_).seconds ();

        for (size_t i = 0; i < vrft_.size (); i++) {
            if (!updated[i]) continue;
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

        std::ofstream csv (vrft_csv_path_);
        if (!csv.is_open ()) {
            RCLCPP_ERROR (get_logger (), "Failed to open VRFT CSV output: %s", vrft_csv_path_.c_str ());
            return;
        }

        csv << std::fixed << std::setprecision (6);
        csv << "wheel,sample_index,elapsed_sec,vrft_time_sec,sample_dt_sec,target_rps,input_duty,output_rps,virtual_reference,virtual_error,kp_coefficient,ki_coefficient,kd_coefficient,used_for_regression,result_kp,result_ki,result_kd\n";

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
                write_optional_float (sample.used_for_regression, sample.kd_coefficient);
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
        dev_    = new LinuxHardwareSerial (serial_path_.c_str (), B230400);
        serial_ = new SerialBridge (dev_);
    }

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
    std::vector<rclcpp::Publisher<debug_msgs::msg::PidGain>::SharedPtr>  pid_gain_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_{nullptr};


    LinuxHardwareSerial *dev_{nullptr};
    SerialBridge        *serial_{nullptr};

    SerialFrames frames_;
    std::chrono::milliseconds command_timeout_{300};
    std::chrono::steady_clock::time_point last_joy_time_{};
};

int main (int argc, char **argv) {
    rclcpp::init (argc, argv);

    try {
        rclcpp::spin (std::make_shared<ParamCheck> ());
    } catch (const std::exception &e) {
        std::cerr << "param_check error: " << e.what () << std::endl;
    }

    if (rclcpp::ok ()) {
        rclcpp::shutdown ();
    }

    return 0;
}
