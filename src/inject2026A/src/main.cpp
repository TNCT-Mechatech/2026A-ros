#include "main.hpp"  // SerialNode

#include "rack_angle_profile.hpp"
#include "rack_gains.hpp"
#include "serial_packets.hpp"

#include <natto_robot_hardware/pid_gain_path.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <filesystem>
#include <thread>

// 実体ヘッダは .cpp 内で
#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <stdexcept>
using std::placeholders::_1;
using namespace natto_serial;

// 2台のマイコンは、それぞれ別のUSBシリアルポートを使用する。
// 注入用ST-Linkの個体番号で固定する。
constexpr const char *INJECT_SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF505071655067254019-if02";

// ロール用ST-Linkの個体番号で固定する。
constexpr const char *ROLL_SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF555185754867214745-if02";

SerialNode::SerialNode (const std::string &node_name, const char port[], Mechanism mechanism, const std::string &feedback_topic, speed_t baud_rate)
    : Node (node_name),
      serial_dev (nullptr),
      serial (nullptr),
      mechanism_ (mechanism),
      s1 (false),
      s2 (false),
      s3 (false),
      s4 (false),
      s5 (false),
      s6 (false),
      lx (0.f),
      ly (0.f),
      rx (0.f),
      ry (0.f),
      l2 (0.f),
      r2 (0.f),
      ex1 (0.f),
      ex2 (0.f),
      cross (false),
      circle (false),
      triangle (false),
      square (false),
      l1 (false),
      r1 (false),
      up (false),
      down (false),
      left (false),
      right (false) {
    // Existing mechanism commands had pid_state=false: their values are duty.
    declare_initialization (*this, frames_, 0, declare_parameter<bool> ("stop_mode", false), declare_parameter<double> ("max_rps", 4.0), {1, 1, 1, 1});

    if (mechanism_ == Mechanism::Inject) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only   = true;
        descriptor.description = "Startup mode: rack position PID or encoder-independent Duty control";
        const bool inject_pid  = declare_parameter<bool> ("inject_pid", true, descriptor);
        if (inject_pid) {
            setup_rack ();
            RCLCPP_INFO (get_logger (), "Inject PID ON: rack %s control", rack_count_ ? (rack_profile_counts_per_degree_ != 0 ? "angle profile" : "count profile") : "angle jog");
        } else {
            rack_open_loop_duty_ = declare_parameter<double> ("rack_open_loop_duty", 0.2);
            if (!std::isfinite (rack_open_loop_duty_) || rack_open_loop_duty_ <= 0 || rack_open_loop_duty_ > 1) throw std::invalid_argument ("rack_open_loop_duty must be in (0, 1]");
            RCLCPP_INFO (get_logger (), "Inject PID OFF: rack button-held Duty control, limit=%.3f", rack_open_loop_duty_);
        }
    }

    serial_path_           = declare_parameter<std::string> ("serial_path", port);
    baud_rate_             = baud_rate;
    reconnect_interval_ms_ = declare_parameter<int64_t> ("reconnect_interval_ms", 1000);
    if (reconnect_interval_ms_ <= 0) throw std::invalid_argument ("reconnect_interval_ms must be positive");
    const auto command_timeout = std::chrono::milliseconds (declare_parameter<int64_t> ("timeout_ms", 300));
    const bool monitor_updates = declare_parameter<bool> ("monitor_serial_updates", false);
    const int  failure_limit   = declare_parameter<int> ("serial_update_failure_limit", 5);
    if (failure_limit < 1) throw std::invalid_argument ("serial_update_failure_limit must be positive");
    watchdog_ = monitor_updates ? std::make_unique<Watchdog> (command_timeout, failure_limit) : std::make_unique<Watchdog> (command_timeout);
    RCLCPP_INFO (get_logger (), "Serial receive monitoring: %s", monitor_updates ? "enabled" : "disabled (send-only)");
    restart_policy_ = std::make_unique<natto_system_monitor::SerialRestartPolicy> (
        declare_parameter<bool> ("auto_restart", false), declare_parameter<int> ("restart_failure_count", 1), std::chrono::milliseconds (declare_parameter<int64_t> ("restart_timeout_ms", 1000)),
        std::chrono::milliseconds (declare_parameter<int64_t> ("restart_recovery_ms", 3000)));
    serial_diagnostics_ = std::make_unique<natto_system_monitor::SerialDiagnostics> (this, serial_path_);
    serial_diagnostics_->observe_watchdog (watchdog_.get ());
    serial_diagnostics_->configuration_status (false, false, frames_.configuration.state ());
    watchdog_->reset (Watchdog::Clock::now ());

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy> ("inject/joy", 10, std::bind (&SerialNode::joy_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray> (feedback_topic, 10);
    if (mechanism_ == Mechanism::Roll) command_duty_pub_ = create_publisher<std_msgs::msg::Float32MultiArray> ("roll/command_duty", 10);

    switch_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray> (mechanism_ == Mechanism::Inject ? "inject/switch_feedback" : "roll/switch_feedback", 10);

    try_open_serial ();

    // タイマーの設定（シリアル通信）
    timer_ = this->create_wall_timer (10ms, std::bind (&SerialNode::serial_callback, this));
}

SerialNode::~SerialNode () {
    // Repeat Duty 0 across several STM loops; mixed command types may be rejected once.
    for (int attempt = 0; serial && attempt < 5; ++attempt) {
        send_stop ();
        std::this_thread::sleep_for (10ms);
    }
    close_serial ();
}

void SerialNode::close_serial () {
    if (rack_) rack_->reset ();
    if (rack_count_) rack_count_->reset ();
    rack_start_held_ = rack_start_pending_ = false;
    rack_button4_held_ = rack_button5_held_ = false;
    rack_command_                           = {};
    square = triangle = false;
    // SerialBridge keeps a pointer to the port, so destroy it first.
    serial.reset ();
    serial_dev.reset ();
    serial_diagnostics_->opened (-1);
    serial_diagnostics_->configuration_status (false, frames_.configuration.failed (), frames_.configuration.failed () ? frames_.configuration.error () : "Serial disconnected; configuration unconfirmed");
    last_reconnect_attempt_ = Watchdog::Clock::now ();
    for (auto &motor : frames_.duty) motor.data.duty = 0.0f;
}

bool SerialNode::try_open_serial () {
    close_serial ();
    serial_diagnostics_->reconnect_attempt (++reconnect_attempts_);
    try {
        serial_dev = std::make_unique<SerialPort> (serial_path_.c_str (), baud_rate_, *serial_diagnostics_);
        serial     = std::make_unique<SerialBridge> (serial_dev.get (), 1024);
        frames_.reset_received ();
        frames_.register_all (*serial);
        frames_.initialize (*serial);
        serial_diagnostics_->configuration_status (false, false, frames_.configuration.state ());
        watchdog_->reset (Watchdog::Clock::now ());
        watchdog_state_ = watchdog_->check (Watchdog::Clock::now ());
        if (!write_command ()) {
            throw std::runtime_error ("failed to send the initial zero command");
        }
        RCLCPP_INFO (get_logger (), "Serial connection established: %s", serial_path_.c_str ());
        return true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR (get_logger (), "Serial connection failed: %s: %s", serial_path_.c_str (), e.what ());
    } catch (...) {
        RCLCPP_ERROR (get_logger (), "Serial connection failed: %s: unknown error", serial_path_.c_str ());
    }
    close_serial ();
    return false;
}

void SerialNode::send_stop () {
    if (rack_) rack_->stop ();
    if (rack_count_) rack_count_->stop ();
    rack_start_pending_ = false;
    rack_command_       = {};
    for (auto &motor : frames_.duty) motor.data.duty = 0.0f;
    write_command ();
}

void SerialNode::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (!serial || !frames_.configuration.ready ()) {
        // Commands received while disconnected must not survive reconnection.
        watchdog_->reset (Watchdog::Clock::now ());
        send_stop ();
        return;
    }
    if (mechanism_ == Mechanism::Inject) {
        if (msg->buttons.size () <= 12) {
            RCLCPP_WARN (this->get_logger (), "Joy message has too few buttons for the inject mechanism");
            watchdog_->reset (Watchdog::Clock::now ());
            send_stop ();
            return;
        }

        cross              = msg->buttons[0];  // A
        circle             = msg->buttons[1];  // B
        square             = msg->buttons[2];  // X
        triangle           = msg->buttons[3];  // Y
        const bool button4 = msg->buttons[4] != 0;
        const bool button5 = rack_profile_counts_per_degree_ != 0 && msg->buttons[5] != 0;
        if (rack_count_) {
            if (button4 && button5)
                rack_start_pending_ = false;
            else if ((button4 && !rack_button4_held_) || (button5 && !rack_button5_held_)) {
                rack_requested_target_ = button4 ? rack_primary_target_ : rack_secondary_target_;
                rack_start_pending_    = true;
            }
        }
        rack_button4_held_ = button4;
        rack_button5_held_ = button5;
        rack_start_held_   = button4 || button5;
        down               = msg->buttons[11];
        up                 = msg->buttons[12];

        // 雑巾装填
        frames_.duty[0].data.duty = up * 0.4f - down * 0.2f;

        // 雑巾送り出し
        frames_.duty[1].data.duty = (cross - circle) * 0.4f;

        // PID mode computes rack commands in the timer; Duty mode needs no encoder.
        frames_.duty[2].data.duty = (rack_ || rack_count_) ? 0.0f : static_cast<float> ((square - triangle) * rack_open_loop_duty_);

        // 未使用
        frames_.duty[3].data.duty = 0;
    } else {
        if (msg->axes.size () <= 7) {
            RCLCPP_WARN (this->get_logger (), "Joy message has too few axes for the roll mechanism");
            watchdog_->reset (Watchdog::Clock::now ());
            send_stop ();
            return;
        }
        ly = msg->axes[1];
        rx = -msg->axes[2];

        ex1 = msg->axes[6];
        ex2 = msg->axes[7];

        // 発射装置1、2
        frames_.duty[0].data.duty = -ex1;
        frames_.duty[1].data.duty = ex2;

        // モーター4: 右スティック左右
        frames_.duty[2].data.duty = -rx * 0.3f;

        // モーター3: 左スティック上下
        frames_.duty[3].data.duty = ly * 0.3f;
    }

    bool neutral = mechanism_ != Mechanism::Inject || (!square && !triangle && (!rack_count_ || !rack_start_held_));
    for (const auto &motor : frames_.duty) {
        if (!std::isfinite (motor.data.duty)) {
            RCLCPP_WARN (get_logger (), "Joy command contains a non-finite motor target");
            watchdog_->reset (Watchdog::Clock::now ());
            send_stop ();
            return;
        }
        neutral = neutral && motor.data.duty == 0.0f;
    }
    if (!watchdog_->command (Watchdog::Clock::now (), neutral)) {
        send_stop ();
        return;
    }
    for (auto &motor : frames_.duty) motor.data.duty = std::clamp (motor.data.duty, -1.0f, 1.0f);
    // One normal command sender avoids interleaving position and Duty packets.
}

bool SerialNode::write_command () {
    if (!serial) return false;
    if (!frames_.configuration.ready () || watchdog_->check (Watchdog::Clock::now ()) != Watchdog::State::Ready) {
        for (auto &motor : frames_.duty) motor.data.duty = 0.0f;
        rack_command_ = {};
    }
    try {
        for (size_t i = 0; i < MOTORS; ++i) {
            if ((rack_ || rack_count_) && i == 2) {
                if (rack_command_.position) {
                    frames_.position[i].data.target_pos = rack_command_.target_count;
                    SerialFrames::write (*serial, static_cast<uint8_t> (CMD_POS_FRAME_ID_BASE + i), frames_.position[i]);
                } else if (rack_command_.velocity) {
                    frames_.velocity[i].data.target_motor_rps = rack_command_.target_rps;
                    SerialFrames::write (*serial, static_cast<uint8_t> (CMD_VEL_FRAME_ID_BASE + i), frames_.velocity[i]);
                } else {
                    frames_.duty[i].data.duty = rack_command_.duty;
                    SerialFrames::write (*serial, static_cast<uint8_t> (CMD_DUTY_FRAME_ID_BASE + i), frames_.duty[i]);
                }
            } else {
                SerialFrames::write (*serial, static_cast<uint8_t> (CMD_DUTY_FRAME_ID_BASE + i), frames_.duty[i]);
            }
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR (get_logger (), "Serial write failed: %s", e.what ());
        close_serial ();
        return false;
    } catch (...) {
        RCLCPP_ERROR (get_logger (), "Serial write failed with an unknown error");
        close_serial ();
        return false;
    }
    // Publish only complete, successful serial writes, including watchdog stops.
    // This reports the PC's sent command, not an acknowledgement from the STM.
    if (command_duty_pub_) {
        std_msgs::msg::Float32MultiArray msg;
        for (const auto &motor : frames_.duty) msg.data.push_back (motor.data.duty);
        command_duty_pub_->publish (msg);
    }
    return true;
}

void SerialNode::serial_callback () {
    // Observe the result of this cycle before deciding on a process restart.
    const auto observe_restart = [this] {
        const auto time    = Watchdog::Clock::now ();
        const auto state   = watchdog_->check (time);
        const bool healthy = serial != nullptr && state != Watchdog::State::WaitingUpdate && state != Watchdog::State::UpdateError;
        if (restart_policy_->observe (time, healthy)) {
            RCLCPP_ERROR (get_logger (), "Persistent serial communication failure: requesting process restart");
            send_stop ();
            close_serial ();
            rclcpp::shutdown ();
        }
    };
    if (!serial) {
        if (rack_ || rack_count_) publish_rack ();
        if (Watchdog::Clock::now () - last_reconnect_attempt_ >= std::chrono::milliseconds (reconnect_interval_ms_)) {
            try_open_serial ();
        }
        observe_restart ();
        return;
    }
    DrainResult received;
    try {
        received = frames_.drain (*serial);
        frames_.service_initialization (*serial);
    } catch (const std::exception &e) {
        serial_diagnostics_->configuration_status (false, frames_.configuration.failed (), frames_.configuration.failed () ? frames_.configuration.error () : frames_.configuration.state ());
        RCLCPP_ERROR (this->get_logger (), "Serial update failed: %s", e.what ());
        send_stop ();
        close_serial ();
        observe_restart ();
        return;
    } catch (...) {
        RCLCPP_ERROR (this->get_logger (), "Serial update failed with an unknown error");
        send_stop ();
        close_serial ();
        observe_restart ();
        return;
    }

    serial_diagnostics_->configuration_status (frames_.configuration.ready (), false, frames_.configuration.state ());
    if (received.configuration_lost) {
        watchdog_->reset (Watchdog::Clock::now ());
        if (rack_) rack_->reset ();
        if (rack_count_) rack_count_->reset ();
        square = triangle = false;
        send_stop ();
        RCLCPP_WARN (get_logger (), "STM configuration changed/reset; mechanism stopped");
    }
    // Periodic feedback and initialization IDs are mandatory in protocol v2.
    const bool feedback_updated = std::any_of (received.motor_updated.begin (), received.motor_updated.end (), [] (bool v) { return v; });
    const int  update_result    = received.status;
    const auto time             = Watchdog::Clock::now ();
    if (watchdog_->uses_update_results ())
        watchdog_->update_result (time, update_result);
    else if (feedback_updated)
        watchdog_->feedback (time);
    if (rack_) {
        if (received.motor_updated[2]) rack_->motor_feedback (time, frames_.feedback[2].data.encoder_raw_count);
        if (received.switches_updated) rack_->switch_feedback (time, frames_.switches.data.state[rack_switch_index_] == rack_switch_active_state_);
        rack_command_ = rack_->step (time, square, triangle, frames_.configuration.ready () && watchdog_->check (time) == Watchdog::State::Ready);
        publish_rack ();
    }
    if (rack_count_) {
        if (received.motor_updated[2]) rack_count_->motor_feedback (time, frames_.feedback[2].data.encoder_raw_count);
        if (received.switches_updated) rack_count_->switch_feedback (time, frames_.switches.data.state[rack_switch_index_] == rack_switch_active_state_);
        rack_command_ = rack_count_->step (time, rack_start_pending_, rack_start_held_, triangle, square, frames_.configuration.ready () && watchdog_->check (time) == Watchdog::State::Ready, rack_requested_target_);
        if (rack_velocity_) rack_command_ = rack_velocity_->command (*rack_count_, rack_command_);
        rack_start_pending_ = false;
        publish_rack ();
    }
    const auto state = watchdog_->check (time);
    if (state != watchdog_state_) {
        RCLCPP_WARN (get_logger (), "Serial watchdog: %s", Watchdog::name (state));
        watchdog_state_ = state;
    }
    if (state != Watchdog::State::Ready) {
        send_stop ();
    } else {
        write_command ();
    }
    if (state == Watchdog::State::UpdateError) {
        RCLCPP_WARN (get_logger (), "serial.update()=%d for %d consecutive failures; reconnecting", update_result, watchdog_->update_failures ());
        close_serial ();
    }

    if (!serial) {
        observe_restart ();
        return;
    }
    if (feedback_updated) {
        std_msgs::msg::Float32MultiArray msg;
        for (const auto &motor : frames_.feedback) msg.data.push_back (motor.data.measured_wheel_rps);
        publisher_->publish (msg);
    }
    if (received.switches_updated) {
        std_msgs::msg::UInt8MultiArray msg;
        for (bool state : frames_.switches.data.state) msg.data.push_back (state);
        switch_pub_->publish (msg);
    }
    observe_restart ();
}

void SerialNode::setup_rack () {
    auto &init = frames_.init[2].data;
    // ROS stores an offset at the switch. STM origin resets would invalidate live targets.
    init.reset_encoder_origin             = false;
    init.reset_encoder_origin_with_switch = false;
    init.pid_state                        = 0;
    init.stop_mode                        = false;  // Zero position error must still permit velocity-loop braking.
    rack_switch_active_state_             = declare_parameter<bool> ("rack_switch_active_state", true);
    const auto switch_index               = declare_parameter<int64_t> ("rack_switch_index", 2);
    if (switch_index < 0 || switch_index >= static_cast<int64_t> (MOTORS)) throw std::invalid_argument ("rack_switch_index must be 0..3");
    rack_switch_index_             = static_cast<size_t> (switch_index);
    const auto   default_gain_path = std::filesystem::path (natto_robot_hardware::resolve_pid_gain_path ("")).parent_path () / "inject_pid_gain.yaml";
    const auto   gain_path         = declare_parameter<std::string> ("rack_pid_gain_path", default_gain_path.string ());
    const double maximum_rps       = declare_parameter<double> ("rack_max_rps", 0.03);
    if (!std::isfinite (maximum_rps) || maximum_rps <= 0 || maximum_rps > std::numeric_limits<float>::max ()) throw std::invalid_argument ("rack_max_rps must be a positive finite float");
    bool                   gains_ready = false;
    inject2026a::RackGains gains;
    try {
        gains       = inject2026a::load_rack_gains (gain_path, static_cast<float> (maximum_rps));
        init.vel_kp = gains.vel_kp;
        init.vel_ki = gains.vel_ki;
        init.vel_kd = 0;
        init.pos_kp = gains.pos_kp;
        init.pos_ki = init.pos_kd = 0;
        init.max_motor_rps        = std::min (static_cast<float> (maximum_rps), gains.max_rps);
        if (gains.ppr) {
            init.encoder_ppr = gains.ppr;
            init.gear_rate   = gains.gear;
        }
        if (gains.polarity) init.set_direction = gains.polarity < 0;
        gains_ready = true;
        RCLCPP_INFO (get_logger (), "Rack motor 2 PID loaded: %s, max=%.4f rps", gain_path.c_str (), init.max_motor_rps);
    } catch (const std::exception &e) {
        RCLCPP_ERROR (get_logger (), "Rack motion disabled; tune motor 2 and restart: %s", e.what ());
    }
    inject2026a::RackController::Config config;
    const double                        transmission = declare_parameter<double> ("rack_transmission_ratio", 0.0);
    const auto                          sign         = declare_parameter<int64_t> ("rack_angle_count_sign", 1);
    if (!std::isfinite (transmission) || transmission < 0 || (sign != 1 && sign != -1)) throw std::invalid_argument ("rack_transmission_ratio must be non-negative; rack_angle_count_sign must be +/-1");
    // Extra reduction from the encoder-configured output shaft to the rack.
    config.counts_per_degree = static_cast<double> (sign) * 4.0 * init.encoder_ppr * init.gear_rate * transmission / 360.0;
    config.max_angle_deg     = declare_parameter<double> ("rack_max_angle_deg", 0.0);
    config.speed_deg_s       = declare_parameter<double> ("rack_speed_deg_s", 15.0);
    config.homing_duty       = declare_parameter<double> ("rack_homing_duty", -0.1);
    config.homing_timeout_s  = declare_parameter<double> ("rack_homing_timeout_s", 10.0);
    config.feedback_timeout  = std::chrono::milliseconds (declare_parameter<int64_t> ("rack_feedback_timeout_ms", 200));
    config.switch_debounce   = std::chrono::milliseconds (declare_parameter<int64_t> ("rack_switch_debounce_ms", 50));
    config.gains_ready       = gains_ready;
    const bool count_profile = declare_parameter<bool> ("rack_count_profile", true);
    const bool angle_profile = declare_parameter<bool> ("rack_angle_profile", false);
    if (count_profile || angle_profile) {
        inject2026a::RackCountController::Config counts;
        counts.target           = declare_parameter<int64_t> ("rack_target_relative_count", -30);
        counts.speed            = declare_parameter<double> ("rack_max_speed_counts_s", 10.0);
        counts.acceleration     = declare_parameter<double> ("rack_acceleration_counts_s2", 20.0);
        counts.tolerance        = declare_parameter<int64_t> ("rack_position_tolerance_counts", 1);
        counts.settle           = std::chrono::milliseconds (declare_parameter<int64_t> ("rack_settle_ms", 100));
        counts.motion_timeout_s = declare_parameter<double> ("rack_motion_timeout_s", 10.0);
        counts.homing_duty      = config.homing_duty;
        counts.manual_duty      = declare_parameter<double> ("rack_manual_duty", 0.2);
        counts.homing_timeout_s = config.homing_timeout_s;
        counts.feedback_timeout = config.feedback_timeout;
        counts.switch_debounce  = config.switch_debounce;
        counts.gains_ready      = gains_ready;
        if (angle_profile) {
            const inject2026a::RackAngleProfile angles{
                config.counts_per_degree,
                config.max_angle_deg,
                config.speed_deg_s,
                declare_parameter<double> ("rack_acceleration_deg_s2", 10.0),
                declare_parameter<double> ("rack_position_tolerance_deg", 0.2),
                declare_parameter<double> ("rack_range_tolerance_deg", 1.0)};
            const double primary            = declare_parameter<double> ("rack_target_angle_4_deg", 5.0);
            const double secondary          = declare_parameter<double> ("rack_target_angle_5_deg", 10.0);
            counts                          = angles.convert (counts, primary);
            rack_secondary_target_          = angles.target_count (secondary);
            rack_profile_counts_per_degree_ = angles.counts_per_degree;
            RCLCPP_INFO (
                get_logger (),
                "Rack angle buttons[4]=%.3f deg / buttons[5]=%.3f deg, "
                "range=0..%.3f deg, speed=%.3f deg/s, acceleration=%.3f deg/s^2, signed scale=%.6f counts/deg",
                primary, secondary, angles.max_angle_deg, angles.speed_deg_s, angles.acceleration_deg_s2, angles.counts_per_degree);
            RCLCPP_INFO (
                get_logger (), "Rack tolerance: arrival=%.3f deg (%lld counts), range=%.3f deg (%lld counts)", angles.tolerance_deg, static_cast<long long> (counts.tolerance), angles.range_tolerance_deg,
                static_cast<long long> (counts.range_tolerance));
        }
        rack_primary_target_ = rack_requested_target_ = counts.target;
        // The STM limits velocity in configured output-shaft rps.
        if (gains_ready && counts.speed > 4.0 * init.encoder_ppr * init.gear_rate * init.max_motor_rps) throw std::invalid_argument ("rack_max_speed_counts_s exceeds STM rack_max_rps/gain speed limit");
        rack_count_ = std::make_unique<inject2026a::RackCountController> (counts);
        RCLCPP_INFO (get_logger (), "Rack manual buttons[2]/[3]: Duty +/- %.3f; overrides automatic motion", counts.manual_duty);
        const bool velocity_feedforward = declare_parameter<bool> ("rack_velocity_feedforward", false);
        if (angle_profile && velocity_feedforward) {
            if (gains_ready && !gains.model_ready) throw std::invalid_argument ("rack velocity feedforward requires response-model position tuning");
            if (gains_ready) {
                rack_velocity_ =
                    std::make_unique<inject2026a::RackVelocityProfile> (inject2026a::RackVelocityProfile::Config{4.0 * init.encoder_ppr * init.gear_rate, init.pos_kp, init.max_motor_rps, gains.response_s, gains.delay_s, gains.steady_gain});
                RCLCPP_INFO (
                    get_logger (),
                    "Rack velocity feedforward: response=%.4fs, delay=%.4fs, gain=%.4f; "
                    "STM velocity PI, ROS position correction, max=%.4f rps",
                    gains.response_s, gains.delay_s, gains.steady_gain, init.max_motor_rps);
            }
        }
        rack_relative_pub_        = create_publisher<std_msgs::msg::Int64> ("inject/rack/relative_count", 10);
        rack_relative_target_pub_ = create_publisher<std_msgs::msg::Int64> ("inject/rack/target_relative_count", 10);
        RCLCPP_INFO (get_logger (), "Rack buttons[4]: target=%lld counts, speed=%.2f counts/s, acceleration=%.2f counts/s^2", static_cast<long long> (counts.target), counts.speed, counts.acceleration);
    } else {
        rack_ = std::make_unique<inject2026a::RackController> (config);
        if (!rack_->configured ()) RCLCPP_WARN (get_logger (), "Rack requires PID gains, rack_transmission_ratio and rack_max_angle_deg before motion");
    }
    rack_angle_pub_  = create_publisher<std_msgs::msg::Float64> ("inject/rack/angle_deg", 10);
    rack_target_pub_ = create_publisher<std_msgs::msg::Float64> ("inject/rack/target_angle_deg", 10);
    rack_goal_pub_   = create_publisher<std_msgs::msg::Float64> ("inject/rack/goal_angle_deg", 10);
    rack_raw_pub_    = create_publisher<std_msgs::msg::Int64> ("inject/rack/encoder_raw_count", 10);
    rack_state_pub_  = create_publisher<std_msgs::msg::String> ("inject/rack/state", rclcpp::QoS (1).transient_local ());
}

void SerialNode::publish_rack () {
    const char *current_state = rack_count_ ? rack_count_->state () : rack_->state ();
    if (rack_last_state_ != current_state) {
        rack_last_state_ = current_state;
        RCLCPP_INFO (get_logger (), "Rack: %s", rack_last_state_.c_str ());
    }
    std_msgs::msg::String state;
    state.data = current_state;
    rack_state_pub_->publish (state);
    const auto now = Watchdog::Clock::now ();
    if (rack_count_ ? rack_count_->fresh (now) : rack_->fresh (now)) {
        std_msgs::msg::Int64 raw;
        raw.data = frames_.feedback[2].data.encoder_raw_count;
        rack_raw_pub_->publish (raw);
        if (rack_count_ && rack_count_->homed ()) {
            std_msgs::msg::Int64 relative, target;
            relative.data = rack_count_->relative_count ();
            target.data   = rack_count_->target_relative_count ();
            rack_relative_pub_->publish (relative);
            rack_relative_target_pub_->publish (target);
            if (rack_profile_counts_per_degree_ != 0) {
                std_msgs::msg::Float64 angle, angle_target;
                angle.data        = static_cast<double> (relative.data) / rack_profile_counts_per_degree_;
                angle_target.data = static_cast<double> (target.data) / rack_profile_counts_per_degree_;
                rack_angle_pub_->publish (angle);
                rack_target_pub_->publish (angle_target);
                std_msgs::msg::Float64 goal;
                goal.data = static_cast<double> (rack_count_->goal_relative_count ()) / rack_profile_counts_per_degree_;
                rack_goal_pub_->publish (goal);
                const bool moving = std::strcmp (current_state, "moving") == 0 || std::strcmp (current_state, "settling") == 0;
                if (moving) {
                    RCLCPP_INFO_THROTTLE (
                        get_logger (), *get_clock (), 1000,
                        "Rack tracking: state=%s, measured=%.3f deg, reference=%.3f deg, goal=%.3f deg, "
                        "error=%.3f deg, mode=%s, command=%.5f rps, feedback=%.5f rps, duty=%.3f",
                        current_state, angle.data, angle_target.data, goal.data, goal.data - angle.data, rack_command_.velocity ? "velocity" : "position", rack_command_.target_rps, frames_.feedback[2].data.measured_wheel_rps,
                        frames_.feedback[2].data.duty);
                }
            }
        }
        if (rack_ && rack_->homed ()) {
            std_msgs::msg::Float64 measured, target;
            measured.data = rack_->measured_angle ();
            target.data   = rack_->target_angle ();
            rack_angle_pub_->publish (measured);
            rack_target_pub_->publish (target);
        }
    }
}

int main (int argc, char *argv[]) {
    rclcpp::init (argc, argv);

    // 1プロセス内で2ノードを動かし、2台のマイコンへ個別に接続する。
    auto inject_node = std::make_shared<SerialNode> ("inject_serial_node", INJECT_SERIAL_PORT, Mechanism::Inject, "inject/fb_speed_topic");
    auto roll_node   = std::make_shared<SerialNode> ("roll_serial_node", ROLL_SERIAL_PORT, Mechanism::Roll, "roll/fb_speed_topic");

    rclcpp::executors::MultiThreadedExecutor exec;  // 並列コールバックOK
    exec.add_node (inject_node);
    exec.add_node (roll_node);
    exec.spin ();

    rclcpp::shutdown ();
    return 0;
}
