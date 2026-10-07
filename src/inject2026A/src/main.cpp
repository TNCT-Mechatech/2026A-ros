#include "main.hpp"  // SerialNode

#include "serial_packets.hpp"

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
    watchdog_->reset (Watchdog::Clock::now ());

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy> ("inject/joy", 10, std::bind (&SerialNode::joy_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray> (feedback_topic, 10);

    switch_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray> (mechanism_ == Mechanism::Inject ? "inject/switch_feedback" : "roll/switch_feedback", 10);

    try_open_serial ();

    // タイマーの設定（シリアル通信）
    timer_ = this->create_wall_timer (10ms, std::bind (&SerialNode::serial_callback, this));
}

SerialNode::~SerialNode () {
    send_stop ();
    close_serial ();
}

void SerialNode::close_serial () {
    // SerialBridge keeps a pointer to the port, so destroy it first.
    serial.reset ();
    serial_dev.reset ();
    serial_diagnostics_->opened (-1);
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
    for (auto &motor : frames_.duty) motor.data.duty = 0.0f;
    write_command ();
}

void SerialNode::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (!serial) {
        // Commands received while disconnected must not survive reconnection.
        send_stop ();
        return;
    }
    if (mechanism_ == Mechanism::Inject) {
        if (msg->buttons.size () <= 12) {
            RCLCPP_WARN (this->get_logger (), "Joy message has too few buttons for the inject mechanism");
            send_stop ();
            return;
        }

        cross    = msg->buttons[0];  // A
        circle   = msg->buttons[1];  // B
        square   = msg->buttons[2];  // X
        triangle = msg->buttons[3];  // Y
        down     = msg->buttons[11];
        up       = msg->buttons[12];

        // 雑巾装填
        frames_.duty[0].data.duty = up * 0.4f - down * 0.2f;

        // 雑巾送り出し
        frames_.duty[1].data.duty = (cross - circle) * 0.4f;

        // 雑巾ラック
        frames_.duty[2].data.duty = (square - triangle) * 0.2f;

        // 未使用
        frames_.duty[3].data.duty = 0;
    } else {
        if (msg->axes.size () <= 7) {
            RCLCPP_WARN (this->get_logger (), "Joy message has too few axes for the roll mechanism");
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

        // モーター3: 左スティック上下
        frames_.duty[2].data.duty = ly * 0.3f;

        // モーター4: 右スティック左右
        frames_.duty[3].data.duty = rx * 0.3f;
    }

    bool neutral = true;
    for (const auto &motor : frames_.duty) {
        if (!std::isfinite (motor.data.duty)) {
            RCLCPP_WARN (get_logger (), "Joy command contains a non-finite motor target");
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
    write_command ();
}

bool SerialNode::write_command () {
    if (!serial) return false;
    if (watchdog_->check (Watchdog::Clock::now ()) != Watchdog::State::Ready) {
        for (auto &motor : frames_.duty) motor.data.duty = 0.0f;
    }
    try {
        for (size_t i = 0; i < MOTORS; ++i) SerialFrames::write (*serial, CMD_DUTY_FRAME_ID_BASE + i, frames_.duty[i]);
        return true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR (get_logger (), "Serial write failed: %s", e.what ());
    } catch (...) {
        RCLCPP_ERROR (get_logger (), "Serial write failed with an unknown error");
    }
    close_serial ();
    return false;
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
        if (Watchdog::Clock::now () - last_reconnect_attempt_ >= std::chrono::milliseconds (reconnect_interval_ms_)) {
            try_open_serial ();
        }
        observe_restart ();
        return;
    }
    DrainResult received;
    try {
        received = frames_.drain (*serial);
    } catch (const std::exception &e) {
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

    // Optional receive monitoring; send-only devices may never return a frame.
    const bool feedback_updated = std::any_of (received.motor_updated.begin (), received.motor_updated.end (), [] (bool v) { return v; });
    const int  update_result    = received.status;
    const auto time             = Watchdog::Clock::now ();
    if (watchdog_->uses_update_results ())
        watchdog_->update_result (time, update_result);
    else if (feedback_updated)
        watchdog_->feedback (time);
    const auto state = watchdog_->check (time);
    if (state != watchdog_state_) {
        RCLCPP_WARN (get_logger (), "Serial watchdog: %s", Watchdog::name (state));
        watchdog_state_ = state;
    }
    if (state != Watchdog::State::Ready) send_stop ();
    if (state == Watchdog::State::UpdateError) {
        RCLCPP_WARN (get_logger (), "serial.update()=%d for %d consecutive failures; reconnecting", update_result, watchdog_->update_failures ());
        close_serial ();
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

void SerialNode::gain_setup () const {}

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
