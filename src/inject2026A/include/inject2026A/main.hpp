#pragma once

#include "rack_controller.hpp"
#include "rack_count_controller.hpp"
#include "rack_velocity_profile.hpp"
#include "rclcpp/rclcpp.hpp"

#include <natto_system_monitor/serial_diagnostics.hpp>
#include <natto_system_monitor/serial_restart.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>

#include <termios.h>  // speed_t

#include <memory>
#include <string>

// ROS2 messages
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"  // Float32MultiArray 用
#include <sensor_msgs/msg/joint_state.hpp>

// Serial massages（メンバに持つので完全型が必要）
#include "LinuxHardwareSerial.hpp"
#include "Message.hpp"

#include <natto_robot_hardware/serial_initialization.hpp>

#include <std_msgs/msg/u_int8_multi_array.hpp>

// 前方宣言（ポインタで保持するので前方宣言でOK）
class SerialDev;
class SerialBridge;

using namespace std::chrono_literals;

enum class Mechanism {
    Inject,
    Roll,
};

class SerialNode : public rclcpp::Node {
   public:
    SerialNode (const std::string &node_name, const char port[], Mechanism mechanism, const std::string &feedback_topic, speed_t baud_rate = B230400);
    ~SerialNode () override;

   private:
    void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg);
    void serial_callback ();
    bool write_command ();
    bool try_open_serial ();
    void close_serial ();
    void send_stop ();
    void setup_rack ();
    void publish_rack ();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;  // Publisher の宣言
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_duty_pub_;
    rclcpp::TimerBase::SharedPtr                                   timer_;

    using SerialPort = natto_system_monitor::ObservedSerial<LinuxHardwareSerial>;
    std::unique_ptr<SerialPort>                              serial_dev;
    std::unique_ptr<SerialBridge>                            serial;
    std::string                                              serial_path_;
    speed_t                                                  baud_rate_;
    int64_t                                                  reconnect_interval_ms_;
    std::chrono::steady_clock::time_point                    last_reconnect_attempt_;
    uint64_t                                                 reconnect_attempts_{0};
    std::unique_ptr<natto_system_monitor::SerialDiagnostics> serial_diagnostics_;
    using Watchdog = natto_system_monitor::SerialWatchdog;
    std::unique_ptr<Watchdog>                                  watchdog_;
    std::unique_ptr<natto_system_monitor::SerialRestartPolicy> restart_policy_;
    Watchdog::State                                            watchdog_state_{Watchdog::State::WaitingUpdate};
    Mechanism                                                  mechanism_;

    natto_serial::SerialFrames                                   frames_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr switch_pub_;

    std::unique_ptr<inject2026a::RackController>         rack_;
    std::unique_ptr<inject2026a::RackCountController>    rack_count_;
    std::unique_ptr<inject2026a::RackVelocityProfile>    rack_velocity_;
    bool                                                 rack_start_held_{false}, rack_start_pending_{false};
    bool                                                 rack_button4_held_{false}, rack_button5_held_{false};
    int64_t                                              rack_primary_target_{-30}, rack_secondary_target_{0}, rack_requested_target_{-30};
    double                                               rack_profile_counts_per_degree_{0};
    double                                               rack_open_loop_duty_{0.2};
    inject2026a::RackController::Command                 rack_command_;
    bool                                                 rack_switch_active_state_{true};
    size_t                                               rack_switch_index_{2};
    std::string                                          rack_last_state_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rack_angle_pub_, rack_target_pub_, rack_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr   rack_raw_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr   rack_relative_pub_, rack_relative_target_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  rack_state_pub_;

    bool s1, s2, s3, s4, s5, s6;

    float lx, ly, rx, ry, l2, r2, ex1, ex2;
    bool  cross, circle, triangle, square, l1, r1;
    bool  up, down, left, right;
};
