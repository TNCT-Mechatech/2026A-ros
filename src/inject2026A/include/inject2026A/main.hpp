#pragma once

#include "rclcpp/rclcpp.hpp"

#include <termios.h>  // speed_t

#include <memory>
#include <string>

// ROS2 messages
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"  // Float32MultiArray 用
#include <sensor_msgs/msg/joint_state.hpp>

// Serial massages（メンバに持つので完全型が必要）
#include "Message.hpp"
#include "serial_packets.hpp"

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

   private:
    void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg);
    void serial_callback ();
    void gain_setup () const;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;  // Publisher の宣言
    rclcpp::TimerBase::SharedPtr                                   timer_;

    SerialDev    *serial_dev;
    SerialBridge *serial;
    Mechanism     mechanism_;

    sb::Message<MotorCommandPacket>  con;
    sb::Message<MotorFeedbackPacket> fb;

    bool s1, s2, s3, s4, s5, s6;

    float lx, ly, rx, ry, l2, r2, ex1, ex2;
    bool  cross, circle, triangle, square, l1, r1;
    bool  up, down, left, right;
};
