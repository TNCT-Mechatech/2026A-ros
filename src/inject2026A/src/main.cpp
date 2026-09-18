#include "main.hpp"  // SerialNode

#include "serial_packets.hpp"  // MotorCommand, MotorFeedback

// 実体ヘッダは .cpp 内で
#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"

#include <cstdio>
#include <exception>
using std::placeholders::_1;

// 2台のマイコンは、それぞれ別のUSBシリアルポートを使用する。
constexpr const char *INJECT_SERIAL_PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.2";

constexpr const char *ROLL_SERIAL_PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.2";
// ros2 runでエラー出たらとりあえずこれ
// sudo chmod 666 /dev/serial/by-path/platform-fd500000.pcie-pci-0000\:01\:00.0-usb-0\:1.4\:1.2

// SerialPort for ThinkPad
// #define SERIAL_PORT "/dev/serial/by-path/pci-0000:00:14.0-usb-0:5:1.2"

SerialNode::SerialNode (const std::string &node_name, const char port[], Mechanism mechanism, const std::string &feedback_topic, speed_t baud_rate)
    : Node (node_name),
      serial_dev (new LinuxHardwareSerial (port, baud_rate)),
      serial (new SerialBridge (serial_dev, 1024)),
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
    con.data = MotorCommandPacket{};  // 初期化
    fb.data  = MotorFeedbackPacket{};

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy> ("inject/joy", 10, std::bind (&SerialNode::joy_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray> (feedback_topic, 10);

    serial->add_frame (0, &con);
    serial->add_frame (1, &fb);
    // serial->add_frame(2, &gain);

    // タイマーの設定（シリアル通信）
    timer_ = this->create_wall_timer (10ms, std::bind (&SerialNode::serial_callback, this));
}

void SerialNode::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (mechanism_ == Mechanism::Inject) {
        if (msg->buttons.size () <= 12) {
            RCLCPP_WARN (this->get_logger (), "Joy message has too few buttons for the inject mechanism");
            return;
        }

        cross    = msg->buttons[0];  // A
        circle   = msg->buttons[1];  // B
        square   = msg->buttons[2];  // X
        triangle = msg->buttons[3];  // Y
        down     = msg->buttons[11];
        up       = msg->buttons[12];

        // 雑巾装填
        con.data.mc[0].target_wheel_rps = up * 0.4f - down * 0.2f;

        // 雑巾送り出し
        con.data.mc[1].target_wheel_rps = (cross - circle) * 0.4f;

        // 雑巾ラック
        con.data.mc[2].target_wheel_rps = (square - triangle) * 0.2f;

        // 未使用
        con.data.mc[3].target_wheel_rps = 0;
    } else {
        if (msg->axes.size () <= 7) {
            RCLCPP_WARN (this->get_logger (), "Joy message has too few axes for the roll mechanism");
            return;
        }
        ly = msg->axes[1];
        rx = -msg->axes[2];

        ex1 = msg->axes[6];
        ex2 = msg->axes[7];

        // 発射装置1、2
        con.data.mc[0].target_wheel_rps = -ex1;
        con.data.mc[1].target_wheel_rps = ex2;

        // モーター3: 左スティック上下
        con.data.mc[2].target_wheel_rps = ly * 0.3f;

        // モーター4: 右スティック左右
        con.data.mc[3].target_wheel_rps = rx * 0.3f;
    }

    const int write_result = serial->write (0);

    RCLCPP_INFO (this->get_logger (), "write=%d command=[%.3f %.3f %.3f %.3f]", write_result, con.data.mc[0].target_wheel_rps, con.data.mc[1].target_wheel_rps, con.data.mc[2].target_wheel_rps, con.data.mc[3].target_wheel_rps);
}

void SerialNode::serial_callback () {
    try {
        serial->update ();
    } catch (const std::exception &e) {
        RCLCPP_ERROR (this->get_logger (), "Serial update failed: %s", e.what ());
        return;
    } catch (...) {
        RCLCPP_ERROR (this->get_logger (), "Serial update failed with an unknown error");
        return;
    }

    // was_updated() clears the update flag, so call it exactly once per cycle.
    const bool feedback_updated = fb.was_updated ();

    // RCLCPP_INFO(
    //   this->get_logger(),
    //   "update=%d updated=%d ",
    //   result,
    //   static_cast<int>(feedback_updated),
    //   // fb.data.mfd[0].measured_wheel_rps,
    //   // fb.data.mfd[1].measured_wheel_rps,
    //   // fb.data.mfd[2].measured_wheel_rps,
    //   // fb.data.mfd[3].measured_wheel_rps
    // );

    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize (4);

    for (std::size_t i = 0; i < 4; ++i) {
        msg.data[i] = fb.data.mfd[i].measured_wheel_rps;
    }

    if (feedback_updated) {
        // std_msgs::msg::Float32MultiArray msg;
        // msg.data.resize(4);

        // for (std::size_t i = 0; i < 4; ++i) {
        //   msg.data[i] = fb.data.mfd[i].measured_wheel_rps;
        // }

        publisher_->publish (msg);
    }
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
