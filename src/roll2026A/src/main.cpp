#include "main.hpp"  // SerialNode

#include "serial_packets.hpp"  // MotorCommand, MotorFeedback

// 実体ヘッダは .cpp 内で
#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"

#include <cstdio>
#include <exception>
using std::placeholders::_1;

// ロール用ST-Linkの個体番号で固定する。
constexpr const char *SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF555185754867214745-if02";

SerialNode::SerialNode (const char port[], speed_t baud_rate)
    : Node ("serial_node"),
      serial_dev (nullptr),
      serial (nullptr),
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

    const auto serial_path = declare_parameter<std::string> ("serial_path", port);
    RCLCPP_INFO (get_logger (), "Opening serial device: %s", serial_path.c_str ());
    serial_dev = new LinuxHardwareSerial (serial_path.c_str (), baud_rate);
    serial = new SerialBridge (serial_dev, 1024);

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy> ("inject/joy", 10, std::bind (&SerialNode::joy_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray> ("fb_speed_topic", 10);  // Publisher を作成

    serial->add_frame (0, &con);
    serial->add_frame (1, &fb);
    // serial->add_frame(2, &gain);

    // タイマーの設定（シリアル通信）
    timer_ = this->create_wall_timer (10ms, std::bind (&SerialNode::serial_callback, this));
}

void SerialNode::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    // lx = msg->axes[0] * -1;
    // ly = msg->axes[1];
    // rx = msg->axes[2] * -1;
    // ry = msg->axes[3];
    // l2 = msg->axes[4];
    // r2 = msg->axes[5];
    ex1 = msg->axes[6];
    ex2 = msg->axes[7];

    // cross    = msg->buttons[0];  // A
    // circle   = msg->buttons[1];  // B
    // square   = msg->buttons[2];  // X
    // triangle = msg->buttons[3];  // Y

    // l1 = msg->buttons[9];
    // r1 = msg->buttons[10];

    // down = msg->buttons[11];
    // up   = msg->buttons[12];
    // left = msg->buttons[13];
    // right = msg->buttons[14];

    // 発射装置1
    con.data.mc[0].target_wheel_rps = -ex1;

    // 発射装置2
    con.data.mc[1].target_wheel_rps = ex2;

    // 未使用
    con.data.mc[2].target_wheel_rps = 0;
    con.data.mc[3].target_wheel_rps = 0;

    const int write_result = serial->write (0);

    RCLCPP_INFO (this->get_logger (), "write=%d command=[%.3f %.3f %.3f %.3f]", write_result, con.data.mc[0].target_wheel_rps, con.data.mc[1].target_wheel_rps, con.data.mc[2].target_wheel_rps, con.data.mc[3].target_wheel_rps);
}

void SerialNode::serial_callback () {
    int result;

    try {
        result = serial->update ();
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

    // ★ 同一プロセスで2ノード同時運転
    auto serial_node = std::make_shared<SerialNode> (SERIAL_PORT);

    rclcpp::executors::MultiThreadedExecutor exec;  // 並列コールバックOK
    exec.add_node (serial_node);
    exec.spin ();

    rclcpp::shutdown ();
    return 0;
}
