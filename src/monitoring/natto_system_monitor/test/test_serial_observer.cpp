#include <natto_system_monitor/serial_diagnostics.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <gtest/gtest.h>
#include <unistd.h>

#include <thread>

namespace {
class StubSerial {
   public:
    StubSerial (const char *, int) : fd_ (open ("/dev/null", O_RDWR)) {}
    virtual ~StubSerial () {
        close (fd_);
    }
    int diagnostic_fd () const {
        return fd_;
    }
    virtual int write (unsigned char *data, unsigned int length) {
        ++calls;
        observed_data   = data;
        observed_length = length;
        errno           = EAGAIN;
        return result;
    }
    int            result = 0, calls = 0;
    unsigned char *observed_data   = nullptr;
    unsigned int   observed_length = 0;

   private:
    int fd_;
};
}  // namespace

TEST (SerialObserver, PreservesCallsAndReportsShortWriteFailureRecovery) {
    rclcpp::init (0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides ({rclcpp::Parameter ("diagnostic_updater.period", .02), rclcpp::Parameter ("serial_diagnostics.recovery_warn_sec", .1)});
    auto                                             node = std::make_shared<rclcpp::Node> ("serial_observer_test", options);
    natto_system_monitor::SerialDiagnostics          diagnostics (node.get (), "/dev/null");
    natto_system_monitor::ObservedSerial<StubSerial> serial ("unused", 0, diagnostics);
    diagnostic_msgs::msg::DiagnosticStatus           status;
    auto                                             sub        = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray> ("/diagnostics", 10, [&status] (diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        for (const auto &s : msg->status)
            if (s.name == "serial_observer_test: Hardware/Serial") status = s;
    });
    const auto wait = [&] (unsigned char level) {
        const auto end = std::chrono::steady_clock::now () + std::chrono::seconds (2);
        do {
            rclcpp::spin_some (node);
            if (!status.values.empty () && status.level == level) return true;
            std::this_thread::sleep_for (std::chrono::milliseconds (5));
        } while (std::chrono::steady_clock::now () < end);
        return false;
    };
    const auto value = [&status] (const std::string &key) {
        for (const auto &v : status.values)
            if (v.key == key) return v.value;
        return std::string ();
    };
    EXPECT_TRUE (wait (diagnostic_msgs::msg::DiagnosticStatus::OK));
    EXPECT_EQ (serial.calls, 0);
    unsigned char bytes[] = {1, 2, 3};
    serial.result         = 2;
    EXPECT_EQ (serial.write (bytes, 3), 2);
    EXPECT_EQ (errno, EAGAIN);
    EXPECT_EQ (serial.calls, 1);
    EXPECT_EQ (serial.observed_data, bytes);
    EXPECT_EQ (serial.observed_length, 3u);
    EXPECT_TRUE (wait (diagnostic_msgs::msg::DiagnosticStatus::WARN));
    EXPECT_EQ (value ("write_failure_count"), "1");
    serial.result = -1;
    EXPECT_EQ (serial.write (bytes, 3), -1);
    EXPECT_EQ (serial.write (bytes, 3), -1);
    EXPECT_TRUE (wait (diagnostic_msgs::msg::DiagnosticStatus::ERROR));
    EXPECT_EQ (value ("consecutive_write_errors"), "3");
    serial.result = 3;
    EXPECT_EQ (serial.write (bytes, 3), 3);
    EXPECT_TRUE (wait (diagnostic_msgs::msg::DiagnosticStatus::WARN));
    EXPECT_TRUE (wait (diagnostic_msgs::msg::DiagnosticStatus::OK));
    EXPECT_EQ (value ("write_success_count"), "1");
    EXPECT_EQ (value ("write_failure_count"), "3");
    EXPECT_EQ (value ("consecutive_write_errors"), "0");
    EXPECT_EQ (serial.calls, 4);
    rclcpp::shutdown ();
}
