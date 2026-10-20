#pragma once
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <natto_system_monitor/serial_watchdog.hpp>
#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <poll.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>

namespace natto_system_monitor {
// All accesses run in the owning node's default mutually exclusive callback group.
class SerialDiagnostics {
   public:
    SerialDiagnostics (rclcpp::Node *node, const std::string &path) : node_ (node), path_ (path), updater_ (node, 1.0, diagnostic_msgs::msg::DiagnosticStatus::STALE) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        error_limit_         = node->declare_parameter<int> ("serial_diagnostics.error_limit", 3, descriptor);
        recovery_sec_        = node->declare_parameter<double> ("serial_diagnostics.recovery_warn_sec", 2.0, descriptor);
        idle_sec_            = node->declare_parameter<double> ("serial_diagnostics.idle_sec", 1.0, descriptor);
        if (error_limit_ < 1 || !std::isfinite (recovery_sec_) || recovery_sec_ < 0 || !std::isfinite (idle_sec_) || idle_sec_ <= 0) throw std::invalid_argument ("Invalid serial_diagnostics thresholds");
        updater_.setHardwareID (path);
        updater_.add ("Hardware/Serial", this, &SerialDiagnostics::report);
    }
    void opened (int fd) {
        fd_ = fd;
    }
    void reconnect_attempt (uint64_t count) {
        reconnect_attempts_ = count;
    }
    void observe_watchdog (SerialWatchdog *watchdog) {
        watchdog_ = watchdog;
    }
    void configuration_status(bool ready, bool failed, const std::string &message) {
        configuration_ready_ = ready;
        configuration_failed_ = failed;
        configuration_message_ = message;
    }
    void write_result (int result, unsigned int length, int error) {
        const auto time = std::chrono::steady_clock::now ();
        last_result_    = result;
        expected_bytes_ = length;
        last_write_     = time;
        if (result >= 0 && static_cast<unsigned int> (result) == length) {
            ++success_;
            consecutive_      = 0;
            last_success_     = time;
            last_errno_       = 0;
            last_success_ros_ = node_->now ().seconds ();
        } else {
            ++failure_;
            ++consecutive_;
            last_failure_ = time;
            last_errno_   = result < 0 ? error : 0;
        }
    }

   private:
    void report (diagnostic_updater::DiagnosticStatusWrapper &s) {
        using H         = diagnostic_msgs::msg::DiagnosticStatus;
        const auto time = std::chrono::steady_clock::now ();
        const auto age  = [time] (auto stamp) { return std::chrono::duration<double> (time - stamp).count (); };
        const bool open = fd_ >= 0 && fcntl (fd_, F_GETFD) != -1;
        pollfd     descriptor{fd_, 0, 0};
        const int  result       = open ? poll (&descriptor, 1, 0) : -1;
        const bool disconnected = result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL));
        const bool idle         = success_ + failure_ == 0 || age (last_write_) > idle_sec_;
        const auto state        = watchdog_ ? watchdog_->check (time) : SerialWatchdog::State::Ready;
        if (watchdog_) {
            s.add ("watchdog_state", SerialWatchdog::name (state));
            s.add ("receive_monitoring", watchdog_->requires_feedback ());
            if (watchdog_->uses_update_results ()) {
                s.add ("serial_update_result", watchdog_->last_update_result ());
                s.add ("serial_update_failure_count", watchdog_->update_failures ());
                s.add ("serial_update_failure_limit", watchdog_->update_failure_limit ());
                s.add ("serial_update_age_sec", watchdog_->feedback_age (time));
            } else {
                s.add ("feedback_age_sec", watchdog_->feedback_age (time));
            }
            s.add ("command_age_sec", watchdog_->command_age (time));
        }
        s.add ("reconnect_attempts", reconnect_attempts_);
        s.add("configuration_ready", configuration_ready_);
        s.add("configuration_state", configuration_message_);
        s.add ("device", path_);
        s.add ("port_open", open);
        s.add ("port_disconnected", disconnected);
        s.add ("write_success_count", success_);
        s.add ("write_failure_count", failure_);
        s.add ("consecutive_write_errors", consecutive_);
        s.add ("last_write_result", last_result_);
        s.add ("expected_write_bytes", expected_bytes_);
        s.add ("last_write_errno", last_errno_);
        s.add ("last_write_error", last_errno_ ? std::strerror (last_errno_) : (last_result_ >= 0 && static_cast<unsigned int> (last_result_) < expected_bytes_ ? "short write" : "none"));
        s.add ("last_success_ros_sec", std::to_string (last_success_ros_));
        s.add ("last_success_age_sec", success_ ? age (last_success_) : -1.0);
        s.add ("operational_state", !open || disconnected || !configuration_ready_ || state != SerialWatchdog::State::Ready ? "STOPPED" : idle ? "IDLE" : "RUNNING");
        if (!open || disconnected)
            s.summary (H::ERROR, "Serial port unavailable/disconnected");
        else if (result < 0)
            s.summary (H::STALE, "Unable to inspect serial port state");
        else if (!configuration_ready_)
            s.summary(configuration_failed_ ? H::ERROR : H::WARN, configuration_message_);
        else if (state == SerialWatchdog::State::FeedbackTimeout || state == SerialWatchdog::State::UpdateError)
            s.summary (H::ERROR, SerialWatchdog::name (state));
        else if (consecutive_ >= static_cast<uint64_t> (error_limit_))
            s.summary (H::ERROR, "Repeated serial write failures");
        else if (state != SerialWatchdog::State::Ready)
            s.summary (H::WARN, SerialWatchdog::name (state));
        else if (consecutive_ > 0 || (failure_ && age (last_failure_) < recovery_sec_))
            s.summary (H::WARN, "Transient serial write failure");
        else
            s.summary (H::OK, "Serial port open; writes healthy (not a firmware acknowledgement)");
    }
    rclcpp::Node                         *node_;
    SerialWatchdog                       *watchdog_ = nullptr;
    bool configuration_ready_{true}, configuration_failed_{false};
    std::string configuration_message_{"Not monitored"};
    std::string                           path_;
    diagnostic_updater::Updater           updater_;
    int                                   fd_ = -1, error_limit_ = 3, last_result_ = 0, last_errno_ = 0;
    unsigned int                          expected_bytes_ = 0;
    uint64_t                              success_ = 0, failure_ = 0, consecutive_ = 0, reconnect_attempts_ = 0;
    double                                recovery_sec_ = 2, idle_sec_ = 1, last_success_ros_ = 0;
    std::chrono::steady_clock::time_point last_write_{}, last_success_{}, last_failure_{};
};
// A transparent decorator: identical arguments, one call, same return and errno.
template <class Base>
class ObservedSerial : public Base {
   public:
    template <class Baud>
    ObservedSerial (const char *path, Baud baud, SerialDiagnostics &diagnostics) : Base (path, baud), diagnostics_ (diagnostics) {
        diagnostics_.opened (this->diagnostic_fd ());
    }
    int write (unsigned char *data, unsigned int length) override {
        const int result      = Base::write (data, length);
        const int saved_errno = errno;
        diagnostics_.write_result (result, length, saved_errno);
        errno = saved_errno;
        return result;
    }

   private:
    SerialDiagnostics &diagnostics_;
};
}  // namespace natto_system_monitor
