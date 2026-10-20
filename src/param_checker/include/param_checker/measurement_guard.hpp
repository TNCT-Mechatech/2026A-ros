#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace param_checker {
// A bounded mechanism must stop even if other motors continue to report feedback.
class MeasurementGuard {
 public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;
    MeasurementGuard(Time opened, std::chrono::milliseconds timeout, int64_t travel,
                     int switch_index, bool switch_active)
        : opened_(opened), timeout_(timeout), travel_(travel), switch_index_(switch_index),
          active_level_(switch_active) {
        if (timeout.count() <= 0 || travel < 0 || switch_index < -1 || switch_index > 3)
            throw std::invalid_argument("invalid measurement feedback, travel or switch limit");
    }
    void motor(Time now, int64_t count) {
        if (!motor_seen_) origin_ = count;
        motor_seen_ = true; motor_time_ = now;
        if (travel_ > 0 && std::abs(static_cast<long double>(count) - origin_) > travel_)
            throw std::runtime_error("measurement travel limit reached; gains not saved");
    }
    void switches(Time now, bool level) {
        switch_seen_ = true; switch_time_ = now;
        if (switch_index_ >= 0 && level == active_level_)
            throw std::runtime_error("measurement origin switch reached: input[" +
                std::to_string(switch_index_) + "]=" + (level ? "true" : "false") +
                ", active_level=" + (active_level_ ? "true" : "false") + "; gains not saved");
    }
    void check(Time now) const {
        if (now - (motor_seen_ ? motor_time_ : opened_) >= timeout_)
            throw std::runtime_error("target motor feedback timed out; gains not saved");
        if (switch_index_ >= 0 && now - (switch_seen_ ? switch_time_ : opened_) >= timeout_)
            throw std::runtime_error("limit switch feedback timed out; gains not saved");
    }
    bool ready() const { return motor_seen_ && (switch_index_ < 0 || switch_seen_); }
    // Initialization uses its own timeout, before measurement feedback is due.
    void start_feedback_deadline(Time now) { opened_ = now; }
    int switch_index() const { return switch_index_; }
 private:
    Time opened_, motor_time_{}, switch_time_{};
    std::chrono::milliseconds timeout_;
    int64_t travel_, origin_{0};
    int switch_index_;
    bool active_level_, motor_seen_{false}, switch_seen_{false};
};
} // namespace param_checker
