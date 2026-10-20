#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace inject2026a {

// No serial or ROS state: all deadlines use the caller's monotonic clock.
class RackController {
 public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;
    struct Config {
        double counts_per_degree = 0.0; // Signed, after STM encoder polarity.
        double max_angle_deg = 0.0;     // Zero means uncalibrated, motion disabled.
        double speed_deg_s = 15.0;
        double homing_duty = -0.1;
        double homing_timeout_s = 10.0;
        std::chrono::milliseconds feedback_timeout{200};
        std::chrono::milliseconds switch_debounce{50};
        bool gains_ready = false;
    };
    struct Command {
        bool position = false;
        int64_t target_count = 0;
        float duty = 0.0f;
        bool velocity = false;
        float target_rps = 0.0f;
    };

    explicit RackController(Config config) : config_(config) {
        if (!std::isfinite(config.counts_per_degree) || !std::isfinite(config.max_angle_deg) ||
            config.max_angle_deg < 0 || !std::isfinite(config.speed_deg_s) || config.speed_deg_s <= 0 ||
            !std::isfinite(config.homing_duty) || config.homing_duty == 0 ||
            std::abs(config.homing_duty) > 1 || !std::isfinite(config.homing_timeout_s) ||
            config.homing_timeout_s <= 0 || config.feedback_timeout.count() <= 0 ||
            config.switch_debounce.count() < 0 ||
            std::abs(static_cast<long double>(config.counts_per_degree) * config.max_angle_deg) >
                std::numeric_limits<int64_t>::max() / 2.0L)
            throw std::invalid_argument("invalid rack angle/homing configuration");
    }

    bool configured() const {
        return config_.gains_ready && config_.counts_per_degree != 0 && config_.max_angle_deg > 0;
    }
    void reset() {
        motor_seen_ = switch_seen_ = homed_ = active_ = contact_latched_ = homing_started_ = fault_ = false;
        need_neutral_ = true;
        tick_seen_ = false;
        target_deg_ = 0;
        state_ = "waiting_feedback";
    }
    void stop() {
        active_ = false;
        need_neutral_ = true;
    }
    void motor_feedback(Time now, int64_t count) {
        raw_count_ = count;
        motor_time_ = now;
        motor_seen_ = true;
    }
    void switch_feedback(Time now, bool active) {
        // A gap in switch reports must not count as continuous contact.
        if (!switch_seen_ || active != switch_active_ ||
            now - switch_time_ >= config_.feedback_timeout)
            contact_since_ = now;
        switch_active_ = active;
        switch_time_ = now;
        switch_seen_ = true;
    }
    bool fresh(Time now) const {
        return motor_seen_ && switch_seen_ && now - motor_time_ < config_.feedback_timeout &&
            now - switch_time_ < config_.feedback_timeout;
    }
    bool homed() const { return homed_; }
    bool fault() const { return fault_; }
    const char *state() const { return state_; }
    double target_angle() const { return target_deg_; }
    double measured_angle() const {
        if (!homed_ || config_.counts_per_degree == 0)
            return std::numeric_limits<double>::quiet_NaN();
        return static_cast<double>((static_cast<long double>(raw_count_) - origin_count_) /
                                   config_.counts_per_degree);
    }

    Command step(Time now, bool increase, bool decrease, bool permitted) {
        const double dt = tick_seen_ ? std::clamp(std::chrono::duration<double>(now - tick_time_).count(), 0.0, 0.05) : 0;
        tick_time_ = now;
        tick_seen_ = true;
        if (!configured()) { stop(); state_ = "configuration_missing"; return {}; }
        if (!fresh(now)) {
            // Neither other motors nor switch-only traffic can keep a rack origin valid.
            homed_ = active_ = contact_latched_ = false;
            need_neutral_ = true;
            state_ = "waiting_feedback";
            return {};
        }
        if (!switch_active_) contact_latched_ = false;
        if (switch_active_ && !contact_latched_ &&
            switch_time_ - contact_since_ >= config_.switch_debounce) {
            // Keep the STM raw coordinate continuous. The switch defines the ROS zero.
            origin_count_ = raw_count_;
            homed_ = contact_latched_ = true;
            homing_started_ = fault_ = active_ = false;
            target_deg_ = 0;
            need_neutral_ = true;
        }
        if (homing_started_ && std::chrono::duration<double>(now - homing_since_).count() >=
            config_.homing_timeout_s) fault_ = true;
        if (fault_) { stop(); state_ = "homing_fault"; return {}; }
        if (!permitted) { stop(); state_ = "stopped"; return {}; }
        if (need_neutral_) {
            if (!increase && !decrease) need_neutral_ = false;
            state_ = homed_ ? "waiting_neutral" : "waiting_origin";
            return {};
        }
        if (!homed_) {
            state_ = "waiting_origin";
            if (decrease && !increase && !switch_active_) {
                if (!homing_started_) { homing_since_ = now; homing_started_ = true; }
                state_ = "homing";
                return {false, 0, static_cast<float>(config_.homing_duty)};
            }
            // Stop immediately on contact, before debounce finishes.
            return {};
        }
        if (!active_) {
            // Resume from the measured position, never from a pre-timeout target.
            target_deg_ = std::clamp(measured_angle(), 0.0, config_.max_angle_deg);
            active_ = true;
        }
        target_deg_ = std::clamp(target_deg_ + (int(increase) - int(decrease)) *
                                config_.speed_deg_s * dt, 0.0, config_.max_angle_deg);
        const long double count = static_cast<long double>(origin_count_) +
                                  target_deg_ * config_.counts_per_degree;
        if (count < std::numeric_limits<int64_t>::min() || count > std::numeric_limits<int64_t>::max()) {
            fault_ = true; stop(); state_ = "count_overflow"; return {};
        }
        state_ = "holding";
        return {true, static_cast<int64_t>(std::round(count)), 0};
    }

 private:
    Config config_;
    Time motor_time_{}, switch_time_{}, contact_since_{}, homing_since_{}, tick_time_{};
    int64_t raw_count_ = 0, origin_count_ = 0;
    double target_deg_ = 0;
    bool motor_seen_ = false, switch_seen_ = false, switch_active_ = false;
    bool homed_ = false, active_ = false, contact_latched_ = false;
    bool homing_started_ = false, fault_ = false, need_neutral_ = true, tick_seen_ = false;
    const char *state_ = "waiting_feedback";
};
} // namespace inject2026a
