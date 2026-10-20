#pragma once

#include "rack_controller.hpp"
#include <string_view>

namespace inject2026a {

// Raw STM counts: no degree, PPR or gear conversion in the trajectory generator.
class RackCountController {
 public:
    using Clock = RackController::Clock;
    using Time = RackController::Time;
    using Command = RackController::Command;
    struct Config {
        int64_t target = -30;
        double speed = 10.0;
        double acceleration = 20.0;
        int64_t tolerance = 1;
        std::chrono::milliseconds settle{100};
        double motion_timeout_s = 10.0;
        double homing_duty = -0.1;
        double manual_duty = 0.0; // Zero disables button2/3 Duty jogging.
        int manual_count_sign = 1; // Raw-count direction of button2 (away from origin).
        double homing_timeout_s = 10.0;
        std::chrono::milliseconds feedback_timeout{200}, switch_debounce{50};
        bool gains_ready = false;
        bool enforce_range = false;
        int64_t min_relative = 0, max_relative = 0;
        int64_t range_tolerance = -1; // -1 retains the arrival tolerance for count-mode callers.
    };

    explicit RackCountController(Config config) : config_(config) {
        if (std::abs(static_cast<long double>(config.target)) > 1e9L ||
            !std::isfinite(config.speed) || config.speed <= 0 ||
            !std::isfinite(config.acceleration) || config.acceleration <= 0 ||
            config.tolerance < 0 || config.tolerance > 1000000000 || config.settle.count() <= 0 ||
            config.range_tolerance < -1 || config.range_tolerance > 1000000000 ||
            !std::isfinite(config.motion_timeout_s) || config.motion_timeout_s <= 0 ||
            !std::isfinite(config.homing_duty) || config.homing_duty == 0 ||
            std::abs(config.homing_duty) > 1 || !std::isfinite(config.homing_timeout_s) ||
            !std::isfinite(config.manual_duty) || config.manual_duty < 0 || config.manual_duty > 1 ||
            (config.manual_count_sign != 1 && config.manual_count_sign != -1) ||
            config.homing_timeout_s <= 0 || config.feedback_timeout.count() <= 0 ||
            config.switch_debounce.count() < 0 ||
            (config.enforce_range && (config.min_relative > 0 || config.max_relative < 0 ||
                config.target < config.min_relative || config.target > config.max_relative ||
                config.min_relative < -1000000000 || config.max_relative > 1000000000)))
            throw std::invalid_argument("invalid rack count profile configuration");
    }
    bool configured() const { return config_.gains_ready; }
    bool homed() const { return homed_; }
    const char *state() const { return state_; }
    int64_t relative_count() const {
        const long double relative = static_cast<long double>(raw_) - origin_;
        return static_cast<int64_t>(std::clamp(relative,
            static_cast<long double>(std::numeric_limits<int64_t>::min()),
            static_cast<long double>(std::numeric_limits<int64_t>::max())));
    }
    int64_t target_relative_count() const { return target_relative_; }
    int64_t goal_relative_count() const { return goal_; }
    double reference_velocity() const { return velocity_; }
    double reference_position() const { return reference_; }
    struct Reference { double position, velocity, acceleration; };
    Reference reference_at(double elapsed) const {
        if (duration_ <= 0 || elapsed >= duration_) return {static_cast<double>(goal_), 0, 0};
        if (elapsed <= 0) return {from_, 0, direction_ * config_.acceleration};
        double traveled, velocity, acceleration;
        if (elapsed < acceleration_time_) {
            velocity = config_.acceleration * elapsed;
            traveled = 0.5 * config_.acceleration * elapsed * elapsed;
            acceleration = config_.acceleration;
        } else if (elapsed < acceleration_time_ + cruise_time_) {
            velocity = peak_;
            traveled = 0.5 * peak_ * acceleration_time_ + peak_ * (elapsed - acceleration_time_);
            acceleration = 0;
        } else {
            const double remaining = duration_ - elapsed;
            velocity = config_.acceleration * remaining;
            traveled = distance_ - 0.5 * config_.acceleration * remaining * remaining;
            acceleration = -config_.acceleration;
        }
        return {from_ + direction_ * std::clamp(traveled, 0.0, distance_),
                direction_ * velocity, direction_ * acceleration};
    }
    Reference current_reference() const { return reference_at(elapsed_); }
    double delayed_reference_position(double delay) const {
        return reference_at(elapsed_ - delay).position;
    }
    void reset() { *this = RackCountController(config_); }
    void stop() {
        running_ = holding_ = settling_ = manual_active_ = false;
        velocity_ = 0;
        need_neutral_ = true;
    }
    void motor_feedback(Time now, int64_t raw) {
        if (motor_seen_ && now - motor_time_ >= config_.feedback_timeout) lose_origin();
        raw_ = raw; motor_time_ = now; motor_seen_ = true;
    }
    void switch_feedback(Time now, bool contact) {
        const bool gap = switch_seen_ && now - switch_time_ >= config_.feedback_timeout;
        if (gap) lose_origin();
        if (!switch_seen_ || gap || contact != contact_) contact_since_ = now;
        contact_ = contact; switch_time_ = now; switch_seen_ = true;
    }
    bool fresh(Time now) const {
        return motor_seen_ && switch_seen_ && now - motor_time_ < config_.feedback_timeout &&
            now - switch_time_ < config_.feedback_timeout;
    }

    // start is a consumed Joy rising-edge event; held is the current button level.
    Command step(Time now, bool start, bool held, bool triangle, bool square, bool permitted) {
        return step(now, start, held, triangle, square, permitted, config_.target);
    }
    // Ignore starts during the trajectory; settling can accept a new destination.
    Command step(Time now, bool start, bool held, bool triangle, bool square, bool permitted,
                 int64_t requested_target) {
        if (!configured()) { stop(); state_ = "configuration_missing"; return {}; }
        if (!fresh(now)) { lose_origin(); state_ = "waiting_feedback"; return {}; }
        if (!contact_) contact_latched_ = false;
        // Do not redefine zero if the switch remains active or bounces during a move.
        if (contact_ && !contact_latched_ && !running_ && !holding_ && !manual_active_ &&
            switch_time_ - contact_since_ >= config_.switch_debounce) {
            origin_ = raw_; homed_ = contact_latched_ = true;
            fault_ = nullptr; homing_started_ = false;
            reference_ = velocity_ = 0; target_relative_ = 0;
            need_neutral_ = true;
        }
        if (homing_started_ && std::chrono::duration<double>(now - homing_since_).count() >=
            config_.homing_timeout_s) fault_ = "homing_fault";
        if (fault_) {
            // A motion deadline stops the motor but does not invalidate a fresh
            // encoder origin. Require neutral, then an explicit new press.
            // Range, homing and configuration faults remain latched.
            if (std::string_view(fault_) == "motion_timeout" && homed_ && permitted) {
                if (need_neutral_) {
                    if (!held && !triangle && !square) need_neutral_ = false;
                    state_ = fault_;
                    return {};
                }
                if ((start && !triangle && !square) ||
                    (config_.manual_duty > 0 && triangle != square && !held)) fault_ = nullptr;
            }
            if (fault_) {
                if (!permitted) stop();
                state_ = fault_;
                return {};
            }
        }
        if (!permitted) { stop(); state_ = "stopped"; return {}; }
        if (need_neutral_) {
            if (!held && !triangle && !square) need_neutral_ = false;
            state_ = homed_ ? "waiting_neutral" : "waiting_origin";
            return {};
        }
        if (!homed_) {
            state_ = "waiting_origin";
            if (triangle && !square && !held && !contact_) {
                if (!homing_started_) { homing_since_ = now; homing_started_ = true; }
                state_ = "homing";
                return {false, 0, static_cast<float>(config_.homing_duty)};
            }
            return {};
        }
        if (config_.enforce_range) {
            const long double measured = static_cast<long double>(raw_) - origin_;
            const auto tolerance = config_.range_tolerance >= 0 ? config_.range_tolerance : config_.tolerance;
            if (measured < static_cast<long double>(config_.min_relative) - tolerance ||
                measured > static_cast<long double>(config_.max_relative) + tolerance)
                return fail("travel_limit");
        }
        if (config_.manual_duty > 0 && (square || triangle)) {
            // Manual Duty overrides even a moving profile. Releasing it never
            // resumes the previous automatic destination or its PID hold.
            running_ = holding_ = settling_ = false;
            manual_active_ = true;
            capture_measured_reference();
            state_ = "manual_stopped";
            if (square == triangle) return {};
            if (triangle && contact_) return {}; // Stop on first switch contact.
            if (config_.enforce_range) {
                const auto measured = relative_count();
                const bool raw_increase = square == (config_.manual_count_sign > 0);
                if ((raw_increase && measured >= config_.max_relative) ||
                    (!raw_increase && measured <= config_.min_relative)) return {};
            }
            state_ = "manual";
            return {false, 0, static_cast<float>((square ? 1 : -1) * config_.manual_duty)};
        }
        if (manual_active_) {
            manual_active_ = false;
            capture_measured_reference();
        }
        const bool trajectory_complete = running_ &&
            std::chrono::duration<double>(now - motion_since_).count() >= duration_;
        if (start && (!running_ || trajectory_complete) && !triangle && !square) {
            if (std::abs(static_cast<long double>(requested_target)) > 1e9L ||
                (config_.enforce_range && (requested_target < config_.min_relative ||
                                          requested_target > config_.max_relative)))
                return fail("target_outside_range");
            goal_ = requested_target;
            const long double from = static_cast<long double>(raw_) - origin_;
            // Double trajectory arithmetic must represent every relative integer.
            if (std::abs(from) > 4503599627370496.0L || !absolute_target(goal_, final_raw_))
                return fail("count_overflow");
            from_ = static_cast<double>(from);
            distance_ = std::abs(static_cast<double>(goal_) - from_);
            direction_ = static_cast<double>(goal_) >= from_ ? 1.0 : -1.0;
            peak_ = std::min(config_.speed, std::sqrt(distance_ * config_.acceleration));
            acceleration_time_ = peak_ / config_.acceleration;
            cruise_time_ = peak_ > 0 ? std::max(0.0,
                (distance_ - peak_ * acceleration_time_) / peak_) : 0;
            duration_ = 2 * acceleration_time_ + cruise_time_;
            // Reject an impossible deadline before sending any motion command.
            if (duration_ + std::chrono::duration<double>(config_.settle).count() >=
                config_.motion_timeout_s) return fail("motion_timeout_configuration");
            motion_since_ = now; running_ = true; holding_ = settling_ = false;
        }
        if (!running_ && !holding_) { state_ = "ready"; return {}; }
        const double elapsed = std::chrono::duration<double>(now - motion_since_).count();
        elapsed_ = elapsed;
        if (running_) {
            if (elapsed >= config_.motion_timeout_s) return fail("motion_timeout");
            const auto reference = current_reference();
            velocity_ = reference.velocity;
            reference_ = reference.position;
            target_relative_ = static_cast<int64_t>(std::round(reference_));
            state_ = elapsed < duration_ ? "moving" : "settling";
            const bool reached = elapsed >= duration_ &&
                std::abs(static_cast<long double>(raw_) - final_raw_) <= config_.tolerance;
            if (!reached) settling_ = false;
            else {
                if (!settling_) { settling_ = true; settle_since_ = now; }
                if (now - settle_since_ >= config_.settle) {
                    running_ = false; holding_ = true;
                }
            }
        }
        if (holding_) state_ = "holding";
        int64_t target;
        if (!absolute_target(target_relative_, target)) return fail("count_overflow");
        return {true, target, 0};
    }

 private:
    void capture_measured_reference() {
        goal_ = target_relative_ = relative_count();
        from_ = reference_ = static_cast<double>(goal_);
        velocity_ = elapsed_ = duration_ = 0;
    }
    bool absolute_target(int64_t offset, int64_t &result) const {
        if ((offset > 0 && origin_ > std::numeric_limits<int64_t>::max() - offset) ||
            (offset < 0 && origin_ < std::numeric_limits<int64_t>::min() - offset)) return false;
        result = origin_ + offset;
        return true;
    }
    Command fail(const char *reason) { fault_ = reason; stop(); state_ = reason; return {}; }
    void lose_origin() {
        stop(); homed_ = contact_latched_ = homing_started_ = false;
    }
    Config config_;
    Time motor_time_{}, switch_time_{}, contact_since_{}, homing_since_{}, motion_since_{}, settle_since_{};
    int64_t raw_ = 0, origin_ = 0, final_raw_ = 0, target_relative_ = 0, goal_ = 0;
    double from_ = 0, distance_ = 0, direction_ = 1, peak_ = 0;
    double acceleration_time_ = 0, cruise_time_ = 0, duration_ = 0, reference_ = 0, velocity_ = 0, elapsed_ = 0;
    bool motor_seen_ = false, switch_seen_ = false, contact_ = false, contact_latched_ = false;
    bool homed_ = false, homing_started_ = false, running_ = false, holding_ = false;
    bool need_neutral_ = true, settling_ = false, manual_active_ = false;
    const char *state_ = "waiting_feedback", *fault_ = nullptr;
};
} // namespace inject2026a
