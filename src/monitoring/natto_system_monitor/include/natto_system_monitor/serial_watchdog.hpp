#pragma once

#include <chrono>
#include <stdexcept>

namespace natto_system_monitor {

// Pure state tracking; the owning control node decides when to send a stop.
// Callers supply a monotonic time so ROS /clock cannot suspend the watchdog.
class SerialWatchdog {
   public:
    using Clock        = std::chrono::steady_clock;
    using Time         = Clock::time_point;
    using Milliseconds = std::chrono::milliseconds;
    enum class State {
        WaitingFeedback,
        FeedbackTimeout,
        Recovering,
        CommandTimeout,
        WaitingNeutral,
        Ready,
        WaitingUpdate,
        UpdateError
    };

    SerialWatchdog (Milliseconds feedback, Milliseconds startup, Milliseconds command, int recovery) : feedback_timeout_ (feedback), startup_timeout_ (startup), command_timeout_ (command), recovery_ (recovery) {
        if (feedback.count () <= 0 || startup.count () <= 0 || command.count () <= 0 || recovery < 1) throw std::invalid_argument ("Serial watchdog timeouts and recovery count must be positive");
    }

    // Return-code mode: one successful update is enough; no feedback-age deadline.
    SerialWatchdog (Milliseconds command, int update_failure_limit) : feedback_timeout_ (0), startup_timeout_ (0), command_timeout_ (command), recovery_ (1), update_failure_limit_ (update_failure_limit) {
        if (command.count () <= 0 || update_failure_limit < 1) throw std::invalid_argument ("Command timeout and serial update failure limit must be positive");
    }

    // Send-only devices: receive traffic is optional; command expiry still stops.
    explicit SerialWatchdog (Milliseconds command) : feedback_timeout_ (0), startup_timeout_ (0), command_timeout_ (command), recovery_ (1), require_feedback_ (false) {
        if (command.count () <= 0) throw std::invalid_argument ("Command timeout must be positive");
    }

    void reset (Time now) {
        last_feedback_ = now;
        last_command_  = now;
        feedback_seen_ = command_seen_ = armed_ = false;
        consecutive_                            = 0;
        update_failures_                        = 0;
        last_update_result_                     = -1;
    }

    State check (Time now) {
        if (require_feedback_ && (uses_update_results () ? update_failures_ >= update_failure_limit_ : now - last_feedback_ >= (feedback_seen_ ? feedback_timeout_ : startup_timeout_))) {
            armed_       = false;
            consecutive_ = 0;
            return uses_update_results () ? State::UpdateError : State::FeedbackTimeout;
        }
        const bool command_expired = command_seen_ && now - last_command_ >= command_timeout_;
        if (command_expired) armed_ = false;
        if (require_feedback_ && !feedback_seen_) return uses_update_results () ? State::WaitingUpdate : State::WaitingFeedback;
        if (require_feedback_ && consecutive_ < recovery_) return State::Recovering;
        if (command_expired) return State::CommandTimeout;
        return armed_ ? State::Ready : State::WaitingNeutral;
    }

    void feedback (Time now) {
        check (now);  // A late packet must not hide a deadline crossed between timer calls.
        feedback_seen_ = true;
        last_feedback_ = now;
        if (consecutive_ < recovery_) ++consecutive_;
    }

    void update_result (Time now, int result) {
        check (now);  // A recovered update must not preserve permission after a fault.
        last_update_result_ = result;
        if (result == 0) {
            update_failures_ = 0;
            feedback (now);
        } else if (update_failures_ < update_failure_limit_) {
            ++update_failures_;
        }
        check (now);
    }

    bool uses_update_results () const {
        return update_failure_limit_ > 0;
    }
    bool requires_feedback () const {
        return require_feedback_;
    }
    int update_failures () const {
        return update_failures_;
    }
    int update_failure_limit () const {
        return update_failure_limit_;
    }
    int last_update_result () const {
        return last_update_result_;
    }

    bool command (Time now, bool neutral) {
        check (now);
        command_seen_ = true;
        last_command_ = now;
        if (neutral && (!require_feedback_ || (feedback_seen_ && consecutive_ >= recovery_))) armed_ = true;
        return check (now) == State::Ready;
    }

    double feedback_age (Time now) const {
        return feedback_seen_ ? std::chrono::duration<double> (now - last_feedback_).count () : -1.0;
    }
    double command_age (Time now) const {
        return command_seen_ ? std::chrono::duration<double> (now - last_command_).count () : -1.0;
    }
    static const char *name (State state) {
        switch (state) {
            case State::WaitingFeedback:
                return "Waiting for first feedback";
            case State::FeedbackTimeout:
                return "Feedback timeout";
            case State::Recovering:
                return "Waiting for consecutive feedback";
            case State::CommandTimeout:
                return "Command timeout";
            case State::WaitingNeutral:
                return "Waiting for neutral command";
            case State::Ready:
                return "Ready";
            case State::WaitingUpdate:
                return "Waiting for successful serial update";
            case State::UpdateError:
                return "Consecutive serial update failures";
        }
        return "Unknown";
    }

   private:
    Milliseconds feedback_timeout_, startup_timeout_, command_timeout_;
    int          recovery_, consecutive_ = 0;
    int          update_failure_limit_ = 0, update_failures_ = 0, last_update_result_ = -1;
    bool         require_feedback_ = true;
    Time         last_feedback_{}, last_command_{};
    bool         feedback_seen_ = false, command_seen_ = false, armed_ = false;
};
}  // namespace natto_system_monitor
