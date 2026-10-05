#pragma once

#include <chrono>
#include <stdexcept>

namespace natto_system_monitor {

// Escalation above the fast stop/reconnect watchdog. Never reset on port open:
// only sustained valid feedback proves recovery. Time is monotonic.
class SerialRestartPolicy {
   public:
    using Clock        = std::chrono::steady_clock;
    using Time         = Clock::time_point;
    using Milliseconds = std::chrono::milliseconds;

    SerialRestartPolicy (bool enabled, int failure_count, Milliseconds timeout, Milliseconds recovery) : enabled_ (enabled), failure_count_ (failure_count), timeout_ (timeout), recovery_ (recovery) {
        if (failure_count < 1 || timeout.count () <= 0 || recovery.count () <= 0) throw std::invalid_argument ("Serial restart count and durations must be positive");
    }

    bool observe (Time now, bool healthy) {
        if (!enabled_) return false;
        if (healthy) {
            if (!healthy_) healthy_since_ = now;
            healthy_ = true;
            if (now - healthy_since_ >= recovery_) {
                fault_    = false;
                failures_ = 0;
            }
            return false;
        }
        healthy_ = false;
        if (!fault_) {
            fault_       = true;
            fault_since_ = now;
            next_check_  = now;
        }
        // A delayed callback counts once, never as multiple missed checks.
        if (now >= next_check_) {
            if (failures_ < failure_count_) ++failures_;
            next_check_ = now + std::chrono::seconds (1);
        }
        return failures_ >= failure_count_ && now - fault_since_ >= timeout_;
    }

   private:
    bool         enabled_;
    int          failure_count_, failures_ = 0;
    Milliseconds timeout_, recovery_;
    bool         fault_ = false, healthy_ = false;
    Time         fault_since_{}, healthy_since_{}, next_check_{};
};
}  // namespace natto_system_monitor
