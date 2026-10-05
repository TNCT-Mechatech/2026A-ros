#include <natto_system_monitor/serial_restart.hpp>

#include <gtest/gtest.h>

using P = natto_system_monitor::SerialRestartPolicy;
using namespace std::chrono_literals;

TEST (SerialRestart, RequiresBothFailureCountAndElapsedTime) {
    P             p (true, 10, 10000ms, 3000ms);
    const P::Time start{};
    for (int ms = 0; ms < 10000; ms += 10) EXPECT_FALSE (p.observe (start + std::chrono::milliseconds (ms), false));
    EXPECT_TRUE (p.observe (start + 10000ms, false));
    P delayed (true, 10, 10000ms, 3000ms);
    EXPECT_FALSE (delayed.observe (start, false));
    EXPECT_FALSE (delayed.observe (start + 60000ms, false));
}

TEST (SerialRestart, StableRecoveryClearsHistory) {
    P             p (true, 10, 10000ms, 3000ms);
    const P::Time start{};
    for (int sec = 0; sec < 9; ++sec) EXPECT_FALSE (p.observe (start + std::chrono::seconds (sec), false));
    EXPECT_FALSE (p.observe (start + 9000ms, true));
    EXPECT_FALSE (p.observe (start + 12000ms, true));
    for (int sec = 13; sec < 23; ++sec) EXPECT_FALSE (p.observe (start + std::chrono::seconds (sec), false));
    EXPECT_TRUE (p.observe (start + 23000ms, false));
}

TEST (SerialRestart, BriefRecoveryDoesNotErasePersistentFault) {
    P             p (true, 10, 10000ms, 3000ms);
    const P::Time start{};
    for (int sec = 0; sec < 10; ++sec) {
        EXPECT_FALSE (p.observe (start + std::chrono::seconds (sec), false));
        EXPECT_FALSE (p.observe (start + std::chrono::seconds (sec) + 100ms, true));
    }
    EXPECT_TRUE (p.observe (start + 10000ms, false));
}

TEST (SerialRestart, HealthyAndDisabledNeverRestart) {
    P healthy (true, 10, 10000ms, 3000ms);
    P disabled (false, 10, 10000ms, 3000ms);
    for (int sec = 0; sec < 100; ++sec) {
        const auto time = P::Time{} + std::chrono::seconds (sec);
        EXPECT_FALSE (healthy.observe (time, true));
        EXPECT_FALSE (disabled.observe (time, false));
    }
}

TEST (SerialRestart, RejectsInvalidConfiguration) {
    EXPECT_THROW (P (true, 0, 10000ms, 3000ms), std::invalid_argument);
    EXPECT_THROW (P (true, 10, 0ms, 3000ms), std::invalid_argument);
    EXPECT_THROW (P (true, 10, 10000ms, 0ms), std::invalid_argument);
}

TEST (SerialRestart, OneSecondPolicyRestartsAtDeadlineWithoutExtraFailureChecks) {
    P             p (true, 1, 1000ms, 3000ms);
    const P::Time start{};
    for (int ms = 0; ms < 1000; ms += 10) EXPECT_FALSE (p.observe (start + std::chrono::milliseconds (ms), false));
    EXPECT_TRUE (p.observe (start + 1000ms, false));
}
