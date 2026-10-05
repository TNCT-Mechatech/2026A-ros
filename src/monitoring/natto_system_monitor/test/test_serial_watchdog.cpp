#include <natto_system_monitor/serial_watchdog.hpp>

#include <gtest/gtest.h>

using W = natto_system_monitor::SerialWatchdog;
using S = W::State;
using namespace std::chrono_literals;

TEST (SerialWatchdog, InitialSilenceExpiresAtStartupDeadline) {
    W             w (200ms, 1000ms, 300ms, 3);
    const W::Time start{};
    w.reset (start);
    EXPECT_EQ (w.check (start + 999ms), S::WaitingFeedback);
    EXPECT_FALSE (w.command (start + 999ms, false));
    EXPECT_EQ (w.check (start + 1000ms), S::FeedbackTimeout);
    EXPECT_FALSE (w.command (start + 1001ms, true));
    EXPECT_EQ (w.feedback_age (start + 1001ms), -1.0);
}

TEST (SerialWatchdog, SilenceExpiresEvenWithContinuingCommandsAndRequiresNeutralToRecover) {
    W             w (200ms, 1000ms, 300ms, 3);
    const W::Time start{};
    w.reset (start);
    w.feedback (start);
    w.feedback (start + 10ms);
    w.feedback (start + 20ms);
    EXPECT_FALSE (w.command (start + 21ms, false));
    EXPECT_TRUE (w.command (start + 22ms, true));
    EXPECT_TRUE (w.command (start + 219ms, false));
    EXPECT_EQ (w.check (start + 220ms), S::FeedbackTimeout);
    EXPECT_FALSE (w.command (start + 221ms, false));
    w.feedback (start + 230ms);
    EXPECT_FALSE (w.command (start + 231ms, true));
    w.feedback (start + 240ms);
    w.feedback (start + 250ms);
    EXPECT_EQ (w.check (start + 251ms), S::WaitingNeutral);
    EXPECT_FALSE (w.command (start + 252ms, false));
    EXPECT_TRUE (w.command (start + 253ms, true));
    EXPECT_TRUE (w.command (start + 254ms, false));
}

TEST (SerialWatchdog, LateFeedbackCannotHideTimeoutBetweenChecks) {
    W             w (200ms, 1000ms, 300ms, 1);
    const W::Time start{};
    w.reset (start);
    w.feedback (start);
    ASSERT_TRUE (w.command (start, true));
    w.feedback (start + 200ms);
    EXPECT_EQ (w.check (start + 200ms), S::WaitingNeutral);
    EXPECT_FALSE (w.command (start + 201ms, false));
}

TEST (SerialWatchdog, CommandSilenceStopsWithHealthyFeedback) {
    W             w (200ms, 1000ms, 300ms, 1);
    const W::Time start{};
    w.reset (start);
    w.feedback (start);
    ASSERT_TRUE (w.command (start, true));
    w.feedback (start + 100ms);
    w.feedback (start + 200ms);
    w.feedback (start + 299ms);
    EXPECT_EQ (w.check (start + 299ms), S::Ready);
    EXPECT_EQ (w.check (start + 300ms), S::CommandTimeout);
    EXPECT_FALSE (w.command (start + 301ms, false));
    EXPECT_TRUE (w.command (start + 302ms, true));
}

TEST (SerialWatchdog, ReconnectAndRecoveryGapsDiscardPriorAuthorization) {
    W             w (200ms, 1000ms, 300ms, 2);
    const W::Time start{};
    w.reset (start);
    w.feedback (start);
    w.feedback (start + 10ms);
    ASSERT_TRUE (w.command (start + 10ms, true));
    w.reset (start + 20ms);
    EXPECT_FALSE (w.command (start + 20ms, false));
    w.feedback (start + 30ms);
    w.feedback (start + 230ms);
    EXPECT_EQ (w.check (start + 230ms), S::Recovering);
    EXPECT_FALSE (w.command (start + 230ms, true));
}

TEST (SerialWatchdog, RejectsInvalidSettings) {
    EXPECT_THROW (W (0ms, 1000ms, 300ms, 3), std::invalid_argument);
    EXPECT_THROW (W (200ms, -1ms, 300ms, 3), std::invalid_argument);
    EXPECT_THROW (W (200ms, 1000ms, 0ms, 3), std::invalid_argument);
    EXPECT_THROW (W (200ms, 1000ms, 300ms, 0), std::invalid_argument);
}

TEST (SerialWatchdog, UpdateModeCountsMixedFailuresAndResetsOnZero) {
    W             w (300ms, 5);
    const W::Time start{};
    w.reset (start);
    EXPECT_EQ (w.check (start + 10s), S::WaitingUpdate);  // No elapsed-time receive limit.
    w.update_result (start + 10s, 0);
    ASSERT_TRUE (w.command (start + 10s, true));
    for (int result : {-1, -2, -3, -1}) w.update_result (start + 10010ms, result);
    EXPECT_EQ (w.update_failures (), 4);
    EXPECT_EQ (w.check (start + 10010ms), S::Ready);
    w.update_result (start + 10020ms, 0);
    EXPECT_EQ (w.update_failures (), 0);
    EXPECT_EQ (w.check (start + 10020ms), S::Ready);
    for (int i = 0; i < 5; ++i) w.update_result (start + 10030ms, -1);
    EXPECT_EQ (w.check (start + 10030ms), S::UpdateError);
    EXPECT_FALSE (w.command (start + 10030ms, false));
    w.update_result (start + 10040ms, 0);
    EXPECT_EQ (w.check (start + 10040ms), S::WaitingNeutral);
    EXPECT_FALSE (w.command (start + 10040ms, false));
    EXPECT_TRUE (w.command (start + 10040ms, true));
}

TEST (SerialWatchdog, UpdateModePreservesCommandDeadlineAndReconnectReset) {
    W             w (300ms, 5);
    const W::Time start{};
    w.reset (start);
    w.update_result (start, 0);
    ASSERT_TRUE (w.command (start, true));
    w.update_result (start + 1s, 0);
    EXPECT_EQ (w.check (start + 1s), S::CommandTimeout);
    EXPECT_FALSE (w.command (start + 1s, false));
    EXPECT_TRUE (w.command (start + 1s, true));
    w.update_result (start + 1010ms, -2);
    w.reset (start + 1020ms);
    EXPECT_EQ (w.update_failures (), 0);
    EXPECT_EQ (w.check (start + 1020ms), S::WaitingUpdate);
    EXPECT_FALSE (w.command (start + 1020ms, true));
    w.update_result (start + 1030ms, 0);
    EXPECT_FALSE (w.command (start + 1030ms, false));
    EXPECT_TRUE (w.command (start + 1030ms, true));
}

TEST (SerialWatchdog, UpdateModeRejectsInvalidSettings) {
    EXPECT_THROW (W (0ms, 5), std::invalid_argument);
    EXPECT_THROW (W (300ms, 0), std::invalid_argument);
    EXPECT_THROW (W (300ms, -1), std::invalid_argument);
}

TEST (SerialWatchdog, SendOnlyNeedsNoFeedbackButStillRequiresNeutralAndFreshCommands) {
    W             w (300ms);
    const W::Time start{};
    w.reset (start);
    EXPECT_FALSE (w.requires_feedback ());
    EXPECT_EQ (w.check (start + 10s), S::WaitingNeutral);
    EXPECT_FALSE (w.command (start + 10s, false));
    EXPECT_TRUE (w.command (start + 10s, true));
    for (int i = 1; i <= 100; ++i) {
        EXPECT_TRUE (w.command (start + 10s + i * 100ms, false));
    }
    EXPECT_EQ (w.feedback_age (start + 20s), -1.0);
    EXPECT_EQ (w.check (start + 20299ms), S::Ready);
    EXPECT_EQ (w.check (start + 20300ms), S::CommandTimeout);
    EXPECT_FALSE (w.command (start + 20301ms, false));
    EXPECT_TRUE (w.command (start + 20302ms, true));
    w.reset (start + 21s);
    EXPECT_FALSE (w.command (start + 21s, false));
    EXPECT_TRUE (w.command (start + 21s, true));
    EXPECT_THROW (W (0ms), std::invalid_argument);
}
