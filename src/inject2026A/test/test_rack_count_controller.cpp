#include "rack_angle_profile.hpp"
#include "rack_count_controller.hpp"
#include "rack_velocity_profile.hpp"

#include <gtest/gtest.h>

#include <string>

using Rack = inject2026a::RackCountController;
using namespace std::chrono_literals;
namespace {
Rack::Config config () {
    Rack::Config c;
    c.gains_ready = true;
    return c;
}
struct Rig {
    Rack       rack;
    Rack::Time now{};
    int64_t    origin = (int64_t{1} << 60) + 199, raw = origin;
    bool       contact = false, follow = false;
    explicit Rig (Rack::Config c = config ()) : rack (c) {}
    Rack::Command tick (bool start = false, bool held = false, bool permitted = true, bool motor = true, bool switches = true, bool triangle = false, bool square = false) {
        now += 10ms;
        if (motor) rack.motor_feedback (now, raw);
        if (switches) rack.switch_feedback (now, contact);
        auto command = rack.step (now, start, held, triangle, square, permitted);
        if (follow && command.position) raw = command.target_count;
        return command;
    }
    void home () {
        tick ();
        contact = true;
        for (int i = 0; i < 8; ++i) tick ();
        EXPECT_TRUE (rack.homed ());
        EXPECT_EQ (rack.relative_count (), 0);
    }
};
}  // namespace

TEST (RackCount, DebounceOriginAndRequireNewPressAfterNeutral) {
    Rig r;
    EXPECT_FALSE (r.tick (true, true).position);
    r.contact = true;
    for (int i = 0; i < 5; ++i) r.tick (false, true);
    EXPECT_FALSE (r.rack.homed ());
    r.tick (false, true);
    EXPECT_TRUE (r.rack.homed ());
    EXPECT_FALSE (r.tick (false, true).position);
    r.tick ();
    EXPECT_FALSE (r.tick (false, true).position);  // Held level alone is not a new press.
    EXPECT_TRUE (r.tick (true, true).position);
}
TEST (RackCount, TrapezoidBoundsContinuousReferenceAndExact64BitTarget) {
    Rig r;
    r.home ();
    r.follow     = true;
    auto command = r.tick (true, true);
    EXPECT_EQ (command.target_count, r.origin);
    double previous_position = 0, previous_velocity = 0;
    bool   cruise = false, decelerated = false;
    for (int i = 0; i < 370; ++i) {
        command = r.tick (false, true);
        ASSERT_TRUE (command.position);
        EXPECT_LE (command.target_count, r.origin);
        EXPECT_GE (command.target_count, r.origin - 30);
        EXPECT_LE (r.rack.reference_position (), previous_position + 1e-9);
        EXPECT_LE (std::abs (r.rack.reference_velocity ()), 10.0 + 1e-9);
        EXPECT_LE (std::abs (r.rack.reference_velocity () - previous_velocity), 0.2 + 1e-9);
        cruise |= std::abs (r.rack.reference_velocity () + 10) < 1e-9;
        decelerated |= cruise && r.rack.reference_velocity () > -9;
        previous_position = r.rack.reference_position ();
        previous_velocity = r.rack.reference_velocity ();
    }
    EXPECT_TRUE (cruise);
    EXPECT_TRUE (decelerated);
    EXPECT_EQ (command.target_count, r.origin - 30);
    EXPECT_EQ (r.rack.relative_count (), -30);
    EXPECT_STREQ (r.rack.state (), "holding");
    for (int i = 0; i < 1000; ++i) EXPECT_EQ (r.tick (false, true).target_count, r.origin - 30);
    // Switch remained true throughout; the origin must never follow moving raw counts.
    EXPECT_EQ (r.rack.relative_count (), -30);
}
TEST (RackCount, ShortMoveUsesTriangleAndStartsFromMeasuredPosition) {
    auto c   = config ();
    c.target = -2;
    Rig r (c);
    r.home ();
    r.follow = true;
    r.tick (true, true);
    double peak = 0;
    for (int i = 0; i < 80; ++i) {
        r.tick (false, true);
        peak = std::max (peak, std::abs (r.rack.reference_velocity ()));
    }
    EXPECT_LT (peak, c.speed);
    EXPECT_NEAR (peak, std::sqrt (2 * c.acceleration), 0.2);
    EXPECT_EQ (r.raw, r.origin - 2);
    r.tick (false, false, false);  // Stop must not resume the old target.
    r.raw = r.origin - 1;
    r.tick ();
    EXPECT_FALSE (r.tick ().position);
    EXPECT_EQ (r.tick (true, true).target_count, r.raw);
}
TEST (RackCount, FeedbackLossAndReconnectLoseOriginAndCannotResumeHeldButton) {
    for (bool motor_loss : {true, false}) {
        Rig r;
        r.home ();
        r.contact = false;
        r.tick (true, true);
        for (int i = 0; i < 22; ++i) r.tick (false, true, true, !motor_loss, motor_loss);
        EXPECT_FALSE (r.rack.homed ());
        EXPECT_FALSE (r.tick (false, true).position);
        r.rack.reset ();
        EXPECT_FALSE (r.tick (true, true).position);
    }
}
TEST (RackCount, JoyTimeoutRequiresReleaseAndNewEdge) {
    Rig r;
    r.home ();
    r.contact = false;
    r.tick (true, true);
    EXPECT_FALSE (r.tick (false, true, false).position);
    EXPECT_FALSE (r.tick (false, true).position);
    r.tick ();
    EXPECT_FALSE (r.tick ().position);
    EXPECT_TRUE (r.tick (true, true).position);
}
TEST (RackCount, StalledMotorTimesOutWithoutOverwritingZeroOrAutoRestart) {
    auto c             = config ();
    c.motion_timeout_s = 4;
    Rig r (c);
    r.home ();
    r.contact = false;
    r.tick (true, true);
    for (int i = 0; i < 405; ++i) r.tick (false, true);
    EXPECT_STREQ (r.rack.state (), "motion_timeout");
    EXPECT_FALSE (r.tick (true, true).position);  // Held through the fault cannot restart.
    EXPECT_FALSE (r.tick ().position);
    EXPECT_FALSE (r.tick (false, true).position);  // Level alone is insufficient.
    EXPECT_STREQ (r.rack.state (), "motion_timeout");
    EXPECT_TRUE (r.tick (true, true).position);
    EXPECT_STREQ (r.rack.state (), "moving");
    EXPECT_EQ (r.rack.relative_count (), 0);
    for (int i = 0; i < 405; ++i) r.tick (false, true);
    r.contact = true;
    for (int i = 0; i < 8; ++i) r.tick ();
    EXPECT_STREQ (r.rack.state (), "ready");
}
TEST (RackCount, SettlingAcceptsNewTargetFromActualPositionWithoutWaitingForTimeout) {
    auto c         = config ();
    c.target       = 25;
    c.speed        = 15;
    c.acceleration = 30;
    Rig r (c);
    r.home ();
    r.contact = false;
    r.tick (true, true);
    r.raw = r.origin + 27;  // Stiction leaves the old goal outside tolerance.
    for (int i = 0; i < 220; ++i) r.tick ();
    ASSERT_STREQ (r.rack.state (), "settling");
    auto command = r.rack.step (r.now, true, true, false, false, true, 15);
    ASSERT_TRUE (command.position);
    EXPECT_STREQ (r.rack.state (), "moving");
    EXPECT_EQ (command.target_count, r.raw);
    EXPECT_EQ (r.rack.goal_relative_count (), 15);
    EXPECT_DOUBLE_EQ (r.rack.reference_velocity (), 0);
    inject2026a::RackVelocityProfile outer ({8192, 5.0966919e-5, 0.06, 0.3024, 0.03936, 0.9966671});
    EXPECT_LT (outer.command (r.rack, command).target_rps, 0);
    r.tick ();
    EXPECT_LT (r.rack.reference_velocity (), 0);
}
TEST (RackCount, MotionTimeoutRecoveryStillRequiresFreshFeedbackNeutralAndValidRange) {
    auto c             = config ();
    c.target           = 10;
    c.speed            = 15;
    c.acceleration     = 30;
    c.motion_timeout_s = 2;
    c.enforce_range    = true;
    c.max_relative     = 30;
    Rig r (c);
    r.home ();
    r.contact = false;
    r.tick (true, true);
    for (int i = 0; i < 205; ++i) r.tick (false, true);
    ASSERT_STREQ (r.rack.state (), "motion_timeout");
    r.tick ();
    EXPECT_FALSE (r.tick (true, true, false).position);  // Watchdog revokes neutral.
    EXPECT_FALSE (r.tick (true, true).position);
    r.tick ();
    r.raw = r.origin + 40;
    EXPECT_FALSE (r.tick (true, true).position);
    EXPECT_STREQ (r.rack.state (), "travel_limit");
    r.raw = r.origin + 20;
    r.tick ();
    EXPECT_FALSE (r.tick (true, true).position);  // Travel fault remains latched.

    Rig lost (c);
    lost.home ();
    lost.contact = false;
    lost.tick (true, true);
    for (int i = 0; i < 205; ++i) lost.tick (false, true);
    lost.tick ();
    for (int i = 0; i < 22; ++i) lost.tick (false, false, true, false, true);
    EXPECT_FALSE (lost.rack.homed ());
    EXPECT_FALSE (lost.tick (true, true).position);
}
TEST (RackCount, GatesGainsInvalidParametersAndCountOverflow) {
    auto c        = config ();
    c.gains_ready = false;
    Rig unconfigured (c);
    unconfigured.tick ();
    EXPECT_FALSE (unconfigured.tick (true, true).position);
    c              = config ();
    c.acceleration = 0;
    EXPECT_THROW (Rack{c}, std::invalid_argument);
    c                  = config ();
    c.motion_timeout_s = 1;
    Rig short_deadline (c);
    short_deadline.home ();
    EXPECT_FALSE (short_deadline.tick (true, true).position);
    EXPECT_STREQ (short_deadline.rack.state (), "motion_timeout_configuration");
    Rig overflow;
    overflow.origin = overflow.raw = std::numeric_limits<int64_t>::min () + 1;
    overflow.home ();
    EXPECT_FALSE (overflow.tick (true, true).position);
    EXPECT_STREQ (overflow.rack.state (), "count_overflow");
}
TEST (RackCount, HomingStopsOnContactAndLatchesTimeout) {
    auto c             = config ();
    c.homing_timeout_s = 0.2;
    Rig r (c);
    r.tick ();
    EXPECT_LT (r.tick (false, false, true, true, true, true).duty, 0);
    r.contact = true;
    EXPECT_EQ (r.tick (false, false, true, true, true, true).duty, 0);
    EXPECT_FALSE (r.rack.homed ());
    r.contact = false;
    for (int i = 0; i < 21; ++i) r.tick ();
    EXPECT_STREQ (r.rack.state (), "homing_fault");
    EXPECT_EQ (r.tick (false, false, true, true, true, true).duty, 0);
}

TEST (RackManual, DutyOverridesAutomaticMoveAndReleaseDoesNotResumeIt) {
    auto c        = config ();
    c.manual_duty = 0.2;
    Rig r (c);
    r.home ();
    r.contact = false;
    EXPECT_TRUE (r.tick (true, true).position);
    inject2026a::RackVelocityProfile outer ({8192, 5.0966919e-5, 0.06, 0.3024, 0.03936, 0.9966671});
    auto                             command = outer.command (r.rack, r.tick (false, true, true, true, true, false, true));
    EXPECT_FLOAT_EQ (command.duty, 0.2f);
    EXPECT_FALSE (command.position);
    EXPECT_FALSE (command.velocity);
    EXPECT_STREQ (r.rack.state (), "manual");
    r.raw = r.origin + 7;
    EXPECT_FLOAT_EQ (r.tick (false, true, true, true, true, true).duty, -0.2f);
    command = r.tick (false, true);
    EXPECT_FLOAT_EQ (command.duty, 0);
    EXPECT_FALSE (command.position);
    EXPECT_STREQ (r.rack.state (), "ready");
    EXPECT_EQ (r.rack.goal_relative_count (), 7);
    for (int i = 0; i < 50; ++i) EXPECT_FALSE (r.tick (false, true).position);
    r.tick ();
    command = r.tick (true, true);
    EXPECT_TRUE (command.position);
    EXPECT_EQ (command.target_count, r.raw);
    EXPECT_EQ (r.rack.goal_relative_count (), -30);
}

TEST (RackManual, RespectsSwitchBothCountDirectionsAndWatchdogNeutral) {
    for (double sign : {-1.0, 1.0}) {
        auto c        = config ();
        c.manual_duty = 0.2;
        inject2026a::RackAngleProfile angles{sign * 8192 / 360, 30, 15, 30, 0.2};
        Rig                           r (angles.convert (c, 15));
        r.home ();
        EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, true).duty, 0);
        EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, false, true).duty, 0.2f);
        r.raw = r.origin + static_cast<int64_t> (sign * 100);
        for (int i = 0; i < 10; ++i) r.tick (false, false, true, true, true, false, true);
        EXPECT_EQ (r.rack.relative_count (), sign * 100);  // Active switch cannot drag the origin.
        r.contact = false;
        EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, true).duty, -0.2f);
        EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, true, true).duty, 0);
        r.raw = r.origin + angles.target_count (30);
        EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, false, true).duty, 0);
        EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, true).duty, -0.2f);
        r.raw = r.origin;
        EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, true).duty, 0);
        r.raw = r.origin + static_cast<int64_t> (sign * 100);
        EXPECT_FLOAT_EQ (r.tick (false, false, false, true, true, false, true).duty, 0);
        EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, false, true).duty, 0);
        r.tick ();
        EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, false, true).duty, 0.2f);
        for (int i = 0; i < 22; ++i) r.tick (false, false, true, false, true, false, true);
        EXPECT_FALSE (r.rack.homed ());
        EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, false, true).duty, 0);
    }
}

TEST (RackManual, TimeoutRequiresReleaseThenAllowsManualAndInvalidDutyIsRejected) {
    auto c             = config ();
    c.manual_duty      = 0.2;
    c.motion_timeout_s = 4;
    Rig r (c);
    r.home ();
    r.contact = false;
    r.tick (true, true);
    for (int i = 0; i < 405; ++i) r.tick (false, true);
    EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, false, true).duty, 0);
    r.tick ();
    EXPECT_FLOAT_EQ (r.tick (false, false, true, true, true, false, true).duty, 0.2f);
    for (double duty : {-0.2, 1.1, std::numeric_limits<double>::quiet_NaN ()}) {
        c.manual_duty = duty;
        EXPECT_THROW (Rack{c}, std::invalid_argument);
    }
}

TEST (RackAngles, ConvertsBothDirectionsAndUsesLoadedEncoderScale) {
    for (double sign : {-1.0, 1.0}) {
        inject2026a::RackAngleProfile angles{sign * 8192 / 360, 20, 5, 10, 0.2};
        const auto                    counts = angles.convert (config (), 5);
        EXPECT_EQ (counts.target, static_cast<int64_t> (sign * 114));
        EXPECT_EQ (angles.target_count (10), static_cast<int64_t> (sign * 228));
        EXPECT_EQ (angles.target_count (0), 0);
        EXPECT_NEAR (counts.speed, 8192 * 5 / 360.0, 1e-9);
        EXPECT_NEAR (counts.acceleration, 8192 * 10 / 360.0, 1e-9);
        EXPECT_EQ (counts.tolerance, 4);
        EXPECT_TRUE (counts.enforce_range);
        EXPECT_LT (counts.min_relative, counts.max_relative);
    }
    inject2026a::RackAngleProfile geared{8192 * 2.0 / 360, 20, 5, 10, 0.2};
    EXPECT_EQ (geared.target_count (5), 228);
    EXPECT_THROW (geared.target_count (-1), std::invalid_argument);
    EXPECT_THROW (geared.target_count (21), std::invalid_argument);
    EXPECT_THROW (geared.target_count (std::numeric_limits<double>::quiet_NaN ()), std::invalid_argument);
    geared.counts_per_degree = 0;
    EXPECT_THROW (geared.convert (config (), 5), std::invalid_argument);
}

TEST (RackAngles, CapturesSelectedTargetIgnoresMidMoveChangesAndCanReturnToZero) {
    inject2026a::RackAngleProfile angles{-8192 / 360.0, 20, 5, 10, 0.2};
    Rig                           r (angles.convert (config (), 5));
    r.home ();
    r.follow = true;
    r.tick (true, true);
    // A second start during a move must not change the active destination.
    for (int i = 0; i < 170; ++i) {
        r.now += 10ms;
        r.rack.motor_feedback (r.now, r.raw);
        r.rack.switch_feedback (r.now, r.contact);
        const auto cmd = r.rack.step (r.now, i == 0, true, false, false, true, angles.target_count (10));
        ASSERT_TRUE (cmd.position);
        r.raw = cmd.target_count;
        if (std::string (r.rack.state ()) == "holding") break;
    }
    EXPECT_EQ (r.raw, r.origin - 114);
    EXPECT_STREQ (r.rack.state (), "holding");
    r.contact = false;
    for (int i = 0; i < 180; ++i) {
        r.now += 10ms;
        r.rack.motor_feedback (r.now, r.raw);
        r.rack.switch_feedback (r.now, r.contact);
        const auto cmd = r.rack.step (r.now, i == 0, i == 0, false, false, true, 0);
        ASSERT_TRUE (cmd.position);
        r.raw = cmd.target_count;
    }
    EXPECT_EQ (r.raw, r.origin);
    EXPECT_STREQ (r.rack.state (), "holding");
}

TEST (RackAngles, StopsAtTravelLimitAndRejectsInvalidSelectedTarget) {
    inject2026a::RackAngleProfile angles{-8192 / 360.0, 20, 5, 10, 0.2};
    Rig                           r (angles.convert (config (), 5));
    r.home ();
    r.contact = false;
    r.raw     = r.origin - 460;  // Beyond the 20-degree endpoint plus tolerance.
    EXPECT_FALSE (r.tick (true, true).position);
    EXPECT_STREQ (r.rack.state (), "travel_limit");
    Rig bad (angles.convert (config (), 5));
    bad.home ();
    EXPECT_FALSE (bad.rack.step (bad.now, true, true, false, false, true, -500).position);
    EXPECT_STREQ (bad.rack.state (), "target_outside_range");
}

TEST (RackAngles, WiderRangeToleranceAllowsOriginDriftButStopsBeyondBothBoundaries) {
    for (double sign : {-1.0, 1.0}) {
        inject2026a::RackAngleProfile angles{sign * 8192 / 360, 30, 15, 30, 0.2, 1.0};
        const auto                    c = angles.convert (config (), 15);
        EXPECT_EQ (c.tolerance, 4);
        EXPECT_EQ (c.range_tolerance, 22);
        Rig drift (c);
        drift.home ();
        drift.contact = false;
        drift.raw     = drift.origin - static_cast<int64_t> (sign * 5);
        drift.tick ();
        EXPECT_STREQ (drift.rack.state (), "ready");
        EXPECT_TRUE (drift.tick (true, true).position);

        for (bool upper : {false, true}) {
            Rig boundary (c);
            boundary.home ();
            boundary.contact        = false;
            const int64_t endpoint  = upper ? c.max_relative : c.min_relative;
            const int64_t direction = upper ? 1 : -1;
            boundary.raw            = boundary.origin + endpoint + direction * c.range_tolerance;
            boundary.tick ();
            EXPECT_STREQ (boundary.rack.state (), "ready");
            boundary.raw += direction;
            EXPECT_FALSE (boundary.tick ().position);
            EXPECT_STREQ (boundary.rack.state (), "travel_limit");
        }
    }
}

TEST (RackAngles, WiderRangeToleranceKeepsArrivalAccuracyManualEndpointsAndTargetLimits) {
    for (double sign : {-1.0, 1.0}) {
        inject2026a::RackAngleProfile angles{sign * 8192 / 360, 30, 15, 30, 0.2, 1.0};
        auto                          c = angles.convert (config (), 15);
        c.manual_duty                   = 0.2;
        Rig arrival (c);
        arrival.home ();
        arrival.contact = false;
        arrival.raw     = arrival.origin + c.target + 5;
        arrival.tick (true, true);
        for (int i = 0; i < 100; ++i) arrival.tick ();
        EXPECT_STREQ (arrival.rack.state (), "settling");
        arrival.raw = arrival.origin + c.target + 4;
        for (int i = 0; i < 12; ++i) arrival.tick ();
        EXPECT_STREQ (arrival.rack.state (), "holding");

        Rig manual (c);
        manual.home ();
        manual.contact = false;
        manual.raw     = manual.origin + angles.target_count (30);
        EXPECT_FLOAT_EQ (manual.tick (false, false, true, true, true, false, true).duty, 0);
        manual.tick ();
        manual.raw = manual.origin;
        EXPECT_FLOAT_EQ (manual.tick (false, false, true, true, true, true).duty, 0);

        Rig target (c);
        target.home ();
        target.contact = false;
        EXPECT_FALSE (target.rack.step (target.now, true, true, false, false, true, c.max_relative + 1).position);
        EXPECT_STREQ (target.rack.state (), "target_outside_range");
    }
}

TEST (RackAngles, RejectsInvalidRangeTolerance) {
    for (double value : {-0.1, 31.0, std::numeric_limits<double>::quiet_NaN (), std::numeric_limits<double>::infinity ()}) {
        inject2026a::RackAngleProfile angles{8192 / 360.0, 30, 15, 30, 0.2, value};
        EXPECT_THROW (angles.convert (config (), 15), std::invalid_argument);
    }
    for (int64_t value : {int64_t{-2}, int64_t{1000000001}}) {
        auto c            = config ();
        c.range_tolerance = value;
        EXPECT_THROW (Rack{c}, std::invalid_argument);
    }
}

TEST (RackVelocity, ReachesTwentyAndThirtyDegreesWithinThreeSecondsWithMeasuredInnerLag) {
    for (double sign : {-1.0, 1.0})
        for (double goal : {20.0, 30.0}) {
            inject2026a::RackAngleProfile angles{sign * 8192 / 360, 30, 15, 30, 0.2};
            Rig                           r (angles.convert (config (), goal));
            r.home ();
            r.contact = false;
            inject2026a::RackVelocityProfile outer ({8192, 5.0966919e-5, 0.06, 0.3024, 0.03936, 0.9966671});
            double                           position = 0, velocity = 0, peak = 0, settled_at = 0;
            std::vector<double>              history (40, 0);
            for (int tick = 0; tick < 310; ++tick) {
                const auto command = outer.command (r.rack, r.tick (tick == 0, tick == 0));
                ASSERT_TRUE (command.velocity) << r.rack.state ();
                EXPECT_FALSE (command.position);
                EXPECT_LE (std::abs (command.target_rps), 0.060001);
                for (int step = 0; step < 10; ++step) {
                    history.push_back (command.target_rps);
                    const double delayed = history[history.size () - 40];
                    velocity += (0.9966671 * delayed - velocity) * 0.001 / 0.3024;
                    position += velocity * 8192 * 0.001;
                }
                r.raw = r.origin + static_cast<int64_t> (std::round (position));
                peak  = std::max (peak, sign * position * 360 / 8192);
                if (std::string (r.rack.state ()) == "holding" && settled_at == 0) settled_at = tick * 0.01;
            }
            EXPECT_GT (settled_at, 0);
            EXPECT_LT (settled_at, 3.0);
            EXPECT_NEAR (sign * position * 360 / 8192, goal, 0.2);
            EXPECT_LE (peak, goal + 0.2);
        }
}

TEST (RackVelocity, ZeroDistanceHasNoFeedforwardKickAndStopsDisengageVelocity) {
    inject2026a::RackAngleProfile angles{8192 / 360.0, 30, 15, 30, 0.2};
    Rig                           r (angles.convert (config (), 0));
    r.home ();
    inject2026a::RackVelocityProfile outer ({8192, 5.0966919e-5, 0.06, 0.3024, 0.03936, 0.9966671});
    auto                             command = outer.command (r.rack, r.tick (true, true));
    ASSERT_TRUE (command.velocity);
    EXPECT_FLOAT_EQ (command.target_rps, 0);
    command = outer.command (r.rack, r.tick (false, true, false));
    EXPECT_FALSE (command.velocity);
    EXPECT_FLOAT_EQ (command.duty, 0);
    EXPECT_THROW (inject2026a::RackVelocityProfile ({8192, 0.0001, 0.06, -0.3, 0.04, 1}), std::invalid_argument);
}
