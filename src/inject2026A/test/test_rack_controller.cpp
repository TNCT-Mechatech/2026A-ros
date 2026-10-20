#include "rack_controller.hpp"
#include "rack_gains.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

using Rack = inject2026a::RackController;
using namespace std::chrono_literals;
namespace {
Rack::Config config () {
    Rack::Config c;
    c.counts_per_degree = 100;
    c.max_angle_deg     = 90;
    c.speed_deg_s       = 20;
    c.gains_ready       = true;
    return c;
}
struct Rig {
    Rack          rack{config ()};
    Rack::Time    now{};
    int64_t       raw     = (int64_t{1} << 40);
    bool          contact = false;
    Rack::Command tick (bool up = false, bool down = false, bool allowed = true, bool motor = true, bool switches = true) {
        now += 10ms;
        if (motor) rack.motor_feedback (now, raw);
        if (switches) rack.switch_feedback (now, contact);
        return rack.step (now, up, down, allowed);
    }
    void home () {
        tick ();
        contact = true;
        for (int i = 0; i < 8; ++i) tick ();
        contact = false;
        tick ();
    }
};
}  // namespace
TEST (Rack, CalibrationAndGainsGateAllMotion) {
    auto c              = config ();
    c.counts_per_degree = 0;
    Rack rack (c);
    EXPECT_FALSE (rack.configured ());
    EXPECT_FLOAT_EQ (rack.step ({}, false, true, true).duty, 0);
    c             = config ();
    c.gains_ready = false;
    EXPECT_FALSE (Rack (c).configured ());
    c               = config ();
    c.max_angle_deg = 0;
    EXPECT_FALSE (Rack (c).configured ());
    c             = config ();
    c.homing_duty = 2;
    EXPECT_THROW (Rack{c}, std::invalid_argument);
}
TEST (Rack, RequiresNeutralThenHomesOnlyWhileTriangleIsHeld) {
    Rig r;
    EXPECT_FLOAT_EQ (r.tick (false, true).duty, 0);
    r.tick ();
    EXPECT_LT (r.tick (false, true).duty, 0);
    EXPECT_STREQ (r.rack.state (), "homing");
    EXPECT_FLOAT_EQ (r.tick ().duty, 0);
    EXPECT_FLOAT_EQ (r.tick (true, true).duty, 0);
    EXPECT_FLOAT_EQ (r.tick (true).duty, 0);
}
TEST (Rack, ContactStopsImmediatelyAndDebouncesBeforeZeroing) {
    Rig r;
    r.tick ();
    r.tick (false, true);
    r.contact = true;
    EXPECT_FLOAT_EQ (r.tick (false, true).duty, 0);
    EXPECT_FALSE (r.rack.homed ());
    r.contact = false;
    r.tick (false, true);
    r.contact = true;
    for (int i = 0; i < 5; ++i) r.tick (false, true);
    EXPECT_FALSE (r.rack.homed ());
    r.tick (false, true);
    EXPECT_TRUE (r.rack.homed ());
    EXPECT_FALSE (r.tick (false, true).position);  // Must release after homing.
    EXPECT_DOUBLE_EQ (r.rack.measured_angle (), 0);
    r.tick ();
    EXPECT_EQ (r.tick ().target_count, r.raw);
}
TEST (Rack, OneSwitchReportCannotCompleteDebounce) {
    Rig r;
    r.tick ();
    r.contact = true;
    r.tick ();
    for (int i = 0; i < 10; ++i) r.tick (false, false, true, true, false);
    EXPECT_FALSE (r.rack.homed ());
    r.tick ();
    EXPECT_TRUE (r.rack.homed ());
}
TEST (Rack, AngleIntegrationHoldingBoundsAnd64BitOrigin) {
    Rig r;
    r.home ();
    const auto first = r.tick (true);
    EXPECT_TRUE (first.position);
    EXPECT_EQ (first.target_count, r.raw + 20);
    const auto held = r.tick ();
    EXPECT_EQ (held.target_count, first.target_count);
    EXPECT_EQ (r.tick (true, true).target_count, first.target_count);
    for (int i = 0; i < 500; ++i) r.tick (true);
    EXPECT_EQ (r.tick ().target_count, r.raw + 9000);
    for (int i = 0; i < 500; ++i) r.tick (false, true);
    EXPECT_EQ (r.tick ().target_count, r.raw);
}
TEST (Rack, MissingMotorOrSwitchInvalidatesOriginAndRequiresNeutral) {
    Rig r;
    r.home ();
    r.tick (true);
    for (int i = 0; i < 22; ++i) r.tick (true, false, true, false, true);
    EXPECT_FALSE (r.rack.homed ());
    EXPECT_FALSE (r.tick (true).position);
    EXPECT_FLOAT_EQ (r.tick (false, true).duty, 0);  // Held controls cannot resume.
    r.tick ();
    EXPECT_LT (r.tick (false, true).duty, 0);
    r.home ();
    for (int i = 0; i < 22; ++i) r.tick (true, false, true, true, false);
    EXPECT_FALSE (r.rack.homed ());
}
TEST (Rack, JoyTimeoutResumesFromMeasuredPositionAndReconnectLosesOrigin) {
    Rig r;
    r.home ();
    for (int i = 0; i < 20; ++i) r.tick (true);
    EXPECT_FALSE (r.tick (true, false, false).position);
    EXPECT_FALSE (r.tick (true).position);
    r.raw += 100;  // The rack actually reached 1 degree, not the previous 4 degree target.
    r.tick ();
    EXPECT_EQ (r.tick ().target_count, r.raw);
    r.rack.reset ();
    EXPECT_FALSE (r.tick (true).position);
    EXPECT_FALSE (r.rack.homed ());
}
TEST (Rack, HomingTimeoutCannotBeBypassedByReleasingButtons) {
    Rig r;
    r.tick ();
    r.tick (false, true);
    for (int i = 0; i < 1001; ++i) r.tick ();
    EXPECT_TRUE (r.rack.fault ());
    EXPECT_FLOAT_EQ (r.tick (false, true).duty, 0);
    EXPECT_TRUE (r.rack.fault ());
    r.home ();  // A confirmed physical origin can clear the fault.
    EXPECT_FALSE (r.rack.fault ());
}
TEST (Rack, SignedCountScaleAndLongTickDoNotJumpTarget) {
    auto c              = config ();
    c.counts_per_degree = -100;
    Rack       rack (c);
    Rack::Time now{};
    rack.motor_feedback (now, 1000);
    rack.switch_feedback (now, true);
    rack.step (now, false, false, true);
    now += 50ms;
    rack.motor_feedback (now, 1000);
    rack.switch_feedback (now, true);
    rack.step (now, false, false, true);
    now += 1s;
    rack.motor_feedback (now, 1000);
    rack.switch_feedback (now, false);
    auto command = rack.step (now, true, false, true);
    EXPECT_EQ (command.target_count, 900);  // At most 50ms = 1 degree.
}
TEST (RackGains, ReadsLocalFlatAndUnifiedVelocitySchemas) {
    auto              path     = std::filesystem::temp_directory_path () / "inject_rack_gains_test.yaml";
    const std::string position = R"(
  position:
    controller: P
    kp: 0.0001
    ki: 0.0
    kd: 0.0
    input_unit: encoder_count
    output_unit: rps
    max_rps: 0.02
    encoder_ppr: 2048
    gear_rate: 40.0
    encoder_polarity: -1
)";
    {
        std::ofstream out (path);
        out << "wheel2:\n  kp: 0.1\n  ki: 0.2\n  kd: -0.01\n" << position;
    }
    const auto local = inject2026a::load_rack_gains (path.string ());
    EXPECT_FLOAT_EQ (local.vel_kp, 0.1f);
    EXPECT_FLOAT_EQ (local.max_rps, 0.02f);
    EXPECT_EQ (local.polarity, -1);
    {
        std::ofstream out (path);
        out << R"(schema_version: 2
wheel2:
  velocity:
    controller: PI
    kp: 0.3
    ki: 0.4
    input_unit: rps
    output_unit: duty
    max_rps: 0.01
)" << position;
    }
    EXPECT_FLOAT_EQ (inject2026a::load_rack_gains (path.string ()).max_rps, 0.01f);
    {
        std::ofstream out (path);
        out << "wheel2: {kp: 0.1, ki: 0.2}\n";
    }
    EXPECT_THROW (inject2026a::load_rack_gains (path.string ()), std::exception);
    std::filesystem::remove (path);
}
