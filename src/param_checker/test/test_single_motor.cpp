#include "vrft_auto_tuner/vrft_auto_tuner.hpp"
#include "param_checker/measurement_guard.hpp"
#include "param_checker/velocity_gain_store.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <unistd.h>

using namespace std::chrono_literals;
TEST(VelocityPi, IdentifiesAFirstOrderPlantWithIrregularFeedback) {
    VRFT tuner;
    constexpr double plant_tau = 0.05, plant_gain = 0.05, desired_tau = 0.3;
    tuner.responseTime(desired_tau);
    double time = 0, previous = 0;
    tuner.update(0, 0, 0);
    // Initial resting samples establish a zero virtual-integral baseline.
    tuner.update(0, 0, 0.01);
    time = 0.01;
    for (int i = 0; i < 300; ++i) {
        const double dt = i % 2 ? 0.012 : 0.018;
        time += dt;
        const double speed = 0.005 * std::sin((time - 0.01) * 8);
        const double duty = (plant_tau * (speed - previous) / dt + speed) / plant_gain;
        tuner.update(duty, speed, time);
        previous = speed;
    }
    const auto fit = tuner.getPI();
    EXPECT_NEAR(fit.kp, plant_tau / (plant_gain * desired_tau), 1e-3);
    EXPECT_NEAR(fit.ki, 1 / (plant_gain * desired_tau), 1e-3);
    EXPECT_GT(fit.r_squared, 0.9999);
    EXPECT_EQ(fit.used_samples, 300u);
}
TEST(VelocityPi, RejectsStillMotorAndReversedEncoderSigns) {
    VRFT still, reversed;
    still.responseTime(0.3); reversed.responseTime(0.3);
    double y = 0;
    for (int i = 0; i < 300; ++i) {
        const double u = 0.1 * std::sin(i * 0.03);
        y += 0.01 / 0.05 * (0.05 * u - y);
        still.update(u, 0, i * 0.01);
        reversed.update(u, -y, i * 0.01);
    }
    EXPECT_EQ(still.getPI().kp, 0);
    EXPECT_FALSE(std::isfinite(still.getPI().r_squared));
    EXPECT_LE(reversed.getPI().r_squared, 0);
}
TEST(MeasurementGuard, LimitsUseAccurate64BitRelativeCounts) {
    using Guard = param_checker::MeasurementGuard;
    Guard::Time now{};
    Guard guard(now, 300ms, 10, -1, true);
    constexpr int64_t origin = 9007199254740993LL;
    guard.motor(now, origin);
    EXPECT_NO_THROW(guard.motor(now + 10ms, origin + 10));
    EXPECT_THROW(guard.motor(now + 20ms, origin + 11), std::runtime_error);
}
TEST(MeasurementGuard, RequiresBothMotorAndConfiguredSwitchReports) {
    using Guard = param_checker::MeasurementGuard;
    Guard::Time now{};
    Guard guard(now, 100ms, 0, 2, false);
    guard.motor(now, 0);
    EXPECT_FALSE(guard.ready());
    guard.switches(now, true); // Active low, so true is clear.
    EXPECT_TRUE(guard.ready());
    guard.motor(now + 90ms, 0);
    EXPECT_THROW(guard.check(now + 100ms), std::runtime_error);
    EXPECT_THROW(guard.switches(now + 100ms, false), std::runtime_error);
    Guard motor_loss(now, 100ms, 0, 2, true);
    motor_loss.switches(now + 90ms, false);
    EXPECT_THROW(motor_loss.check(now + 100ms), std::runtime_error);
}
TEST(VelocityStore, PreservesOtherMotorsAndInvalidatesOnlyTargetPosition) {
    const auto dir = std::filesystem::temp_directory_path() /
        ("param-checker-gain-test-" + std::to_string(getpid()));
    std::filesystem::create_directories(dir);
    const auto path = dir / "gain.yaml";
    { std::ofstream out(path); out << R"(schema_version: 2
metadata: preserve
wheel0:
  velocity: {kp: 0.9, ki: 0.8}
  position: {kp: 0.0001}
wheel2:
  kp: 0.1
  ki: 0.2
  kd: 0.3
  notes: preserve
  position: {kp: 0.0002}
)"; }
    param_checker::save_velocity_gain(path.string(), 2, 0.12, 0.3, 0.3, 0.03, 0.99, 2048, 40, 1);
    auto root = YAML::LoadFile(path.string());
    EXPECT_EQ(root["metadata"].as<std::string>(), "preserve");
    EXPECT_DOUBLE_EQ(root["wheel0"]["velocity"]["kp"].as<double>(), 0.9);
    EXPECT_TRUE(root["wheel0"]["position"]);
    EXPECT_FALSE(root["wheel2"]["position"]);
    EXPECT_FALSE(root["wheel2"]["kp"]);
    EXPECT_EQ(root["wheel2"]["notes"].as<std::string>(), "preserve");
    EXPECT_EQ(root["wheel2"]["velocity"]["controller"].as<std::string>(), "PI");
    EXPECT_DOUBLE_EQ(root["wheel2"]["velocity"]["kd"].as<double>(), 0);
    EXPECT_DOUBLE_EQ(root["wheel2"]["encoder"]["counts_per_revolution"].as<double>(), 327680);
    // A missing file also produces a mapping containing the selected motor only.
    const auto fresh = dir / "new.yaml";
    EXPECT_NO_THROW(param_checker::save_velocity_gain(fresh.string(), 2, 0.12, 0.3, 0.3, 0.03, 0.99, 2048, 40, 1));
    EXPECT_EQ(YAML::LoadFile(fresh.string()).size(), 1u);
    std::filesystem::remove_all(dir);
}
