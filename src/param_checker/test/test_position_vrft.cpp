#include "param_checker/position_vrft.hpp"
#include <gtest/gtest.h>

TEST(PositionVrft, IdentifiesPositivePGainWithIrregularSamplingAndHugeCounts) {
    param_checker::PositionVrft tuner(2.0);
    int64_t count = 9007199254740993LL;
    double time = 0.0;
    tuner.update(0.0, count, time);
    const double kp = 0.00002;
    for (int i = 1; i <= 100; ++i) {
    const double dt = i % 2 == 0 ? 0.01 : 0.02;
    const int delta = i % 3 == 0 ? -11 : 7;
    count += delta;
    time += dt;
    tuner.update(kp * 2.0 * delta / dt, count, time);
    }
    const auto result = tuner.result();
    EXPECT_NEAR(result.kp, kp, 1e-14);
    EXPECT_NEAR(result.r_squared, 1.0, 1e-12);
    EXPECT_EQ(result.used_samples, 100u);
    EXPECT_DOUBLE_EQ(tuner.samples()[1].relative_count, 7.0);
}

TEST(PositionVrft, RejectsNoMotionConstantInputAndReversedSigns) {
    param_checker::PositionVrft still(2.0), constant(2.0), reversed(2.0);
    for (int i = 0; i < 50; ++i) {
    still.update(i % 2 ? 0.01 : -0.01, 42, i * 0.01);
    constant.update(0.01, i, i * 0.01);
    reversed.update(i % 2 ? -0.01 : 0.01, i % 2 ? 1 : 0, i * 0.01);
    }
    EXPECT_EQ(still.result().kp, 0.0);
    EXPECT_FALSE(std::isfinite(still.result().r_squared));
    EXPECT_FALSE(std::isfinite(constant.result().r_squared));
    EXPECT_EQ(reversed.result().kp, 0.0);
    EXPECT_LT(reversed.result().r_squared, 0.7);
}

TEST(PositionVrft, IgnoresNonFiniteAndRepeatedTimeSamples) {
    param_checker::PositionVrft tuner(2.0);
    tuner.update(0.0, 0, 0.0);
    tuner.update(0.1, 1, 0.0);
    tuner.update(std::numeric_limits<double>::quiet_NaN(), 1, 0.1);
    tuner.update(0.1, 1, 0.1);
    EXPECT_EQ(tuner.samples().size(), 2u);
    EXPECT_EQ(tuner.result().kp, 0.0);
}
