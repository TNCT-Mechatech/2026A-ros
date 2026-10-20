#include "param_checker/position_vrft.hpp"

#include <gtest/gtest.h>

#include <functional>

using Tuner = param_checker::PositionVrft;
namespace {
// Independent, 1ms Euler simulation of a velocity loop with delayed commands.
// Test input ends at each reported sample, as in the serial node's update API.
struct Plant {
    double                         response = 0.3, delay = 0.04, gain = 0.95, cpr = 8192;
    bool                           stalled = false, changed = false;
    std::function<double (double)> input = [] (double time) { return 0.04 * (std::sin (3.1 * time) + 0.3 * std::sin (8.3 * time)); };
    void                           collect (Tuner &tuner, double duration = 12) const {
        const int64_t                          origin = (int64_t{1} << 60) + 17;
        std::vector<std::pair<double, double>> events;
        double                                 time = 0, position = 0, velocity = 0, command = 0;
        size_t                                 event = 0;
        tuner.update (0, origin, time);
        for (int index = 1; time < duration; ++index) {
            const double dt     = index % 7 == 0 ? 0.02 : (index % 3 == 0 ? 0.009 : 0.011);
            const double target = std::min (duration, time + dt);
            const double sent   = input (time);
            events.push_back ({time + delay, sent});
            while (time < target - 1e-12) {
                const double step = std::min (0.001, target - time);
                while (event < events.size () && events[event].first <= time + step / 2) command = events[event++].second;
                const double actual_gain = changed && time >= duration * 0.61 ? gain * 0.2 : gain;
                if (response > 0)
                    velocity += (actual_gain * command - velocity) * step / response;
                else
                    velocity = actual_gain * command;
                if (!stalled) position += velocity * cpr * step;
                time += step;
            }
            time = target;
            tuner.update (sent, origin + static_cast<int64_t> (std::llround (position)), time);
        }
    }
};
}  // namespace

TEST (PositionVrft, FitsDelayedQuantizedPlantWithIrregularSamplingAndHugeCounts) {
    Plant plant;
    Tuner tuner ({2, plant.cpr, 0.1});
    plant.collect (tuner);
    const auto r = tuner.result ();
    ASSERT_TRUE (r.model_valid) << r.reason;
    EXPECT_GT (r.training_r_squared, 0.98);
    EXPECT_GT (r.validation_r_squared, 0.98);
    EXPECT_LT (r.raw_r_squared, 0.7);  // The old instantaneous fit fails this normal plant.
    EXPECT_NEAR (r.velocity_response_s, plant.response, 0.02);
    EXPECT_NEAR (r.delay_s, plant.delay, 0.012);
    EXPECT_NEAR (r.velocity_gain, plant.gain, 0.02);
    const double expected = (2 - plant.response) / (4 * plant.cpr * plant.gain) * std::exp (-plant.delay / 2);
    EXPECT_NEAR (r.kp, expected, expected * 0.03);
    EXPECT_GT (r.validation_samples, 20u);
    EXPECT_EQ (r.used_samples, tuner.model_samples ().size ());
    EXPECT_DOUBLE_EQ (tuner.samples ()[0].relative_count, 0);
}

TEST (PositionVrft, EncoderScaleAndGearChangeTheGainInversely) {
    for (double gear : {1.0, 40.0}) {
        Plant plant;
        plant.cpr      = 8192 * gear;
        plant.response = 0;
        plant.delay    = 0;
        plant.gain     = 1;
        Tuner tuner ({2, plant.cpr, 0.1});
        plant.collect (tuner);
        const auto r = tuner.result ();
        ASSERT_TRUE (r.model_valid) << r.reason;
        EXPECT_GT (r.r_squared, 0.98);
        EXPECT_NEAR (r.kp, 1 / (2 * plant.cpr), 0.015 / (2 * plant.cpr));
    }
}

TEST (PositionVrft, RejectsStillMotorAndEncoderScaleMismatch) {
    Plant plant;
    plant.stalled = true;
    Tuner still ({2, 8192, 0.1});
    plant.collect (still);
    EXPECT_FALSE (still.result ().model_valid);
    EXPECT_EQ (still.result ().kp, 0);
    plant.stalled = false;
    Tuner wrong_scale ({2, 3 * 8192, 0.1});
    plant.collect (wrong_scale);
    EXPECT_FALSE (wrong_scale.result ().model_valid);
    EXPECT_STREQ (wrong_scale.result ().reason, "velocity_gain_or_encoder_scale_mismatch");
}

TEST (PositionVrft, RequiresExcitationInTrainingAndValidation) {
    Plant plant;
    plant.input = [] (double) { return 0.03; };
    Tuner constant (2);
    plant.collect (constant);
    EXPECT_FALSE (constant.result ().model_valid);
    plant.input = [] (double t) { return t >= 7.1 ? 0.03 : 0.04 * std::sin (3 * t); };
    Tuner missing_validation (2);
    plant.collect (missing_validation);
    EXPECT_FALSE (missing_validation.result ().model_valid);
    EXPECT_STREQ (missing_validation.result ().reason, "need_positive_and_negative_input_in_both_parts");
}

TEST (PositionVrft, HeldOutPlantChangeCannotPassSaveThreshold) {
    Plant plant;
    plant.changed = true;
    Tuner tuner (2);
    plant.collect (tuner);
    const auto r = tuner.result ();
    EXPECT_GT (r.training_r_squared, 0.95);
    EXPECT_LT (r.validation_r_squared, 0.7);
    EXPECT_LT (r.r_squared, 0.7);
}

TEST (PositionVrft, RejectsOuterResponseTooFastForMeasuredInnerLoop) {
    Plant plant;
    Tuner tuner ({0.5, 8192, 0.1});
    plant.collect (tuner);
    const auto r = tuner.result ();
    EXPECT_GT (r.r_squared, 0.95);
    EXPECT_FALSE (r.model_valid);
    EXPECT_EQ (r.kp, 0);
    EXPECT_STREQ (r.reason, "position_response_too_fast_for_measured_velocity_loop");
}

TEST (PositionVrft, IgnoresInvalidSamplesAndInvalidatesCache) {
    Tuner tuner (2);
    tuner.update (0, 9007199254740993LL, 0);
    tuner.update (0.1, 9007199254741000LL, 0);
    tuner.update (std::numeric_limits<double>::quiet_NaN (), 1, 0.1);
    EXPECT_EQ (tuner.samples ().size (), 1u);
    EXPECT_FALSE (tuner.result ().model_valid);
    Plant plant;
    Tuner full (2);
    plant.collect (full);
    const auto before = full.result ();
    full.update (0, full.samples ().back ().raw_count, full.samples ().back ().time + 0.2);
    EXPECT_GT (full.result ().used_samples, before.used_samples);
    EXPECT_EQ (full.result ().used_samples, full.model_samples ().size ());
    EXPECT_THROW (Tuner (Tuner::Config{2, 0, 0.1}), std::invalid_argument);
    EXPECT_THROW (Tuner (Tuner::Config{2, 8192, 0.001}), std::invalid_argument);
}

TEST (PositionVrft, EstimatedPGainHasStableClosedLoopResponseWithInnerDelay) {
    Plant plant;
    Tuner tuner (2);
    plant.collect (tuner);
    const auto r = tuner.result ();
    ASSERT_TRUE (r.model_valid);
    double              position = 0, velocity = 0, peak = 0;
    std::vector<double> commands;
    constexpr double    step = 0.001, target = 30;
    const size_t        delayed_steps = static_cast<size_t> (plant.delay / step);
    for (int k = 0; k < 10000; ++k) {
        commands.push_back (r.kp * (target - position));
        const double delayed = commands.size () > delayed_steps ? commands[commands.size () - delayed_steps - 1] : 0;
        velocity += (plant.gain * delayed - velocity) * step / plant.response;
        position += velocity * plant.cpr * step;
        peak = std::max (peak, position);
    }
    EXPECT_LE (peak, target * 1.02);
    EXPECT_NEAR (position, target, target * 0.02);
}
