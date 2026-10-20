#pragma once

#include "rack_count_controller.hpp"

namespace inject2026a {
// The STM retains the tuned velocity PI. The ROS outer loop combines inverse
// velocity dynamics with position correction, aligned to the measured delay.
class RackVelocityProfile {
   public:
    struct Config {
        double counts_per_revolution, position_kp, max_rps;
        double response_s, delay_s, steady_gain;
    };
    explicit RackVelocityProfile (Config config) : config_ (config) {
        if (!std::isfinite (config.counts_per_revolution) || config.counts_per_revolution <= 0 || !std::isfinite (config.position_kp) || config.position_kp <= 0 || !std::isfinite (config.max_rps) || config.max_rps <= 0 ||
            !std::isfinite (config.response_s) || config.response_s < 0 || config.response_s > 2 || !std::isfinite (config.delay_s) || config.delay_s < 0 || config.delay_s > 0.3 || !std::isfinite (config.steady_gain) || config.steady_gain < 0.5 ||
            config.steady_gain > 1.5)
            throw std::invalid_argument ("invalid rack velocity response model");
    }
    RackController::Command command (const RackCountController &profile, RackController::Command command) const {
        // Origin, neutral, watchdog and range faults all disengage the outer loop.
        if (!command.position) return command;
        const auto   reference  = profile.current_reference ();
        const double correction = config_.position_kp * (profile.delayed_reference_position (config_.delay_s) - static_cast<double> (profile.relative_count ()));
        const double inverse    = (reference.velocity + config_.response_s * reference.acceleration) / (config_.counts_per_revolution * config_.steady_gain);
        command.position        = false;
        command.velocity        = true;
        command.target_rps      = static_cast<float> (std::clamp (inverse + correction, -config_.max_rps, config_.max_rps));
        return command;
    }

   private:
    Config config_;
};
}  // namespace inject2026a
