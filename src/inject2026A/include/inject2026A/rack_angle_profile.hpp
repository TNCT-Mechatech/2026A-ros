#pragma once

#include "rack_count_controller.hpp"

namespace inject2026a {

// Convert rack degrees to signed STM counts using the loaded encoder calibration.
struct RackAngleProfile {
    double counts_per_degree;
    double max_angle_deg;
    double speed_deg_s;
    double acceleration_deg_s2;
    double tolerance_deg;
    double range_tolerance_deg = -1.0;  // -1 uses tolerance_deg for existing callers.

    void validate () const {
        if (!std::isfinite (counts_per_degree) || counts_per_degree == 0 || !std::isfinite (max_angle_deg) || max_angle_deg <= 0 || !std::isfinite (speed_deg_s) || speed_deg_s <= 0 || !std::isfinite (acceleration_deg_s2) ||
            acceleration_deg_s2 <= 0 || !std::isfinite (tolerance_deg) || tolerance_deg < 0 || tolerance_deg > max_angle_deg || !std::isfinite (range_tolerance_deg) ||
            (range_tolerance_deg != -1.0 && (range_tolerance_deg < 0 || range_tolerance_deg > max_angle_deg)) || std::abs (static_cast<long double> (counts_per_degree) * max_angle_deg) > 1e9L ||
            !std::isfinite (std::abs (counts_per_degree) * speed_deg_s) || !std::isfinite (std::abs (counts_per_degree) * acceleration_deg_s2))
            throw std::invalid_argument ("invalid rack angle profile calibration, limits or dynamics");
    }

    int64_t target_count (double angle_deg) const {
        validate ();
        if (!std::isfinite (angle_deg) || angle_deg < 0 || angle_deg > max_angle_deg) throw std::invalid_argument ("rack target angle must be within 0..rack_max_angle_deg");
        return static_cast<int64_t> (std::round (angle_deg * counts_per_degree));
    }

    RackCountController::Config convert (RackCountController::Config config, double target_deg) const {
        config.target                = target_count (target_deg);
        config.speed                 = std::abs (counts_per_degree) * speed_deg_s;
        config.acceleration          = std::abs (counts_per_degree) * acceleration_deg_s2;
        config.tolerance             = static_cast<int64_t> (std::floor (std::abs (counts_per_degree) * tolerance_deg));
        const double range_tolerance = range_tolerance_deg >= 0 ? range_tolerance_deg : tolerance_deg;
        config.range_tolerance       = static_cast<int64_t> (std::floor (std::abs (counts_per_degree) * range_tolerance));
        config.enforce_range         = true;
        config.manual_count_sign     = counts_per_degree > 0 ? 1 : -1;
        const auto endpoint          = target_count (max_angle_deg);
        config.min_relative          = std::min (int64_t{0}, endpoint);
        config.max_relative          = std::max (int64_t{0}, endpoint);
        return config;
    }
};
}  // namespace inject2026a
