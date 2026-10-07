#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace param_checker {
// STM's outer loop maps raw encoder count error to rps with a P controller.
// For M(s)=1/(tau*s+1), the VRFT virtual error is tau*dy/dt.
class PositionVrft {
   public:
    struct Sample {
        double  time;
        int64_t raw_count;
        double  relative_count;
        double  input_rps;
        double  virtual_error;
        bool    used;
    };
    struct Result {
        double kp{0.0};
        double r_squared{-std::numeric_limits<double>::infinity ()};
        size_t used_samples{0};
    };

    explicit PositionVrft (double tau) : tau_ (tau) {}

    void update (double input, int64_t count, double time) {
        if (!std::isfinite (input) || !std::isfinite (time)) {
            return;
        }
        if (!samples_.empty () && time <= samples_.back ().time) {
            return;
        }
        // Subtract before converting: counts above 2^53 must retain small steps.
        const long double relative = static_cast<long double> (count) - (samples_.empty () ? static_cast<long double> (count) : origin_);
        if (samples_.empty ()) {
            origin_ = static_cast<long double> (count);
        }
        double     error = 0.0;
        const bool used  = !samples_.empty ();
        if (used) {
            const long double delta = static_cast<long double> (count) - samples_.back ().raw_count;
            error                   = tau_ * static_cast<double> (delta) / (time - samples_.back ().time);
        }
        samples_.push_back ({time, count, static_cast<double> (relative), input, error, used});
    }

    Result result () const {
        Result result;
        double ee = 0.0, eu = 0.0, sum = 0.0;
        for (const auto &s : samples_) {
            if (!s.used) {
                continue;
            }
            ee += s.virtual_error * s.virtual_error;
            eu += s.virtual_error * s.input_rps;
            sum += s.input_rps;
            ++result.used_samples;
        }
        if (ee <= 0.0 || result.used_samples < 3) {
            return result;
        }
        result.kp             = std::max (0.0, eu / ee);
        const double mean     = sum / result.used_samples;
        double       residual = 0.0, variance = 0.0;
        for (const auto &s : samples_) {
            if (!s.used) {
                continue;
            }
            residual += std::pow (s.input_rps - result.kp * s.virtual_error, 2);
            variance += std::pow (s.input_rps - mean, 2);
        }
        if (variance > std::numeric_limits<double>::epsilon ()) {
            result.r_squared = 1.0 - residual / variance;
        }
        return result;
    }

    const std::vector<Sample> &samples () const {
        return samples_;
    }

   private:
    double              tau_;
    long double         origin_{0};
    std::vector<Sample> samples_;
};
}  // namespace param_checker
