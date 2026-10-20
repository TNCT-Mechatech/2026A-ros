#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace param_checker {
// Fit G*exp(-delay*s)/(response*s+1) to the inner velocity response.
// Commands are integrated over non-overlapping count windows; integer counts
// are never differentiated twice. Keep the historical API name for callers.
class PositionVrft {
   public:
    struct Config {
        double position_time         = 2.0;
        double counts_per_revolution = 8192.0;
        double filter_window         = 0.1;
    };
    struct Sample {
        double  time;
        int64_t raw_count;
        double  relative_count, input_rps, virtual_error;
        bool    used;  // Original VRFT diagnostic, not the response-model regressor.
    };
    struct ModelSample {
        size_t source_index;
        double time, input_rps, measured_rps, predicted_rps;
        bool   training;
    };
    struct Result {
        double      kp                   = 0.0;
        double      r_squared            = -std::numeric_limits<double>::infinity ();
        double      training_r_squared   = -std::numeric_limits<double>::infinity ();
        double      validation_r_squared = -std::numeric_limits<double>::infinity ();
        double      raw_r_squared        = -std::numeric_limits<double>::infinity ();
        double      velocity_gain = 0.0, velocity_response_s = 0.0, delay_s = 0.0;
        size_t      used_samples = 0, training_samples = 0, validation_samples = 0;
        bool        model_valid = false;
        const char *reason      = "insufficient_samples";
    };

    explicit PositionVrft (double tau) : PositionVrft (Config{tau, 8192.0, 0.1}) {}
    explicit PositionVrft (Config config) : config_ (config) {
        if (!std::isfinite (config.position_time) || config.position_time <= 0 || !std::isfinite (config.counts_per_revolution) || config.counts_per_revolution <= 0 || !std::isfinite (config.filter_window) || config.filter_window < 0.02 ||
            config.filter_window > 0.5)
            throw std::invalid_argument ("invalid position response model configuration");
    }

    // input is the command held during the interval ending at this sample.
    void update (double input, int64_t count, double time) {
        if (!std::isfinite (input) || !std::isfinite (time)) return;
        if (!samples_.empty () && time <= samples_.back ().time) return;
        if (samples_.empty ()) origin_ = count;
        const long double relative = static_cast<long double> (count) - origin_;
        double            error    = 0;
        const bool        used     = !samples_.empty ();
        if (used) {
            const long double delta = static_cast<long double> (count) - samples_.back ().raw_count;
            error                   = config_.position_time * static_cast<double> (delta) / (time - samples_.back ().time);
        }
        samples_.push_back ({time, count, static_cast<double> (relative), input, error, used});
        cached_ = false;
    }

    Result result () const {
        if (cached_) return result_;
        cached_ = true;
        result_ = {};
        model_samples_.clear ();
        if (samples_.size () < 3) return result_;
        result_.raw_r_squared = raw_fit ();
        std::vector<Window> windows;
        size_t              left           = 0;
        double              input_integral = 0;
        for (size_t right = 1; right < samples_.size (); ++right) {
            input_integral += samples_[right].input_rps * (samples_[right].time - samples_[right - 1].time);
            const double duration = samples_[right].time - samples_[left].time;
            if (duration + 1e-9 < config_.filter_window) continue;
            const long double delta = static_cast<long double> (samples_[right].raw_count) - samples_[left].raw_count;
            windows.push_back ({left, right, duration, static_cast<double> (delta) / (config_.counts_per_revolution * duration), input_integral / duration});
            left           = right;
            input_integral = 0;
        }
        result_.used_samples       = windows.size ();
        const size_t split         = windows.size () * 3 / 5;
        result_.training_samples   = split;
        result_.validation_samples = windows.size () - split;
        if (split < 10 || windows.size () - split < 6) return result_;
        if (!excited (windows, 0, split) || !excited (windows, split, windows.size ())) {
            result_.reason = "need_positive_and_negative_input_in_both_parts";
            return result_;
        }

        // Search on the first 60% only. The remaining 40% must pass independently.
        Candidate best;
        for (double response : {0.0, 0.01, 0.02, 0.03, 0.05, 0.075, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.7, 1.0, 1.4, 2.0})
            for (int delay_index = 0; delay_index <= 15; ++delay_index) consider (windows, split, response, delay_index * 0.02, best);
        for (double scale : {0.25, 0.08, 0.025, 0.008}) {
            const double response_step = std::max (best.response, 0.02) * scale;
            const double delay_step    = 0.04 * scale;
            const auto   center        = best;
            for (int r = -2; r <= 2; ++r)
                for (int d = -2; d <= 2; ++d) consider (windows, split, std::clamp (center.response + r * response_step, 0.0, 2.0), std::clamp (center.delay + d * delay_step, 0.0, 0.3), best);
        }
        if (!std::isfinite (best.cost)) return result_;
        const auto          curve = integrals (best.response, best.delay);
        std::vector<double> measured, predicted;
        for (size_t k = 0; k < windows.size (); ++k) {
            const auto  &w          = windows[k];
            const double prediction = best.gain * (curve[w.right] - curve[w.left]) / w.duration + best.initial_velocity * initial_basis (best.response, w);
            measured.push_back (w.velocity);
            predicted.push_back (prediction);
            model_samples_.push_back ({w.right, samples_[w.right].time, w.input, w.velocity, prediction, k < split});
        }
        result_.velocity_gain        = best.gain;
        result_.velocity_response_s  = best.response;
        result_.delay_s              = best.delay;
        result_.training_r_squared   = score (measured, predicted, 0, split);
        result_.validation_r_squared = score (measured, predicted, split, windows.size ());
        result_.r_squared            = std::min (result_.training_r_squared, result_.validation_r_squared);
        if (best.gain < 0.5 || best.gain > 1.5) {
            result_.reason = "velocity_gain_or_encoder_scale_mismatch";
            return result_;
        }
        if (best.response > 1.98 || best.delay > 0.298) {
            result_.reason = "velocity_response_outside_model_range";
            return result_;
        }
        if (config_.position_time < 3 * (best.response + best.delay)) {
            result_.reason = "position_response_too_fast_for_measured_velocity_loop";
            return result_;
        }
        // P outer-loop characteristic equation:
        // response*s^2+s+CPR*G*Kp*exp(-delay*s)=0.
        // Place the slow real pole at -1/position_time, with separation checked above.
        result_.kp          = (config_.position_time - best.response) / (config_.position_time * config_.position_time * config_.counts_per_revolution * best.gain) * std::exp (-best.delay / config_.position_time);
        result_.model_valid = std::isfinite (result_.kp) && result_.kp > 0;
        result_.reason      = result_.model_valid ? "model_fitted" : "invalid_position_gain";
        return result_;
    }

    const std::vector<Sample> &samples () const {
        return samples_;
    }
    const std::vector<ModelSample> &model_samples () const {
        (void)result ();
        return model_samples_;
    }

   private:
    struct Window {
        size_t left, right;
        double duration, velocity, input;
    };
    struct Candidate {
        double response = 0, delay = 0, gain = 0, initial_velocity = 0;
        double cost = std::numeric_limits<double>::infinity ();
    };
    static bool excited (const std::vector<Window> &windows, size_t begin, size_t end) {
        int positive = 0, negative = 0;
        for (size_t k = begin; k < end; ++k) {
            positive += windows[k].input > 1e-6;
            negative += windows[k].input < -1e-6;
        }
        return positive >= 2 && negative >= 2;
    }

    // Exact integrals, splitting at delayed changes in a zero-order-held command.
    std::vector<double> integrals (double response, double delay) const {
        std::vector<double> curve (samples_.size ());
        struct Event {
            double time, input;
        };
        std::vector<Event> events;
        double             previous = samples_[0].input_rps;
        for (size_t k = 1; k < samples_.size (); ++k) {
            if (samples_[k].input_rps != previous) {
                events.push_back ({samples_[k - 1].time + delay, samples_[k].input_rps});
                previous = samples_[k].input_rps;
            }
        }
        double     time = samples_[0].time, input = samples_[0].input_rps, state = 0, integral = 0;
        const auto advance = [&] (double target) {
            const double dt = target - time;
            if (response > 0) {
                const double change = -std::expm1 (-dt / response);
                integral += input * dt + (state - input) * response * change;
                state += (input - state) * change;
            } else {
                integral += input * dt;
            }
            time = target;
        };
        size_t event = 0;
        for (size_t k = 0; k < samples_.size (); ++k) {
            while (event < events.size () && events[event].time <= samples_[k].time) {
                advance (events[event].time);
                input = events[event++].input;
            }
            advance (samples_[k].time);
            curve[k] = integral;
        }
        return curve;
    }

    double initial_basis (double response, const Window &w) const {
        if (response == 0) return 0;
        return response * (std::exp (-(samples_[w.left].time - samples_[0].time) / response) - std::exp (-(samples_[w.right].time - samples_[0].time) / response)) / w.duration;
    }

    void consider (const std::vector<Window> &windows, size_t split, double response, double delay, Candidate &best) const {
        const auto curve = integrals (response, delay);
        double     xx = 0, xz = 0, zz = 0, xy = 0, zy = 0;
        for (size_t k = 0; k < split; ++k) {
            const auto  &w = windows[k];
            const double x = (curve[w.right] - curve[w.left]) / w.duration;
            const double z = initial_basis (response, w);
            xx += x * x;
            xz += x * z;
            zz += z * z;
            xy += x * w.velocity;
            zy += z * w.velocity;
        }
        if (xx <= 1e-20) return;
        const double determinant = xx * zz - xz * xz;
        const double g           = determinant > xx * zz * 1e-10 && zz > 1e-20 ? (xy * zz - zy * xz) / determinant : xy / xx;
        const double initial     = zz > 1e-20 ? (zy - g * xz) / zz : 0;
        double       cost        = 0;
        for (size_t k = 0; k < split; ++k) {
            const auto  &w          = windows[k];
            const double prediction = g * (curve[w.right] - curve[w.left]) / w.duration + initial * initial_basis (response, w);
            cost += std::pow (w.velocity - prediction, 2);
        }
        if (cost < best.cost) best = {response, delay, g, initial, cost};
    }

    static double score (const std::vector<double> &measured, const std::vector<double> &predicted, size_t begin, size_t end) {
        double mean = 0;
        for (size_t k = begin; k < end; ++k) mean += measured[k];
        mean /= static_cast<double> (end - begin);
        double variance = 0, residual = 0;
        for (size_t k = begin; k < end; ++k) {
            variance += std::pow (measured[k] - mean, 2);
            residual += std::pow (measured[k] - predicted[k], 2);
        }
        return variance > 1e-20 ? 1 - residual / variance : -std::numeric_limits<double>::infinity ();
    }

    double raw_fit () const {
        double ee = 0, eu = 0, sum = 0;
        size_t n = 0;
        for (const auto &s : samples_)
            if (s.used) {
                ee += s.virtual_error * s.virtual_error;
                eu += s.virtual_error * s.input_rps;
                sum += s.input_rps;
                ++n;
            }
        if (ee <= 0 || n < 3) return -std::numeric_limits<double>::infinity ();
        const double kp = std::max (0.0, eu / ee), mean = sum / static_cast<double> (n);
        double       variance = 0, residual = 0;
        for (const auto &s : samples_)
            if (s.used) {
                variance += std::pow (s.input_rps - mean, 2);
                residual += std::pow (s.input_rps - kp * s.virtual_error, 2);
            }
        return variance > 1e-20 ? 1 - residual / variance : -std::numeric_limits<double>::infinity ();
    }

    Config                           config_;
    long double                      origin_ = 0;
    std::vector<Sample>              samples_;
    mutable bool                     cached_ = false;
    mutable Result                   result_;
    mutable std::vector<ModelSample> model_samples_;
};
}  // namespace param_checker
