#pragma once

#include <algorithm>
#include <limits>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

class VRFT {
   public:
    struct DiagnosticSample {
        std::size_t index;
        float       time;
        float       sample_dt;
        float       input;
        float       output;
        float       virtual_reference;
        float       virtual_error;
        float       kp_coefficient;
        float       ki_coefficient;
        float       kd_coefficient;
        bool        has_virtual_values;
        bool        used_for_regression;
    };

    VRFT () : count_ (0), time_step_ (0.0f), responseTime_ (0.0f) {}

    void timeStep (float seconds) {
        time_step_ = seconds;
    }

    void responseTime (float seconds) {
        responseTime_ = seconds;
    }

    void update (float in, float out) {
        const float time = count_ == 0 ? 0.0f : times_.back () + time_step_;
        update (in, out, time);
    }

    void update (float in, float out, float time) {
        u_.push_back (in);
        y_.push_back (out);
        times_.push_back (time);
        count_++;
    }

    std::array<float, 3> getPID () {
        std::array<float, 3> gain = {0.0f, 0.0f, 0.0f};

        if (!readyForDiagnosis ()) return gain;

        float      A[3][3] = {0.0f};
        float      b[3]    = {0.0f};
        const auto samples = diagnosticSamples ();

        for (const auto& sample : samples) {
            if (!sample.used_for_regression) continue;

            float phi[3] = {sample.kp_coefficient, sample.ki_coefficient, sample.kd_coefficient};

            for (int i = 0; i < 3; i++) {
                b[i] += phi[i] * sample.input;

                for (int j = 0; j < 3; j++) {
                    A[i][j] += phi[i] * phi[j];
                }
            }
        }

        if (!solve3x3 (A, b, gain)) {
            return {0.0f, 0.0f, 0.0f};
        }

        return gain;
    }

    struct PiResult {
        double kp = 0, ki = 0;
        double r_squared = -std::numeric_limits<double>::infinity();
        size_t used_samples = 0;
    };

    // Identify the actual PI used by STM, rather than fit a PID and discard Kd.
    // Solve two-variable nonnegative least squares, including both boundary cases.
    PiResult getPI () const {
        PiResult result;
        const auto samples = diagnosticSamples ();
        double pp = 0, pi = 0, ii = 0, pu = 0, iu = 0, sum_u = 0;
        for (const auto &s : samples) {
            if (!s.used_for_regression) continue;
            const double p = s.kp_coefficient, integral = s.ki_coefficient;
            if (!std::isfinite(p) || !std::isfinite(integral) || !std::isfinite(s.input)) return result;
            pp += p * p; pi += p * integral; ii += integral * integral;
            pu += p * s.input; iu += integral * s.input; sum_u += s.input;
            ++result.used_samples;
        }
        const double determinant = pp * ii - pi * pi;
        if (result.used_samples < 3 || pp <= 0 || ii <= 0 ||
            determinant <= 1e-8 * pp * ii) return result;
        const double free_p = (pu * ii - iu * pi) / determinant;
        const double free_i = (iu * pp - pu * pi) / determinant;
        const std::array<std::array<double, 2>, 4> candidates = {{
            {0, 0}, {std::max(0.0, pu / pp), 0}, {0, std::max(0.0, iu / ii)},
            {free_p, free_i}}};
        double best = std::numeric_limits<double>::infinity(), variance = 0;
        const double mean = sum_u / result.used_samples;
        for (const auto &s : samples)
            if (s.used_for_regression) variance += std::pow(s.input - mean, 2);
        for (const auto &gain : candidates) {
            if (gain[0] < 0 || gain[1] < 0 || !std::isfinite(gain[0]) || !std::isfinite(gain[1])) continue;
            double residual = 0;
            for (const auto &s : samples) {
                if (!s.used_for_regression) continue;
                residual += std::pow(s.input - gain[0] * s.kp_coefficient - gain[1] * s.ki_coefficient, 2);
            }
            if (residual < best) { best = residual; result.kp = gain[0]; result.ki = gain[1]; }
        }
        if (variance > std::numeric_limits<double>::epsilon()) result.r_squared = 1 - best / variance;
        return result;
    }

    std::vector<DiagnosticSample> diagnosticSamples () const {
        std::vector<DiagnosticSample> samples;
        samples.reserve (count_);

        std::vector<float> sample_dt (count_, 0.0f);
        std::vector<float> v_ref (count_, 0.0f);
        std::vector<float> err_v (count_, 0.0f);
        std::vector<bool>  has_virtual (count_, false);

        if (responseTime_ > 0.0f) {
            for (std::size_t k = 1; k < count_; k++) {
                sample_dt[k] = times_[k] - times_[k - 1];

                if (sample_dt[k] <= 0.0f) {
                    continue;
                }

                v_ref[k]       = responseTime_ * (y_[k] - y_[k - 1]) / sample_dt[k] + y_[k];
                err_v[k]       = v_ref[k] - y_[k];
                has_virtual[k] = true;
            }
        }

        float integral = 0.0f;

        for (std::size_t k = 0; k < count_; k++) {
            DiagnosticSample sample    = {};
            sample.index               = k;
            sample.time                = times_[k];
            sample.sample_dt           = sample_dt[k];
            sample.input               = u_[k];
            sample.output              = y_[k];
            sample.has_virtual_values  = has_virtual[k];
            sample.used_for_regression = (k >= 2 && has_virtual[k] && has_virtual[k - 1]);

            if (sample.has_virtual_values) {
                sample.virtual_reference = v_ref[k];
                sample.virtual_error     = err_v[k];
            }

            if (sample.used_for_regression) {
                integral += err_v[k] * sample_dt[k];
                sample.kp_coefficient = err_v[k];
                sample.ki_coefficient = integral;
                sample.kd_coefficient = (err_v[k] - err_v[k - 1]) / sample_dt[k];
            }

            samples.push_back (sample);
        }

        return samples;
    }

    std::size_t sampleCount () const {
        return count_;
    }

    void clear () {
        count_ = 0;
        u_.clear ();
        y_.clear ();
        times_.clear ();
    }

   private:
    bool readyForDiagnosis () const {
        return count_ >= 3 && responseTime_ > 0.0f && times_.size () == count_;
    }

    bool solve3x3 (float A[3][3], float b[3], std::array<float, 3>& x) const {
        const int N = 3;

        for (int i = 0; i < N; i++) {
            int   pivot  = i;
            float maxVal = std::fabs (A[i][i]);

            for (int r = i + 1; r < N; r++) {
                if (std::fabs (A[r][i]) > maxVal) {
                    maxVal = std::fabs (A[r][i]);
                    pivot  = r;
                }
            }

            if (maxVal < 1e-12f) {
                return false;
            }

            if (pivot != i) {
                for (int c = 0; c < N; c++) {
                    float tmp   = A[i][c];
                    A[i][c]     = A[pivot][c];
                    A[pivot][c] = tmp;
                }

                float tmp = b[i];
                b[i]      = b[pivot];
                b[pivot]  = tmp;
            }

            for (int r = i + 1; r < N; r++) {
                float factor = A[r][i] / A[i][i];

                for (int c = i; c < N; c++) {
                    A[r][c] -= factor * A[i][c];
                }

                b[r] -= factor * b[i];
            }
        }

        for (int i = N - 1; i >= 0; i--) {
            float sum = b[i];

            for (int c = i + 1; c < N; c++) {
                sum -= A[i][c] * x[static_cast<std::size_t> (c)];
            }

            x[static_cast<std::size_t> (i)] = sum / A[i][i];
        }

        return true;
    }

   private:
    std::size_t count_;

    float time_step_;
    float responseTime_;

    std::vector<float> u_;
    std::vector<float> y_;
    std::vector<float> times_;
};
