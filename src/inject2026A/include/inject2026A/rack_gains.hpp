#pragma once

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace inject2026a {
struct RackGains {
    float vel_kp = 0, vel_ki = 0, pos_kp = 0, max_rps = 0;
    uint16_t ppr = 0;
    float gear = 0;
    int polarity = 0;
    double response_s = 0, delay_s = 0, steady_gain = 1;
    bool model_ready = false;
};
inline RackGains load_rack_gains(const std::string &path, float fallback_max_rps = 0.03f) {
    const auto root = YAML::LoadFile(path);
    if (root["schema_version"] && root["schema_version"].as<int>() != 2)
        throw std::runtime_error("rack PID file schema_version must be 2");
    const auto wheel = root["wheel2"];
    const auto velocity = wheel["velocity"] ? wheel["velocity"] : wheel;
    const auto position = wheel["position"];
    auto number = [](const YAML::Node &node, const char *name) {
        if (!node[name]) throw std::runtime_error(std::string("rack PID field missing: ") + name);
        const double value = node[name].as<double>();
        if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
            throw std::runtime_error(std::string("invalid rack PID field: ") + name);
        return static_cast<float>(value);
    };
    auto units = [](const YAML::Node &node, const char *controller, const char *in, const char *out) {
        if (node["controller"].as<std::string>() != controller ||
            node["input_unit"].as<std::string>() != in || node["output_unit"].as<std::string>() != out)
            throw std::runtime_error("rack PID controller/units mismatch");
    };
    if (wheel["velocity"]) units(velocity, "PI", "rps", "duty");
    units(position, "P", "encoder_count", "rps");
    RackGains result;
    result.vel_kp = number(velocity, "kp");
    result.vel_ki = number(velocity, "ki");
    result.pos_kp = number(position, "kp");
    result.max_rps = std::min(velocity["max_rps"] ? number(velocity, "max_rps") : fallback_max_rps,
                              number(position, "max_rps"));
    if (velocity["kd"]) (void)number(velocity, "kd"); // Legacy saved Kd is ignored, like position_checker.
    if (result.vel_kp <= 0 || result.vel_ki < 0 || result.pos_kp <= 0 || result.max_rps <= 0 ||
        number(position, "ki") != 0 || number(position, "kd") != 0)
        throw std::runtime_error("rack requires velocity PI and position P with valid gains/limits");
    if (velocity["response_time_seconds"] && position["response_time_seconds"] &&
        number(position, "response_time_seconds") <= number(velocity, "response_time_seconds"))
        throw std::runtime_error("rack position response must be slower than velocity response");
    const bool calibration = bool(position["encoder_ppr"]) || bool(position["gear_rate"]) ||
                             bool(position["encoder_polarity"]);
    if (calibration) {
        const auto ppr = position["encoder_ppr"].as<int>();
        result.gear = number(position, "gear_rate");
        result.polarity = position["encoder_polarity"].as<int>();
        if (ppr < 1 || ppr > 65535 || result.gear <= 0 ||
            (result.polarity != -1 && result.polarity != 1))
            throw std::runtime_error("invalid rack PID encoder calibration");
        result.ppr = static_cast<uint16_t>(ppr);
    }
    if (wheel["encoder"]) {
        const auto encoder = wheel["encoder"];
        const auto ppr = encoder["pulses_per_revolution"].as<int>();
        const float gear = number(encoder, "gear_ratio");
        if (ppr < 1 || ppr > 65535 || gear <= 0 ||
            (result.ppr && (result.ppr != ppr || std::abs(result.gear - gear) > 1e-4)))
            throw std::runtime_error("rack encoder calibration mismatch");
        if (encoder["counts_per_revolution"] &&
            std::abs(number(encoder, "counts_per_revolution") / (4.0 * ppr * gear) - 1) > 1e-4)
            throw std::runtime_error("rack encoder CPR mismatch");
        result.ppr = static_cast<uint16_t>(ppr);
        result.gear = gear;
    }
    if (position["identification_method"] &&
        position["identification_method"].as<std::string>() == "velocity_response_model_v1") {
        result.response_s = number(position, "velocity_response_time_seconds");
        result.delay_s = number(position, "velocity_delay_seconds");
        result.steady_gain = number(position, "velocity_steady_gain");
        if (result.response_s < 0 || result.response_s > 2 || result.delay_s < 0 ||
            result.delay_s > 0.3 || result.steady_gain < 0.5 || result.steady_gain > 1.5)
            throw std::runtime_error("rack velocity response model outside supported range");
        result.model_ready = true;
    }
    return result;
}
} // namespace inject2026a
