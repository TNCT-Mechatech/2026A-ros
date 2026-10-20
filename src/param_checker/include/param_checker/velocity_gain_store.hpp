#pragma once
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace param_checker {
// Update only the selected motor. A new inner PI requires a new outer P tuning.
inline void save_velocity_gain (const std::string &path, int motor, double kp, double ki, double tau, double max_rps, double fit, int ppr, double gear, int polarity) {
    const auto destination = std::filesystem::absolute (path);
    const auto temporary   = destination.string () + ".velocity.tmp";
    YAML::Node root        = std::filesystem::exists (destination) ? YAML::LoadFile (path) : YAML::Node (YAML::NodeType::Map);
    if (!root.IsMap () || (root["schema_version"] && root["schema_version"].as<int> () != 2)) throw std::runtime_error ("unsupported velocity gain YAML mapping/schema");
    auto wheel = root["wheel" + std::to_string (motor)];
    if (wheel && !wheel.IsMap ()) throw std::runtime_error ("target wheel gains must be a mapping");
    wheel.remove ("kp");
    wheel.remove ("ki");
    wheel.remove ("kd");
    wheel.remove ("position");
    auto velocity                     = wheel["velocity"];
    velocity["controller"]            = "PI";
    velocity["kp"]                    = kp;
    velocity["ki"]                    = ki;
    velocity["kd"]                    = 0.0;
    velocity["input_unit"]            = "rps";
    velocity["output_unit"]           = "duty";
    velocity["response_time_seconds"] = tau;
    velocity["max_rps"]               = max_rps;
    velocity["fit_r_squared"]         = fit;
    auto encoder                      = wheel["encoder"];
    encoder["pulses_per_revolution"]  = ppr;
    encoder["gear_ratio"]             = gear;
    encoder["counts_per_revolution"]  = 4.0 * ppr * gear;
    encoder["polarity"]               = polarity;
    std::filesystem::create_directories (destination.parent_path ());
    try {
        {
            std::ofstream stream (temporary);
            stream << std::setprecision (17) << root << '\n';
            stream.flush ();
            if (!stream) throw std::runtime_error ("velocity YAML write failed");
        }
        std::filesystem::rename (temporary, destination);
    } catch (...) {
        std::error_code ignored;
        std::filesystem::remove (temporary, ignored);
        throw;
    }
}
}  // namespace param_checker
