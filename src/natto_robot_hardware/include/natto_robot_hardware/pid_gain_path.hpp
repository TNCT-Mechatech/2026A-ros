#pragma once

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace natto_robot_hardware {

// Generated calibration data must stay writable and must not require rebuilding.
inline std::string resolve_pid_gain_path (const std::string &configured_path) {
    if (!configured_path.empty ()) {
        return std::filesystem::absolute (configured_path).lexically_normal ().string ();
    }
    const char *ros_home = std::getenv ("ROS_HOME");
    if (ros_home != nullptr && ros_home[0] != '\0') {
        return (std::filesystem::absolute (ros_home) / "natto" / "pid_gain.yaml").lexically_normal ().string ();
    }
    const char *user_home = std::getenv ("HOME");
    if (user_home == nullptr || user_home[0] == '\0') {
        throw std::runtime_error ("Set a PID gain path explicitly, or set ROS_HOME or HOME");
    }
    return (std::filesystem::absolute (user_home) / ".ros" / "natto" / "pid_gain.yaml").lexically_normal ().string ();
}

}  // namespace natto_robot_hardware
