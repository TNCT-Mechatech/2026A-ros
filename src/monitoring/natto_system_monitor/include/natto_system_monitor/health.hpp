#pragma once
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <string>

namespace natto_system_monitor {
using Health = diagnostic_msgs::msg::DiagnosticStatus;
using Clock  = std::chrono::steady_clock;
inline double seconds (Clock::duration value) {
    return std::chrono::duration<double> (value).count ();
}
// Monotonic receipt time deliberately does not depend on /clock or sensor stamps.
struct Receipt {
    bool                          seen = false;
    Clock::time_point             last{};
    double                        ros_received = 0.0;
    double                        stamp        = 0.0;
    std::deque<Clock::time_point> samples;
    void                          tick (double ros_time, double source_stamp, double window, Clock::time_point now = Clock::now ()) {
        seen         = true;
        last         = now;
        ros_received = ros_time;
        stamp        = source_stamp;
        samples.push_back (now);
        while (samples.size () > 1 && seconds (now - samples.front ()) > window) samples.pop_front ();
    }
    double age (Clock::time_point now = Clock::now ()) const {
        return seen ? seconds (now - last) : -1.0;
    }
    double hz (Clock::time_point now = Clock::now ()) const {
        if (samples.size () < 2) return 0.0;
        const double span = seconds (now - samples.front ());
        return span > 0.0 ? static_cast<double> (samples.size () - 1) / span : 0.0;
    }
};
inline bool valid_pose (const geometry_msgs::msg::Pose &p, double tolerance) {
    const auto  &q    = p.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite (p.position.x) && std::isfinite (p.position.y) && std::isfinite (p.position.z) && std::isfinite (norm) && std::abs (norm - 1.0) <= tolerance;
}
inline std::string scan_error (const sensor_msgs::msg::LaserScan &s, double max_nan_fraction) {
    if (s.header.frame_id.empty ()) return "Empty frame_id";
    if (!std::isfinite (s.range_min) || !std::isfinite (s.range_max) || s.range_min < 0 || s.range_max <= s.range_min) return "Invalid range bounds";
    if (s.ranges.empty ()) return "Empty scan";
    if (!std::isfinite (s.angle_min) || !std::isfinite (s.angle_max) || !std::isfinite (s.angle_increment) || s.angle_increment == 0.0f || (s.angle_max - s.angle_min) / s.angle_increment < 0.0f) return "Invalid angle metadata";
    if (!std::isfinite (s.scan_time) || s.scan_time < 0 || !std::isfinite (s.time_increment) || s.time_increment < 0) return "Invalid scan timing metadata";
    const auto nans = std::count_if (s.ranges.begin (), s.ranges.end (), [] (float r) { return std::isnan (r); });
    // +/- infinity and finite out-of-range returns are valid no-return representations.
    if (static_cast<double> (nans) / static_cast<double> (s.ranges.size ()) > max_nan_fraction) return "Too many NaN ranges";
    return "";
}
inline unsigned char combine (unsigned char a, unsigned char b) {
    if (a == Health::ERROR || b == Health::ERROR) return Health::ERROR;
    if (a == Health::STALE || b == Health::STALE) return Health::STALE;
    return std::max (a, b);
}
}  // namespace natto_system_monitor
