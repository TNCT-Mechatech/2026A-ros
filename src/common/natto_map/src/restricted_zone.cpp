// Copyright 2026 Kazusa Hashimoto
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "natto_map/restricted_zone.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace natto_map {
namespace {

constexpr double kEpsilon = 1.0e-10;

bool same_point (const Point2d &a, const Point2d &b) {
    return std::abs (a.x - b.x) <= kEpsilon && std::abs (a.y - b.y) <= kEpsilon;
}

double cross (const Point2d &a, const Point2d &b, const Point2d &p) {
    return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

bool point_on_segment (const Point2d &p, const Point2d &a, const Point2d &b) {
    const double scale = std::max ({1.0, std::abs (a.x), std::abs (a.y), std::abs (b.x), std::abs (b.y), std::abs (p.x), std::abs (p.y)});
    if (std::abs (cross (a, b, p)) > kEpsilon * scale * scale) return false;
    return p.x >= std::min (a.x, b.x) - kEpsilon && p.x <= std::max (a.x, b.x) + kEpsilon && p.y >= std::min (a.y, b.y) - kEpsilon && p.y <= std::max (a.y, b.y) + kEpsilon;
}

int orientation (const Point2d &a, const Point2d &b, const Point2d &c) {
    const double value = cross (a, b, c);
    if (std::abs (value) <= kEpsilon) return 0;
    return value > 0.0 ? 1 : -1;
}

bool segments_intersect (const Point2d &a, const Point2d &b, const Point2d &c, const Point2d &d) {
    const int o1 = orientation (a, b, c);
    const int o2 = orientation (a, b, d);
    const int o3 = orientation (c, d, a);
    const int o4 = orientation (c, d, b);
    if (o1 != o2 && o3 != o4) return true;
    return (o1 == 0 && point_on_segment (c, a, b)) || (o2 == 0 && point_on_segment (d, a, b)) || (o3 == 0 && point_on_segment (a, c, d)) || (o4 == 0 && point_on_segment (b, c, d));
}

bool segment_intersects_cell (const Point2d &a, const Point2d &b, double min_x, double max_x, double min_y, double max_y) {
    auto in_cell = [&] (const Point2d &p) { return p.x >= min_x - kEpsilon && p.x <= max_x + kEpsilon && p.y >= min_y - kEpsilon && p.y <= max_y + kEpsilon; };
    if (in_cell (a) || in_cell (b)) return true;
    const Point2d bottom_left{min_x, min_y};
    const Point2d bottom_right{max_x, min_y};
    const Point2d top_right{max_x, max_y};
    const Point2d top_left{min_x, max_y};
    return segments_intersect (a, b, bottom_left, bottom_right) || segments_intersect (a, b, bottom_right, top_right) || segments_intersect (a, b, top_right, top_left) || segments_intersect (a, b, top_left, bottom_left);
}

}  // namespace

bool make_restricted_zone (const std::string &name, const std::vector<double> &points, bool lidar_ignore, bool keepout, RestrictedZone &zone, std::string &error) {
    if (points.size () % 2 != 0) {
        error = "points must contain an even number of values";
        return false;
    }
    if (points.size () < 6) {
        error = "at least three vertices are required";
        return false;
    }

    RestrictedZone candidate;
    candidate.name         = name;
    candidate.lidar_ignore = lidar_ignore;
    candidate.keepout      = keepout;
    candidate.min_x        = std::numeric_limits<double>::infinity ();
    candidate.max_x        = -std::numeric_limits<double>::infinity ();
    candidate.min_y        = std::numeric_limits<double>::infinity ();
    candidate.max_y        = -std::numeric_limits<double>::infinity ();

    for (size_t i = 0; i < points.size (); i += 2) {
        if (!std::isfinite (points[i]) || !std::isfinite (points[i + 1])) {
            error = "points must not contain NaN or Inf";
            return false;
        }
        candidate.vertices.push_back ({points[i], points[i + 1]});
    }

    if (candidate.vertices.size () > 1 && same_point (candidate.vertices.front (), candidate.vertices.back ())) candidate.vertices.pop_back ();
    if (candidate.vertices.size () < 3) {
        error = "at least three vertices are required after removing the repeated closing vertex";
        return false;
    }

    double twice_area = 0.0;
    for (size_t i = 0; i < candidate.vertices.size (); ++i) {
        const Point2d &a = candidate.vertices[i];
        const Point2d &b = candidate.vertices[(i + 1) % candidate.vertices.size ()];
        twice_area += a.x * b.y - b.x * a.y;
        candidate.min_x = std::min (candidate.min_x, a.x);
        candidate.max_x = std::max (candidate.max_x, a.x);
        candidate.min_y = std::min (candidate.min_y, a.y);
        candidate.max_y = std::max (candidate.max_y, a.y);
    }
    if (std::abs (twice_area) <= kEpsilon) {
        error = "polygon area must be non-zero";
        return false;
    }

    zone = std::move (candidate);
    error.clear ();
    return true;
}

bool point_in_restricted_zone (const RestrictedZone &zone, double x, double y) {
    if (x < zone.min_x - kEpsilon || x > zone.max_x + kEpsilon || y < zone.min_y - kEpsilon || y > zone.max_y + kEpsilon) return false;
    const Point2d p{x, y};
    bool          inside = false;
    for (size_t i = 0, j = zone.vertices.size () - 1; i < zone.vertices.size (); j = i++) {
        const Point2d &a = zone.vertices[j];
        const Point2d &b = zone.vertices[i];
        if (point_on_segment (p, a, b)) return true;
        if ((a.y > y) == (b.y > y)) continue;
        const double intersection_x = a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y);
        if (x < intersection_x) inside = !inside;
    }
    return inside;
}

bool point_in_lidar_ignore_zone (const std::vector<RestrictedZone> &zones, double x, double y) {
    for (const auto &zone : zones) {
        if (zone.lidar_ignore && point_in_restricted_zone (zone, x, y)) return true;
    }
    return false;
}

std::vector<RestrictedZone> load_restricted_zones (rclcpp::Node &node) {
    std::vector<RestrictedZone> zones;
    std::vector<std::string>    names;
    try {
        const auto &overrides  = node.get_node_parameters_interface ()->get_parameter_overrides ();
        const auto  override   = overrides.find ("restricted_zone_names");
        bool        valid_type = true;
        if (override != overrides.end ()) {
            if (override->second.get_type () == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
                names = override->second.get<std::vector<std::string>> ();
            } else if (override->second.get_type () != rclcpp::ParameterType::PARAMETER_NOT_SET) {
                valid_type = false;
                RCLCPP_WARN (node.get_logger (), "Invalid restricted_zone_names: expected a string array. Restricted zones are disabled.");
            }
            node.declare_parameter<std::vector<std::string>> ("restricted_zone_names", names, rcl_interfaces::msg::ParameterDescriptor{}, true);
        } else {
            names = node.declare_parameter<std::vector<std::string>> ("restricted_zone_names", std::vector<std::string>{});
        }
        if (!valid_type) return zones;
    } catch (const std::exception &error) {
        RCLCPP_WARN (node.get_logger (), "Invalid restricted_zone_names: %s. Restricted zones are disabled.", error.what ());
        return zones;
    }

    std::unordered_set<std::string> seen_names;
    for (const auto &name : names) {
        if (name.empty ()) {
            if (names.size () != 1) RCLCPP_WARN (node.get_logger (), "Ignoring restricted zone with an empty name.");
            continue;
        }
        if (!seen_names.insert (name).second) {
            RCLCPP_WARN (node.get_logger (), "Ignoring duplicate restricted zone name '%s'.", name.c_str ());
            continue;
        }
        try {
            const std::string prefix       = "restricted_zones." + name + ".";
            const auto        points       = node.declare_parameter<std::vector<double>> (prefix + "points", std::vector<double>{});
            const bool        lidar_ignore = node.declare_parameter<bool> (prefix + "lidar_ignore", false);
            const bool        keepout      = node.declare_parameter<bool> (prefix + "keepout", false);
            RestrictedZone    zone;
            std::string       error;
            if (!make_restricted_zone (name, points, lidar_ignore, keepout, zone, error)) {
                RCLCPP_WARN (node.get_logger (), "Ignoring invalid restricted zone '%s': %s.", name.c_str (), error.c_str ());
                continue;
            }
            zones.push_back (std::move (zone));
        } catch (const std::exception &error) {
            RCLCPP_WARN (node.get_logger (), "Ignoring invalid restricted zone '%s': %s.", name.c_str (), error.what ());
        }
    }
    return zones;
}

size_t rasterize_keepout_zones (nav_msgs::msg::OccupancyGrid &grid, const std::vector<RestrictedZone> &zones) {
    const int    width      = static_cast<int> (grid.info.width);
    const int    height     = static_cast<int> (grid.info.height);
    const double resolution = grid.info.resolution;
    const double origin_x   = grid.info.origin.position.x;
    const double origin_y   = grid.info.origin.position.y;
    if (width <= 0 || height <= 0 || resolution <= 0.0 || grid.data.size () < static_cast<size_t> (width * height)) return 0;

    size_t changed_cells = 0;
    for (const auto &zone : zones) {
        if (!zone.keepout) continue;
        const int min_gx = std::max (0, static_cast<int> (std::floor ((zone.min_x - origin_x) / resolution)) - 1);
        const int max_gx = std::min (width - 1, static_cast<int> (std::floor ((zone.max_x - origin_x) / resolution)) + 1);
        const int min_gy = std::max (0, static_cast<int> (std::floor ((zone.min_y - origin_y) / resolution)) - 1);
        const int max_gy = std::min (height - 1, static_cast<int> (std::floor ((zone.max_y - origin_y) / resolution)) + 1);
        if (min_gx > max_gx || min_gy > max_gy) continue;

        for (int gy = min_gy; gy <= max_gy; ++gy) {
            for (int gx = min_gx; gx <= max_gx; ++gx) {
                const double cell_min_x = origin_x + static_cast<double> (gx) * resolution;
                const double cell_min_y = origin_y + static_cast<double> (gy) * resolution;
                const double cell_max_x = cell_min_x + resolution;
                const double cell_max_y = cell_min_y + resolution;
                bool         occupied   = point_in_restricted_zone (zone, cell_min_x + 0.5 * resolution, cell_min_y + 0.5 * resolution);
                if (!occupied) {
                    for (size_t i = 0; i < zone.vertices.size (); ++i) {
                        const Point2d &a = zone.vertices[i];
                        const Point2d &b = zone.vertices[(i + 1) % zone.vertices.size ()];
                        if (segment_intersects_cell (a, b, cell_min_x, cell_max_x, cell_min_y, cell_max_y)) {
                            occupied = true;
                            break;
                        }
                    }
                }
                const size_t index = static_cast<size_t> (gy * width + gx);
                if (occupied && grid.data[index] != 100) {
                    grid.data[index] = 100;
                    ++changed_cells;
                }
            }
        }
    }
    return changed_cells;
}

}  // namespace natto_map
