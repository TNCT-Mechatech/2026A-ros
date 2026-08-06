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

#ifndef NATTO_MAP__RESTRICTED_ZONE_HPP_
#define NATTO_MAP__RESTRICTED_ZONE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <vector>

namespace natto_map {

struct Point2d {
    double x;
    double y;
};

struct RestrictedZone {
    std::string          name;
    std::vector<Point2d> vertices;
    bool                 lidar_ignore;
    bool                 keepout;
    double               min_x;
    double               max_x;
    double               min_y;
    double               max_y;
};

bool make_restricted_zone (const std::string &name, const std::vector<double> &points, bool lidar_ignore, bool keepout, RestrictedZone &zone, std::string &error);
bool point_in_restricted_zone (const RestrictedZone &zone, double x, double y);
bool point_in_lidar_ignore_zone (const std::vector<RestrictedZone> &zones, double x, double y);
std::vector<RestrictedZone> load_restricted_zones (rclcpp::Node &node);
size_t rasterize_keepout_zones (nav_msgs::msg::OccupancyGrid &grid, const std::vector<RestrictedZone> &zones);

}  // namespace natto_map

#endif  // NATTO_MAP__RESTRICTED_ZONE_HPP_
