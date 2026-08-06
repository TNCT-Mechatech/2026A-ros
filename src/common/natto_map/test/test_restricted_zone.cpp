#include "natto_map/restricted_zone.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

natto_map::RestrictedZone make_zone (const std::vector<double> &points, bool lidar_ignore = true, bool keepout = true) {
    natto_map::RestrictedZone zone;
    std::string               error;
    EXPECT_TRUE (natto_map::make_restricted_zone ("test", points, lidar_ignore, keepout, zone, error)) << error;
    return zone;
}

TEST (RestrictedZone, ClassifiesInteriorExteriorEdgesAndVertices) {
    const auto zone = make_zone ({1.0, 2.0, 2.0, 2.0, 2.0, 3.0, 1.0, 3.0});
    EXPECT_TRUE (natto_map::point_in_restricted_zone (zone, 1.5, 2.5));
    EXPECT_FALSE (natto_map::point_in_restricted_zone (zone, 0.5, 2.5));
    EXPECT_TRUE (natto_map::point_in_restricted_zone (zone, 1.0, 2.5));
    EXPECT_TRUE (natto_map::point_in_restricted_zone (zone, 1.0, 2.0));
}

TEST (RestrictedZone, SupportsBothVertexOrdersAndConcavePolygons) {
    const auto clockwise        = make_zone ({0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 2.0, 0.0});
    const auto counterclockwise = make_zone ({0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0});
    const auto concave          = make_zone ({0.0, 0.0, 3.0, 0.0, 3.0, 1.0, 1.0, 1.0, 1.0, 3.0, 0.0, 3.0});
    EXPECT_TRUE (natto_map::point_in_restricted_zone (clockwise, 1.0, 1.0));
    EXPECT_TRUE (natto_map::point_in_restricted_zone (counterclockwise, 1.0, 1.0));
    EXPECT_TRUE (natto_map::point_in_restricted_zone (concave, 0.5, 2.0));
    EXPECT_FALSE (natto_map::point_in_restricted_zone (concave, 2.0, 2.0));
}

TEST (RestrictedZone, RemovesRepeatedClosingVertex) {
    const auto zone = make_zone ({0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0, 0.0, 0.0});
    EXPECT_EQ (zone.vertices.size (), 4u);
}

TEST (RestrictedZone, RejectsMalformedPoints) {
    natto_map::RestrictedZone zone;
    std::string               error;
    EXPECT_FALSE (natto_map::make_restricted_zone ("odd", {0.0, 0.0, 1.0}, true, true, zone, error));
    EXPECT_FALSE (natto_map::make_restricted_zone ("two", {0.0, 0.0, 1.0, 1.0}, true, true, zone, error));
    EXPECT_FALSE (natto_map::make_restricted_zone ("nan", {0.0, 0.0, 1.0, 0.0, std::numeric_limits<double>::quiet_NaN (), 1.0}, true, true, zone, error));
    EXPECT_FALSE (natto_map::make_restricted_zone ("inf", {0.0, 0.0, 1.0, 0.0, 0.0, std::numeric_limits<double>::infinity ()}, true, true, zone, error));
}

TEST (RestrictedZone, EmptyZonesLeaveGridUnchanged) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width      = 4;
    grid.info.height     = 4;
    grid.info.resolution = 1.0;
    grid.data.assign (16, 0);
    const auto before = grid.data;
    EXPECT_EQ (natto_map::rasterize_keepout_zones (grid, {}), 0u);
    EXPECT_EQ (grid.data, before);
}

TEST (RestrictedZone, RasterizesInteriorAndBoundaryCells) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width      = 5;
    grid.info.height     = 5;
    grid.info.resolution = 1.0;
    grid.data.assign (25, 0);
    const auto zone = make_zone ({1.25, 1.25, 3.25, 1.25, 3.25, 3.25, 1.25, 3.25});
    EXPECT_GT (natto_map::rasterize_keepout_zones (grid, {zone}), 0u);
    EXPECT_EQ (grid.data[2 * 5 + 2], 100);
    EXPECT_EQ (grid.data[1 * 5 + 1], 100);
}

TEST (RestrictedZone, DisabledFlagsHaveNoEffect) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width      = 3;
    grid.info.height     = 3;
    grid.info.resolution = 1.0;
    grid.data.assign (9, 0);
    const auto zone = make_zone ({0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0}, false, false);
    EXPECT_EQ (natto_map::rasterize_keepout_zones (grid, {zone}), 0u);
    EXPECT_FALSE (natto_map::point_in_lidar_ignore_zone ({zone}, 1.0, 1.0));
}

}  // namespace
