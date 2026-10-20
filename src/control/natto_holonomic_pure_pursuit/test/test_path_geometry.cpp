#include "natto_holonomic_pure_pursuit/path_geometry.hpp"

#include <gtest/gtest.h>

using holonomic_pure_pursuit::path_geometry::PathGeometry;
using holonomic_pure_pursuit::path_geometry::Point;

TEST (PathGeometry, RepeatedRotationPosesDoNotTrapLookaheadAtRecordedStartupPoint) {
    // 18:52 bag: nearest vertex is only 4.5 cm away despite a 10 cm request.
    PathGeometry path ({{3.8058, 9.9581, 0.11}, {3.8058, 9.9581, 0.15}, {3.7350, 9.8720, 0.18}, {3.7350, 9.8720, 0.21}});
    const auto look = path.lookahead (3.8408, 9.9866, 0, 0.1);
    EXPECT_NEAR (std::hypot (look.x - 3.8408, look.y - 9.9866), 0.1, 1e-9);
    EXPECT_GT (look.yaw, 0.15);
    EXPECT_LT (look.yaw, 0.18);
}

TEST (PathGeometry, OffPathLookaheadIsOnSegmentAndRequestedCircle) {
    PathGeometry path ({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});
    const auto look = path.lookahead (0.1, 0.06, 0, 0.2);
    EXPECT_NEAR (look.y, 0.0, 1e-12);
    EXPECT_NEAR (std::hypot (look.x - 0.1, look.y - 0.06), 0.2, 1e-12);
    EXPECT_GT (look.x, 0.1);
}

TEST (PathGeometry, YawInterpolationTakesShortRouteAcrossPi) {
    const double pi = std::acos (-1.0);
    PathGeometry path ({{0.0, 0.0, 179.0 * pi / 180.0}, {1.0, 0.0, -179.0 * pi / 180.0}});
    const auto look = path.lookahead (0.0, 0.0, 0, 0.5);
    EXPECT_NEAR (std::abs (look.yaw), pi, 1e-12);
}

TEST (PathGeometry, ShortPathPreservesDestinationAndOrientation) {
    PathGeometry path ({{0.0, 0.0, 0.0}, {0.07, 0.0, 0.9}});
    const auto look = path.lookahead (0.0, 0.0, 0, 0.2);
    EXPECT_DOUBLE_EQ (look.x, 0.07);
    EXPECT_DOUBLE_EQ (look.yaw, 0.9);
}

TEST (PathGeometry, NearbyDestinationDoesNotShortcutAnOutgoingLoop) {
    PathGeometry path ({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.05, 0.0, 0.0}});
    const auto look = path.lookahead (0.0, 0.0, 0, 0.2);
    EXPECT_NEAR (look.x, 0.2, 1e-12);
    EXPECT_NEAR (look.y, 0.0, 1e-12);
}

TEST (PathGeometry, PureRotationKeepsFinalYawAndZeroGeometricCurvature) {
    PathGeometry path ({{1.0, 2.0, 0.0}, {1.0, 2.0, 0.7}, {1.0, 2.0, 1.5}});
    const auto look = path.lookahead (1.0, 2.0, 0, 0.2);
    EXPECT_DOUBLE_EQ (look.x, 1.0);
    EXPECT_DOUBLE_EQ (look.y, 2.0);
    EXPECT_DOUBLE_EQ (look.yaw, 1.5);
    EXPECT_DOUBLE_EQ (path.curvature (1.0, 2.0, 0.3), 0.0);
}

TEST (PathGeometry, CurvatureIgnoresDuplicatePoseDensity) {
    const std::vector<Point> points {{0.0, 0.0, 0.0}, {0.1, 0.0, 0.1}, {0.2, 0.1, 0.2}, {0.25, 0.3, 0.3}, {0.25, 0.5, 0.4}};
    std::vector<Point> repeated;
    for (const auto &p : points) {
        for (int i = 0; i < 5; ++i) repeated.push_back ({p.x, p.y, p.yaw + 0.01 * i});
    }
    PathGeometry plain (points), rotations (repeated);
    const double curve = plain.curvature (0.1, 0.0, 0.3);
    EXPECT_GT (curve, 0.1);
    EXPECT_NEAR (rotations.curvature (0.1, 0.0, 0.3), curve, 1e-9);
}

TEST (PathGeometry, CurvatureIgnoresExtraVerticesOnSameSegments) {
    PathGeometry coarse ({{0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}, {0.4, 0.2, 0.0}, {0.4, 0.5, 0.0}});
    PathGeometry dense ({{0.0, 0.0, 0.0}, {0.05, 0.0, 0.0}, {0.2, 0.0, 0.0}, {0.25, 0.05, 0.0}, {0.4, 0.2, 0.0}, {0.4, 0.3, 0.0}, {0.4, 0.5, 0.0}});
    EXPECT_NEAR (coarse.curvature (0.1, 0.0, 0.3), dense.curvature (0.1, 0.0, 0.3), 1e-9);
}

TEST (PathGeometry, ShortRemainingCurveDoesNotUseDuplicateEndpointSamples) {
    PathGeometry path ({{0.0, 0.0, 0.0}, {0.05, 0.0, 0.0}, {0.05, 0.05, 0.0}});
    EXPECT_GT (path.curvature (0.0, 0.0, 0.3), 0.0);
    EXPECT_DOUBLE_EQ (path.curvature (0.05, 0.05, 0.3), 0.0);
}

TEST (PathGeometry, EmptyAndStraightPathsStayFinite) {
    PathGeometry empty;
    EXPECT_DOUBLE_EQ (empty.curvature (0.0, 0.0, 0.3), 0.0);
    EXPECT_DOUBLE_EQ (empty.lookahead (0.0, 0.0, 0, 0.2).x, 0.0);
    PathGeometry straight ({{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.001, 0.0, 0.0}, {1.0, 0.0, 0.0}});
    EXPECT_DOUBLE_EQ (straight.curvature (0.2, 0.1, 0.3), 0.0);
}
