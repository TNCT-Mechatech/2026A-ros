#include <natto_system_monitor/health.hpp>

#include <gtest/gtest.h>

#include <limits>
using namespace natto_system_monitor;
TEST (Health, UnknownIsNotAnError) {
    EXPECT_EQ (combine (Health::OK, Health::STALE), Health::STALE);
    EXPECT_EQ (combine (Health::WARN, Health::STALE), Health::STALE);
    EXPECT_EQ (combine (Health::ERROR, Health::STALE), Health::ERROR);
    EXPECT_EQ (combine (Health::OK, Health::WARN), Health::WARN);
}
TEST (Receipt, FrequencyAndAgeUseMonotonicTime) {
    Receipt    r;
    const auto t = Clock::now ();
    EXPECT_EQ (r.age (t), -1);
    r.tick (100, 90, 2, t);
    r.tick (1, 0, 2, t + std::chrono::milliseconds (100));
    EXPECT_NEAR (r.hz (t + std::chrono::milliseconds (100)), 10, 1e-6);
    EXPECT_NEAR (r.age (t + std::chrono::milliseconds (600)), .5, 1e-6);
    EXPECT_NEAR (r.hz (t + std::chrono::milliseconds (500)), 2, 1e-6);
}
TEST (Scan, MissingReturnsAreNotMalformedData) {
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "lidar_front";
    scan.range_min       = .1f;
    scan.range_max       = 30;
    scan.angle_min       = 0;
    scan.angle_max       = 1;
    scan.angle_increment = .5;
    scan.ranges          = {1, std::numeric_limits<float>::infinity (), 0};
    EXPECT_TRUE (scan_error (scan, .5).empty ());
    scan.ranges.assign (3, std::numeric_limits<float>::quiet_NaN ());
    EXPECT_FALSE (scan_error (scan, .5).empty ());
    scan.ranges    = {1, 2, 3};
    scan.range_max = 0;
    EXPECT_FALSE (scan_error (scan, .5).empty ());
}
TEST (Pose, RejectNonFiniteAndInvalidQuaternion) {
    geometry_msgs::msg::Pose p;
    p.orientation.w = 1;
    EXPECT_TRUE (valid_pose (p, .01));
    p.orientation.w = 0;
    EXPECT_FALSE (valid_pose (p, .01));
    p.orientation.w = 1;
    p.position.x    = std::numeric_limits<double>::infinity ();
    EXPECT_FALSE (valid_pose (p, .01));
}
