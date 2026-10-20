#include "natto_astar_planner/astar_planner.hpp"

#include <gtest/gtest.h>

#include <functional>
#include <thread>

namespace {

using Planner = astar_planner::astar_planner;

nav_msgs::msg::OccupancyGrid wall_map () {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id           = "map";
    map.info.resolution           = 0.05f;
    map.info.width                = 110;
    map.info.height               = 221;
    map.info.origin.orientation.w = 1.0;
    map.data.assign (map.info.width * map.info.height, 0);
    for (size_t x = 0; x < map.info.width; ++x) map.data[210 * map.info.width + x] = 100;  // y=10.5 wall
    return map;
}

geometry_msgs::msg::PoseStamped pose_at (double x, double y, double yaw_deg = 0.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id    = "map";
    pose.pose.position.x    = x;
    pose.pose.position.y    = y;
    pose.pose.orientation.z = std::sin (yaw_deg * M_PI / 360.0);
    pose.pose.orientation.w = std::cos (yaw_deg * M_PI / 360.0);
    return pose;
}

class Harness {
   public:
    explicit Harness (const std::vector<double> &bounds = {}, bool keepout = false, const std::vector<double> &reflection_x = {}, const std::vector<double> &zone_points = {}, bool reflect_yaw = false) {
        static int          count = 0;
        const auto          ns    = "/start_area_test_" + std::to_string (++count);
        rclcpp::NodeOptions options;
        options.arguments ({"--ros-args", "-r", "__ns:=" + ns});
        options.append_parameter_override ("managed_navigation", true);
        options.append_parameter_override ("xy_inflation", 0.40);
        options.append_parameter_override ("xy_offset", 0.30);
        options.append_parameter_override ("start_area_bounds", bounds);
        options.append_parameter_override ("start_area_xy_inflation", 0.30);
        if (!reflection_x.empty ()) options.append_parameter_override ("keepout_grid_reflection_x", reflection_x);
        if (reflect_yaw) options.append_parameter_override ("angular_grid_reflect_yaw", true);
        if (keepout) {
            options.append_parameter_override ("restricted_zone_names", std::vector<std::string>{"refill"});
            // Includes a source cell on the wall, to test overlapping sources.
            options.append_parameter_override ("restricted_zones.refill.points", zone_points.empty () ? std::vector<double>{4.4, 9.675, 4.85, 9.675, 4.85, 10.6, 4.4, 10.6} : zone_points);
            options.append_parameter_override ("restricted_zones.refill.keepout", true);
        }
        planner        = std::make_shared<Planner> (options);
        probe          = std::make_shared<rclcpp::Node> ("probe", ns);
        const auto qos = rclcpp::QoS (1).transient_local ().reliable ();
        map_pub        = probe->create_publisher<nav_msgs::msg::OccupancyGrid> ("occupancy_grid", qos);
        footprint_pub  = probe->create_publisher<geometry_msgs::msg::PolygonStamped> ("footprint", 10);
        pose_pub       = probe->create_publisher<geometry_msgs::msg::PoseStamped> ("current_pose", 10);
        goal_pub       = probe->create_publisher<natto_msgs::msg::NavigationGoal> ("navigation_goal", 10);
        cost_sub       = probe->create_subscription<nav_msgs::msg::OccupancyGrid> ("costmap", qos, [this] (nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { costmap = *msg; });
        path_sub       = probe->create_subscription<natto_msgs::msg::NavigationPath> ("navigation_path", 10, [this] (natto_msgs::msg::NavigationPath::ConstSharedPtr msg) { paths.push_back (*msg); });
        executor.add_node (planner);
        executor.add_node (probe);
    }

    bool until (const std::function<bool ()> &condition) {
        const auto deadline = std::chrono::steady_clock::now () + std::chrono::seconds (5);
        while (!condition () && std::chrono::steady_clock::now () < deadline) {
            executor.spin_some ();
            std::this_thread::sleep_for (std::chrono::milliseconds (5));
        }
        return condition ();
    }

    bool load (const nav_msgs::msg::OccupancyGrid &map) {
        if (!until ([this] { return map_pub->get_subscription_count () && footprint_pub->get_subscription_count () && pose_pub->get_subscription_count () && goal_pub->get_subscription_count () && probe->count_publishers ("navigation_path"); }))
            return false;
        map_pub->publish (map);
        return until ([this] { return !costmap.data.empty (); });
    }

    bool plan (double x, double y, double goal_y, double goal_x = std::numeric_limits<double>::quiet_NaN (), double start_yaw = 0.0, double goal_yaw = 0.0) {
        geometry_msgs::msg::PolygonStamped footprint;
        footprint.header.frame_id = "base_link";
        for (const auto &xy : std::vector<std::pair<float, float>>{
                 { 0.43f,  0.35f},
                 {-0.43f,  0.35f},
                 {-0.43f, -0.35f},
                 { 0.43f, -0.35f}
        }) {
            geometry_msgs::msg::Point32 point;
            point.x = xy.first;
            point.y = xy.second;
            footprint.polygon.points.push_back (point);
        }
        footprint_pub->publish (footprint);
        pose_pub->publish (pose_at (x, y, start_yaw));
        natto_msgs::msg::NavigationGoal goal;
        goal.request_id = "test-start";
        goal.pose       = pose_at (std::isfinite (goal_x) ? goal_x : x, goal_y, goal_yaw);
        goal_pub->publish (goal);
        return until ([this] { return !paths.empty (); });
    }

    int cost (double x, double y) const {
        const auto gx = static_cast<size_t> (x / costmap.info.resolution);
        const auto gy = static_cast<size_t> (y / costmap.info.resolution);
        return costmap.data.at (gy * costmap.info.width + gx);
    }

    nav_msgs::msg::OccupancyGrid                 costmap;
    std::vector<natto_msgs::msg::NavigationPath> paths;

   private:
    rclcpp::executors::SingleThreadedExecutor                        executor;
    std::shared_ptr<Planner>                                         planner;
    rclcpp::Node::SharedPtr                                          probe;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr       map_pub;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    pose_pub;
    rclcpp::Publisher<natto_msgs::msg::NavigationGoal>::SharedPtr    goal_pub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    cost_sub;
    rclcpp::Subscription<natto_msgs::msg::NavigationPath>::SharedPtr path_sub;
};

class StartArea : public testing::Test {
   protected:
    static void SetUpTestSuite () {
        rclcpp::init (0, nullptr);
    }
    static void TearDownTestSuite () {
        rclcpp::shutdown ();
    }
};

TEST_F (StartArea, RedAndBlueRelaxOnlyTheirOwnRectangle) {
    auto map = wall_map ();
    // A second obstacle makes the lower y boundary observable as well.
    map.data[183 * map.info.width + 30] = 100;
    Harness normal;
    ASSERT_TRUE (normal.load (map));
    for (const auto &bounds : std::vector<std::vector<double>>{
             {3.4, 4.4, 9.5, 10.5},
             {1.0, 2.0, 9.5, 10.5}
    }) {
        Harness local (bounds);
        ASSERT_TRUE (local.load (map));
        const double middle_x = (bounds[0] + bounds[1]) / 2.0;
        EXPECT_EQ (normal.cost (middle_x, 10.125), 100);
        EXPECT_LT (local.cost (middle_x, 10.125), 100);
        EXPECT_EQ (local.cost (middle_x, 10.525), 100);  // Actual wall stays occupied.
        for (size_t y = 0; y < map.info.height; ++y) {
            for (size_t x = 0; x < map.info.width; ++x) {
                const double wx = (static_cast<double> (x) + 0.5) * map.info.resolution;
                const double wy = (static_cast<double> (y) + 0.5) * map.info.resolution;
                if (wx < bounds[0] || wx > bounds[1] || wy < bounds[2] || wy > bounds[3]) {
                    EXPECT_EQ (local.costmap.data[y * map.info.width + x], normal.costmap.data[y * map.info.width + x]) << wx << ", " << wy;
                }
            }
        }
    }
}

TEST_F (StartArea, KeepoutInflationStaysBlockedInsideStartArea) {
    Harness normal ({}, true);
    Harness local ({3.4, 4.4, 9.5, 10.5}, true);
    ASSERT_TRUE (normal.load (wall_map ()));
    ASSERT_TRUE (local.load (wall_map ()));
    EXPECT_LT (local.cost (3.9, 10.125), 100);
    EXPECT_EQ (local.cost (4.075, 10.025), 100);  // Keepout margin inside the rectangle.
    EXPECT_EQ (local.cost (4.425, 10.525), 100);  // Wall / keepout overlap.
    EXPECT_EQ (local.cost (4.075, 10.675), normal.cost (4.075, 10.675));
}

TEST_F (StartArea, WallNearStartCanConnectToPathOutsideStartArea) {
    for (const auto &bounds : std::vector<std::vector<double>>{
             {3.4, 4.4, 9.5, 10.5},
             {1.0, 2.0, 9.5, 10.5}
    }) {
        const double x = (bounds[0] + bounds[1]) / 2.0;
        Harness      normal;
        ASSERT_TRUE (normal.load (wall_map ()));
        ASSERT_TRUE (normal.plan (x, 10.125, 9.0));
        EXPECT_FALSE (normal.paths.back ().valid);
        Harness local (bounds);
        ASSERT_TRUE (local.load (wall_map ()));
        ASSERT_TRUE (local.plan (x, 10.125, 9.0));
        ASSERT_TRUE (local.paths.back ().valid) << local.paths.back ().reason;
        EXPECT_DOUBLE_EQ (local.paths.back ().path.poses.front ().pose.position.y, 10.125);
        EXPECT_DOUBLE_EQ (local.paths.back ().path.poses.back ().pose.position.y, 9.0);
    }
}

TEST_F (StartArea, ActualFootprintCollisionStillRejectsStartAndDestination) {
    for (bool blocked_start : {true, false}) {
        Harness local ({3.4, 4.4, 9.5, 10.5});
        ASSERT_TRUE (local.load (wall_map ()));
        ASSERT_TRUE (local.plan (3.9, blocked_start ? 10.3 : 10.0, blocked_start ? 9.0 : 10.3));
        EXPECT_FALSE (local.paths.back ().valid);
        EXPECT_EQ (local.paths.back ().reason, "start or destination is blocked");
    }
}

TEST_F (StartArea, InvalidBoundsAndInflationAreRejected) {
    for (const auto &bounds : std::vector<std::vector<double>>{
             {3.4, 4.4, 9.5},
             {4.4, 3.4, 9.5, 10.5},
             {3.4, 4.4, 10.5, 9.5},
             {3.4, 4.4, 9.5, std::numeric_limits<double>::infinity ()}
    }) {
        rclcpp::NodeOptions options;
        options.append_parameter_override ("start_area_bounds", bounds);
        EXPECT_THROW (std::make_shared<Planner> (options), std::invalid_argument);
    }
    for (double inflation : {-0.1, 0.6, std::numeric_limits<double>::quiet_NaN ()}) {
        rclcpp::NodeOptions options;
        options.append_parameter_override ("start_area_bounds", std::vector<double>{3.4, 4.4, 9.5, 10.5});
        options.append_parameter_override ("start_area_xy_inflation", inflation);
        EXPECT_THROW (std::make_shared<Planner> (options), std::invalid_argument);
    }
}

TEST_F (StartArea, ReflectedRedKeepoutMatchesLegacyBlueAtFloatGridBoundaries) {
    auto map        = wall_map ();
    map.info.width  = 109;
    map.info.height = 211;
    map.data.assign (map.info.width * map.info.height, 0);
    const std::vector<double> red_zone{4.4, 9.675, 4.4, 10.325, 4.85, 10.325, 4.85, 9.675};
    const std::vector<double> blue_zone{1.0, 9.675, 1.0, 10.325, 0.55, 10.325, 0.55, 9.675};
    Harness                   red_legacy ({}, true, {}, red_zone);
    Harness                   red_fixed ({}, true, {2.7}, red_zone);
    Harness                   blue_legacy ({}, true, {}, blue_zone);
    ASSERT_TRUE (red_legacy.load (map));
    ASSERT_TRUE (red_fixed.load (map));
    ASSERT_TRUE (blue_legacy.load (map));
    size_t legacy_mismatches = 0;
    for (size_t y = 0; y < map.info.height; ++y) {
        for (size_t x = 0; x < 108; ++x) {
            const size_t red_index  = y * map.info.width + x;
            const size_t blue_index = y * map.info.width + 107 - x;
            if (red_legacy.costmap.data[red_index] != blue_legacy.costmap.data[blue_index]) ++legacy_mismatches;
            EXPECT_EQ (red_fixed.costmap.data[red_index], blue_legacy.costmap.data[blue_index]) << x << ", " << y;
        }
    }
    EXPECT_GT (legacy_mismatches, 0u);  // Reproduce the original one-cell error.
    EXPECT_EQ (red_fixed.cost (4.425, 10.025), 100);
    EXPECT_EQ (red_fixed.cost (4.625, 10.025), 100);
    EXPECT_EQ (red_fixed.cost (4.825, 10.025), 100);
    ASSERT_TRUE (red_fixed.plan (4.625, 10.025, 9.0));
    EXPECT_FALSE (red_fixed.paths.back ().valid);
    EXPECT_EQ (red_fixed.paths.back ().reason, "start or destination is blocked");
}

TEST_F (StartArea, ReflectedKeepoutPreservesWallsAndFullKeepoutInflation) {
    Harness walls;
    Harness reflected_walls ({}, false, {2.7});
    ASSERT_TRUE (walls.load (wall_map ()));
    ASSERT_TRUE (reflected_walls.load (wall_map ()));
    EXPECT_EQ (walls.costmap.data, reflected_walls.costmap.data);

    Harness local ({3.4, 4.4, 9.5, 10.5}, true, {2.7});
    ASSERT_TRUE (local.load (wall_map ()));
    EXPECT_EQ (local.cost (4.125, 10.025), 100);  // Keepout margin stays blocked.
    EXPECT_EQ (local.cost (4.425, 10.525), 100);  // Wall / keepout overlap.
    EXPECT_EQ (local.cost (3.9, 10.525), 100);    // Unmodified wall.
    ASSERT_TRUE (local.plan (3.9, 10.3, 9.0));
    EXPECT_FALSE (local.paths.back ().valid);
    EXPECT_EQ (local.paths.back ().reason, "start or destination is blocked");
}

TEST_F (StartArea, InvalidKeepoutReflectionIsRejected) {
    for (const auto &axis : std::vector<std::vector<double>>{
             {2.7, 3.0},
             {std::numeric_limits<double>::infinity ()},
             {std::numeric_limits<double>::quiet_NaN ()}
    }) {
        rclcpp::NodeOptions options;
        options.append_parameter_override ("keepout_grid_reflection_x", axis);
        EXPECT_THROW (std::make_shared<Planner> (options), std::invalid_argument);
    }
}

TEST_F (StartArea, RedYawGridMirrorsLegacyBlueAndPreservesBoundaryEndpoints) {
    auto map = wall_map ();
    map.data.assign (map.info.width * map.info.height, 0);
    for (double start_yaw : {90.0, 92.5}) {
        Harness red ({}, false, {}, {}, true);
        Harness blue;
        ASSERT_TRUE (red.load (map));
        ASSERT_TRUE (blue.load (map));
        ASSERT_TRUE (red.plan (3.77, 9.89, 7.77, 4.58, start_yaw, 154.0));
        ASSERT_TRUE (blue.plan (1.63, 9.89, 7.77, 0.82, 180.0 - start_yaw, 26.0));
        ASSERT_TRUE (red.paths.back ().valid) << red.paths.back ().reason;
        ASSERT_TRUE (blue.paths.back ().valid) << blue.paths.back ().reason;
        const auto &red_poses  = red.paths.back ().path.poses;
        const auto &blue_poses = blue.paths.back ().path.poses;
        EXPECT_NEAR (tf2::getYaw (red_poses.front ().pose.orientation), start_yaw * M_PI / 180.0, 1.0e-8);
        EXPECT_NEAR (tf2::getYaw (red_poses.back ().pose.orientation), 154.0 * M_PI / 180.0, 1.0e-8);
        if (start_yaw == 90.0) {
            ASSERT_EQ (red_poses.size (), blue_poses.size ());
            for (size_t i = 0; i < red_poses.size (); ++i) {
                const double red_yaw  = tf2::getYaw (red_poses[i].pose.orientation);
                const double blue_yaw = tf2::getYaw (blue_poses[i].pose.orientation);
                EXPECT_NEAR (std::remainder (red_yaw + blue_yaw - M_PI, 2.0 * M_PI), 0.0, 1.0e-8) << i;
            }
        } else {
            // Independently constructed mirrored quaternions can lie on
            // opposite sides of the half-bin tie by one ULP. Preserve the
            // existing blue quantizer; bound the ambiguity to a single bin.
            EXPECT_LE (std::abs (static_cast<int> (red_poses.size ()) - static_cast<int> (blue_poses.size ())), 1);
            auto sample_yaw = [] (const auto &poses, double fraction) {
                const double index = fraction * static_cast<double> (poses.size () - 1);
                const size_t i     = static_cast<size_t> (index);
                const size_t j     = std::min (i + 1, poses.size () - 1);
                const double a     = tf2::getYaw (poses[i].pose.orientation);
                const double b     = tf2::getYaw (poses[j].pose.orientation);
                return a + (index - static_cast<double> (i)) * std::remainder (b - a, 2.0 * M_PI);
            };
            for (int i = 0; i <= 100; ++i) {
                const double fraction = static_cast<double> (i) / 100.0;
                EXPECT_LE (std::abs (std::remainder (sample_yaw (red_poses, fraction) + sample_yaw (blue_poses, fraction) - M_PI, 2.0 * M_PI)), 5.0 * M_PI / 180.0 + 1.0e-8) << i;
            }
        }
    }
}

TEST_F (StartArea, ReflectedStartOnGridBoundaryMatchesLegacyBlueRotation) {
    const std::vector<double> red_zone{4.4, 9.675, 4.4, 10.325, 4.85, 10.325, 4.85, 9.675};
    const std::vector<double> blue_zone{1.0, 9.675, 1.0, 10.325, 0.55, 10.325, 0.55, 9.675};
    Harness                   red ({3.4, 4.4, 9.5, 10.5}, true, {2.7}, red_zone, true);
    Harness                   blue ({1.0, 2.0, 9.5, 10.5}, true, {}, blue_zone);
    ASSERT_TRUE (red.load (wall_map ()));
    ASSERT_TRUE (blue.load (wall_map ()));
    ASSERT_TRUE (red.plan (3.9, 10.0, 7.77, 4.58, 0.0, 154.0));
    ASSERT_TRUE (blue.plan (1.5, 10.0, 7.77, 0.82, 180.0, 26.0));
    ASSERT_TRUE (red.paths.back ().valid) << red.paths.back ().reason;
    ASSERT_TRUE (blue.paths.back ().valid) << blue.paths.back ().reason;
    const auto &red_poses  = red.paths.back ().path.poses;
    const auto &blue_poses = blue.paths.back ().path.poses;
    ASSERT_EQ (red_poses.size (), blue_poses.size ());
    for (size_t i = 0; i < red_poses.size (); ++i) {
        const double red_yaw  = tf2::getYaw (red_poses[i].pose.orientation);
        const double blue_yaw = tf2::getYaw (blue_poses[i].pose.orientation);
        EXPECT_NEAR (std::remainder (red_yaw + blue_yaw - M_PI, 2.0 * M_PI), 0.0, 1.0e-8) << i;
    }
}

}  // namespace
