// Copyright 2025 Kazusa Hashimoto
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

#include "natto_holonomic_pure_pursuit/holonomic_pure_pursuit.hpp"

#include <stdexcept>

namespace holonomic_pure_pursuit {

holonomic_pure_pursuit::holonomic_pure_pursuit (const rclcpp::NodeOptions &options) : Node ("holonomic_pure_pursuit", options) {
    managed_navigation_ = this->declare_parameter<bool> ("managed_navigation", false);
    rcl_interfaces::msg::ParameterDescriptor debug_descriptor;
    debug_descriptor.read_only = true;
    if (this->declare_parameter<bool> ("publish_debug", false, debug_descriptor)) {
        debug_publisher_ = this->create_publisher<natto_msgs::msg::PursuitDebug> ("pursuit_debug", rclcpp::SensorDataQoS ());
    }
    if (!managed_navigation_) cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped> ("command_velocity", 1);
    lookahead_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("lookahead", 1);
    if (!managed_navigation_) goal_reached_publisher_ = this->create_publisher<std_msgs::msg::Bool> ("goal_reached", 1);
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped> ("current_pose", 1, std::bind (&holonomic_pure_pursuit::pose_callback, this, std::placeholders::_1));
    if (managed_navigation_) {
        navigation_feedback_publisher_ = this->create_publisher<natto_msgs::msg::NavigationFeedback> ("navigation_feedback", 10);
        navigation_path_subscriber_    = this->create_subscription<natto_msgs::msg::NavigationPath> ("navigation_path", 10, std::bind (&holonomic_pure_pursuit::navigation_path_callback, this, std::placeholders::_1));
    } else {
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path> ("path", 1, std::bind (&holonomic_pure_pursuit::path_callback, this, std::placeholders::_1));
    }

    double frequency = this->declare_parameter<double> ("frequency", 50.0);
    delta_t_s_       = 1.0 / frequency;

    timer_ = this->create_wall_timer (std::chrono::milliseconds (static_cast<int> (1000.0 / frequency)), std::bind (&holonomic_pure_pursuit::timer_callback, this));

    lookahead_time_                 = this->declare_parameter<double> ("lookahead_time_s", 1.0);
    min_lookahead_distance_         = this->declare_parameter<double> ("min_lookahead_distance_m", 0.1);
    max_lookahead_distance_         = this->declare_parameter<double> ("max_lookahead_distance_m", 1.0);
    yaw_speed_p_                    = this->declare_parameter<double> ("yaw_speed_p", 1.0);
    curvature_lookahead_distance_   = this->declare_parameter<double> ("curvature_lookahead_distance_m", 0.3);
    if (!std::isfinite (curvature_lookahead_distance_) || curvature_lookahead_distance_ <= 0.0) {
        throw std::invalid_argument ("curvature_lookahead_distance_m must be finite and positive");
    }
    curvature_deceleration_p_       = this->declare_parameter<double> ("curvature_deceleration_p", 1.0);
    min_curvature_speed_m_s_        = this->declare_parameter<double> ("min_curvature_speed_m_s", 0.3);
    yaw_deceleration_p_             = this->declare_parameter<double> ("yaw_deceleration_p", 1.0);
    max_speed_xy_m_s_               = this->declare_parameter<double> ("max_speed_xy_m_s", 3.0);
    min_speed_xy_m_s_               = this->declare_parameter<double> ("min_speed_xy_m_s", 0.1);
    max_speed_yaw_deg_s_            = this->declare_parameter<double> ("max_speed_yaw_deg_s", 180.0);
    min_speed_yaw_deg_s_            = this->declare_parameter<double> ("min_speed_yaw_deg_s", 18.0);
    max_acceleration_xy_m_s2_       = this->declare_parameter<double> ("max_acceleration_xy_m_s2", 10.0);
    max_acceleration_yaw_deg_s2_    = this->declare_parameter<double> ("max_acceleration_yaw_deg_s2", 500.0);
    goal_deceleration_m_s2_         = this->declare_parameter<double> ("goal_deceleration_m_s2", 4.0);
    goal_deceleration_distance_p_   = this->declare_parameter<double> ("goal_deceleration_distance_p", 1.0);
    goal_position_tolerance_        = this->declare_parameter<double> ("goal_position_tolerance_m", 0.03);
    goal_yaw_tolerance_deg_         = this->declare_parameter<double> ("goal_yaw_tolerance_deg", 10.0);
    goal_speed_tolerance_xy_m_s_    = this->declare_parameter<double> ("goal_speed_tolerance_xy_m_s", 0.3);
    goal_speed_tolerance_yaw_deg_s_ = this->declare_parameter<double> ("goal_speed_tolerance_yaw_deg_s", 30.0);

    RCLCPP_INFO (this->get_logger (), "holonomic_pure_pursuit node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "lookahead_time_s : %f", lookahead_time_);
    RCLCPP_INFO (this->get_logger (), "min_lookahead_distance_m : %f", min_lookahead_distance_);
    RCLCPP_INFO (this->get_logger (), "max_lookahead_distance_m : %f", max_lookahead_distance_);
    RCLCPP_INFO (this->get_logger (), "yaw_speed_p : %f", yaw_speed_p_);
    RCLCPP_INFO (this->get_logger (), "curvature_lookahead_distance_m : %f", curvature_lookahead_distance_);
    RCLCPP_INFO (this->get_logger (), "curvature_deceleration_p : %f", curvature_deceleration_p_);
    RCLCPP_INFO (this->get_logger (), "min_curvature_speed_m_s : %f", min_curvature_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "yaw_deceleration_p : %f", yaw_deceleration_p_);
    RCLCPP_INFO (this->get_logger (), "max_speed_xy_m_s : %f", max_speed_xy_m_s_);
    RCLCPP_INFO (this->get_logger (), "min_speed_xy_m_s : %f", min_speed_xy_m_s_);
    RCLCPP_INFO (this->get_logger (), "max_speed_yaw_deg_s : %f", max_speed_yaw_deg_s_);
    RCLCPP_INFO (this->get_logger (), "min_speed_yaw_deg_s : %f", min_speed_yaw_deg_s_);
    RCLCPP_INFO (this->get_logger (), "max_acceleration_xy_m_s2_ : %f", max_acceleration_xy_m_s2_);
    RCLCPP_INFO (this->get_logger (), "max_acceleration_yaw_deg_s2 : %f", max_acceleration_yaw_deg_s2_);
    RCLCPP_INFO (this->get_logger (), "goal_deceleration_m_s2 : %f", goal_deceleration_m_s2_);
    RCLCPP_INFO (this->get_logger (), "goal_position_tolerance_m : %f", goal_position_tolerance_);
    RCLCPP_INFO (this->get_logger (), "goal_yaw_tolerance_deg : %f", goal_yaw_tolerance_deg_);
    RCLCPP_INFO (this->get_logger (), "goal_speed_tolerance_xy_m_s : %f", goal_speed_tolerance_xy_m_s_);
    RCLCPP_INFO (this->get_logger (), "goal_speed_tolerance_yaw_deg_s : %f", goal_speed_tolerance_yaw_deg_s_);
}

void holonomic_pure_pursuit::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock (data_mutex_);
    current_pose_ = msg;
}

void holonomic_pure_pursuit::navigation_path_callback (const natto_msgs::msg::NavigationPath::SharedPtr msg) {
    std::lock_guard<std::mutex> lock (data_mutex_);
    if (msg->request_id != active_request_id_ || !msg->valid) last_cmd_vel_.reset ();
    active_request_id_ = msg->valid ? msg->request_id : "";
    path_              = std::make_shared<nav_msgs::msg::Path> (msg->path);
    if (!msg->valid) path_->poses.clear ();
    update_path_geometry ();
}

void holonomic_pure_pursuit::path_callback (const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock (data_mutex_);
    path_ = msg;
    update_path_geometry ();
}

void holonomic_pure_pursuit::update_path_geometry () {
    std::vector<path_geometry::Point> points;
    points.reserve (path_->poses.size ());
    for (const auto &pose : path_->poses) {
        points.push_back ({pose.pose.position.x, pose.pose.position.y, tf2::getYaw (pose.pose.orientation)});
    }
    path_geometry_ = path_geometry::PathGeometry (std::move (points));
}

void holonomic_pure_pursuit::timer_callback () {
    std::lock_guard<std::mutex> lock (data_mutex_);

    if (!current_pose_) {
        RCLCPP_DEBUG (this->get_logger (), "Current pose not received yet.");
        return;
    }

    if (!path_) {
        RCLCPP_DEBUG (this->get_logger (), "Path not received yet.");
        return;
    }

    if (!last_cmd_vel_) {
        last_cmd_vel_                  = std::make_shared<geometry_msgs::msg::TwistStamped> ();
        last_cmd_vel_->header.stamp    = this->now ();
        last_cmd_vel_->header.frame_id = "base_link";
        last_cmd_vel_->twist.linear.x  = 0.0;
        last_cmd_vel_->twist.linear.y  = 0.0;
        last_cmd_vel_->twist.angular.z = 0.0;
    }

    if (path_->poses.empty () || path_->poses.size () < 2) {
        RCLCPP_DEBUG (this->get_logger (), "Path is empty or has less than 2 poses.");
        return;
    }
    double position_error        = std::hypot (path_->poses.back ().pose.position.x - current_pose_->pose.position.x, path_->poses.back ().pose.position.y - current_pose_->pose.position.y);
    bool   goal_position_reached = (position_error < goal_position_tolerance_);

    double yaw_error = tf2::getYaw (path_->poses.back ().pose.orientation) - tf2::getYaw (current_pose_->pose.orientation);
    while (yaw_error > +M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
    yaw_error             = std::abs (yaw_error);
    bool goal_yaw_reached = (yaw_error < goal_yaw_tolerance_deg_ * M_PI / 180.0);

    double current_speed_xy       = std::hypot (last_cmd_vel_->twist.linear.x, last_cmd_vel_->twist.linear.y);
    bool   goal_speed_xy_reached  = (current_speed_xy < goal_speed_tolerance_xy_m_s_);
    double current_speed_yaw      = std::abs (last_cmd_vel_->twist.angular.z);
    bool   goal_speed_yaw_reached = (current_speed_yaw < goal_speed_tolerance_yaw_deg_s_ * M_PI / 180.0);

    size_t closest_index = 0;
    double min_distance  = std::numeric_limits<double>::max ();
    for (size_t i = 0; i < path_->poses.size (); i++) {
        double dist = std::hypot (path_->poses[i].pose.position.x - current_pose_->pose.position.x, path_->poses[i].pose.position.y - current_pose_->pose.position.y);
        if (dist < min_distance) {
            min_distance  = dist;
            closest_index = i;
        }
    }

    if (closest_index + 1 >= path_->poses.size ()) closest_index = path_->poses.size () - 2;

    geometry_msgs::msg::Pose goal_pose     = path_->poses.back ().pose;
    double                   goal_distance = std::hypot (goal_pose.position.x - current_pose_->pose.position.x, goal_pose.position.y - current_pose_->pose.position.y);

    double last_speed      = std::hypot (last_cmd_vel_->twist.linear.x, last_cmd_vel_->twist.linear.y);
    double predicted_speed = last_speed + max_acceleration_xy_m_s2_ * delta_t_s_;
    predicted_speed        = std::clamp (predicted_speed, 0.0, max_speed_xy_m_s_);
    lookahead_distance_    = std::clamp (lookahead_time_ * predicted_speed, min_lookahead_distance_, max_lookahead_distance_);

    const auto lookahead = path_geometry_.lookahead (current_pose_->pose.position.x, current_pose_->pose.position.y, closest_index, lookahead_distance_);
    double current_yaw   = tf2::getYaw (current_pose_->pose.orientation);
    double lookahead_x   = lookahead.x;
    double lookahead_y   = lookahead.y;
    double lookahead_yaw = lookahead.yaw;

    double dx = lookahead_x - current_pose_->pose.position.x;
    double dy = lookahead_y - current_pose_->pose.position.y;

    double angle_diff   = std::atan2 (dy, dx) - current_yaw;
    double target_speed = std::hypot (dx, dy) / lookahead_time_;
    target_speed        = std::clamp (target_speed, 0.0, max_speed_xy_m_s_);

    double d = std::max (goal_distance, 0.0);

    double max_stop_speed = std::sqrt (2.0 * std::min (goal_deceleration_m_s2_, max_acceleration_xy_m_s2_) * std::max (0.0, d / goal_deceleration_distance_p_));
    target_speed          = std::min (target_speed, max_stop_speed);

    const double curvature = path_geometry_.curvature (current_pose_->pose.position.x, current_pose_->pose.position.y, curvature_lookahead_distance_);

    double curvature_speed = target_speed / (std::abs (curvature * curvature_deceleration_p_) + 1e-6);

    target_speed = std::min (target_speed, std::max (curvature_speed, min_curvature_speed_m_s_));

    double yaw_diff = lookahead_yaw - current_yaw;
    while (yaw_diff > +M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    double yaw_speed = yaw_diff / lookahead_time_ * yaw_speed_p_;

    double required_yaw         = std::abs (yaw_diff);
    double max_yaw_change       = std::abs (last_cmd_vel_->twist.angular.z) * lookahead_time_ + 0.5 * (max_acceleration_yaw_deg_s2_ * M_PI / 180.0) * lookahead_time_ * lookahead_time_;
    double max_yaw_change_limit = (max_speed_yaw_deg_s_ * M_PI / 180.0) * lookahead_time_;
    max_yaw_change              = std::min (max_yaw_change, max_yaw_change_limit);

    if (required_yaw > max_yaw_change) {
        double ratio = max_yaw_change / required_yaw;
        ratio        = std::clamp (ratio / yaw_deceleration_p_, 0.0, 1.0);
        target_speed *= ratio;
    }

    target_speed = std::clamp (target_speed, min_speed_xy_m_s_, max_speed_xy_m_s_);

    double acceleration = (target_speed - last_speed) / delta_t_s_;
    acceleration        = std::clamp (acceleration, -max_acceleration_xy_m_s2_, max_acceleration_xy_m_s2_);
    double speed        = last_speed + acceleration * delta_t_s_;

    double yaw_acceleration = (yaw_speed - last_cmd_vel_->twist.angular.z) / delta_t_s_;
    yaw_acceleration        = std::clamp (yaw_acceleration, -max_acceleration_yaw_deg_s2_ * M_PI / 180.0, max_acceleration_yaw_deg_s2_ * M_PI / 180.0);
    yaw_speed               = last_cmd_vel_->twist.angular.z + yaw_acceleration * delta_t_s_;
    yaw_speed               = std::clamp (yaw_speed, -max_speed_yaw_deg_s_ * M_PI / 180.0, max_speed_yaw_deg_s_ * M_PI / 180.0);

    if (goal_position_reached && goal_speed_xy_reached) {
        speed = 0.0;
    }

    if (goal_yaw_reached && goal_speed_yaw_reached) {
        yaw_speed = 0.0;
    }

    std_msgs::msg::Bool goal_reached_msg;
    goal_reached_msg.data = goal_position_reached && goal_speed_xy_reached && goal_yaw_reached && goal_speed_yaw_reached;
    if (!managed_navigation_) goal_reached_publisher_->publish (goal_reached_msg);

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp    = this->now ();
    cmd_vel.header.frame_id = "command/base_link";
    cmd_vel.twist.linear.x  = speed * std::cos (angle_diff);
    cmd_vel.twist.linear.y  = speed * std::sin (angle_diff);
    cmd_vel.twist.angular.z = yaw_speed;
    if (managed_navigation_) {
        natto_msgs::msg::NavigationFeedback feedback;
        feedback.request_id = active_request_id_;
        feedback.command    = cmd_vel;
        feedback.reached    = goal_reached_msg.data;
        navigation_feedback_publisher_->publish (feedback);
    } else {
        cmd_vel_publisher_->publish (cmd_vel);
    }

    // Copy existing calculations only; this output never feeds the controller.
    if (debug_publisher_) {
        natto_msgs::msg::PursuitDebug debug;
        debug.header         = cmd_vel.header;
        debug.request_id     = active_request_id_;
        debug.path_error     = min_distance;
        debug.goal_distance  = goal_distance;
        debug.goal_yaw_error = yaw_error;
        debug.target_speed   = target_speed;
        debug_publisher_->publish (debug);
    }

    geometry_msgs::msg::PoseStamped lookahead_msg;
    lookahead_msg.header.stamp    = this->now ();
    lookahead_msg.header.frame_id = "map";
    lookahead_msg.pose.position.x = lookahead_x;
    lookahead_msg.pose.position.y = lookahead_y;
    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, lookahead_yaw);
    lookahead_msg.pose.orientation = tf2::toMsg (q);
    lookahead_publisher_->publish (lookahead_msg);
    last_cmd_vel_ = std::make_shared<geometry_msgs::msg::TwistStamped> (cmd_vel);
}
}  // namespace holonomic_pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (holonomic_pure_pursuit::holonomic_pure_pursuit)
