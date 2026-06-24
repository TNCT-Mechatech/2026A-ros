#include "natto_joy/joy_to_twist.hpp"

#include <cmath>

namespace joy_to_twist {

joy_to_twist::joy_to_twist (const rclcpp::NodeOptions &node_options) : Node ("joy_to_twist", node_options) {
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped> ("command_velocity", 1);
    joy_subscriber_  = this->create_subscription<sensor_msgs::msg::Joy> ("joy", 1, std::bind (&joy_to_twist::joy_callback, this, std::placeholders::_1));

    max_xy_speed_m_s_    = this->declare_parameter<double> ("max_xy_speed_m_s", 2.0);
    max_yaw_speed_rad_s_ = this->declare_parameter<double> ("max_yaw_speed_rad_s", 3.1415);
    axis_linear_x_       = this->declare_parameter<int> ("axis_linear_x", 1);
    axis_linear_y_       = this->declare_parameter<int> ("axis_linear_y", 0);
    axis_yaw_            = this->declare_parameter<int> ("axis_yaw", 2);
    joy_deadzone_        = this->declare_parameter<double> ("joy_deadzone", 0.0);

    RCLCPP_INFO (this->get_logger (), "joy_to_twist node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "max_xy_speed_m_s: %.2f", max_xy_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "max_yaw_speed_rad_s: %.4f", max_yaw_speed_rad_s_);
    RCLCPP_INFO (this->get_logger (), "axis_linear_x: %ld", axis_linear_x_);
    RCLCPP_INFO (this->get_logger (), "axis_linear_y: %ld", axis_linear_y_);
    RCLCPP_INFO (this->get_logger (), "axis_yaw: %ld", axis_yaw_);
    RCLCPP_INFO (this->get_logger (), "joy_deadzone: %.4f", joy_deadzone_);
}

void joy_to_twist::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp    = this->now ();
    twist_msg.header.frame_id = "command/base_link";
    twist_msg.twist.linear.x  = apply_deadzone (get_axis (*msg, axis_linear_x_)) * max_xy_speed_m_s_;
    twist_msg.twist.linear.y  = apply_deadzone (get_axis (*msg, axis_linear_y_)) * max_xy_speed_m_s_;
    twist_msg.twist.angular.z = apply_deadzone (get_axis (*msg, axis_yaw_)) * max_yaw_speed_rad_s_;
    twist_publisher_->publish (twist_msg);
}

double joy_to_twist::get_axis (const sensor_msgs::msg::Joy &msg, int64_t axis) const {
    if (axis < 0 || static_cast<size_t> (axis) >= msg.axes.size ()) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 3000, "Joy axis %ld is out of range. axes size: %zu", axis, msg.axes.size ());
        return 0.0;
    }
    return msg.axes[static_cast<size_t> (axis)];
}

double joy_to_twist::apply_deadzone (double value) const {
    return std::abs (value) < joy_deadzone_ ? 0.0 : value;
}

}  // namespace joy_to_twist

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (joy_to_twist::joy_to_twist)