#ifndef __JOY_TO_TWIST_HPP__
#define __JOY_TO_TWIST_HPP__

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <cstdint>

namespace joy_to_twist {
class joy_to_twist : public rclcpp::Node {
   public:
    joy_to_twist (const rclcpp::NodeOptions &node_options);

   private:
    double  max_xy_speed_m_s_;
    double  max_yaw_speed_rad_s_;
    int64_t axis_linear_x_;
    int64_t axis_linear_y_;
    int64_t axis_yaw_;
    double  joy_deadzone_;

    void   joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg);
    double get_axis (const sensor_msgs::msg::Joy &msg, int64_t axis) const;
    double apply_deadzone (double value) const;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         joy_subscriber_;
};
}  // namespace joy_to_twist

#endif  // __JOY_TO_TWIST_HPP__
