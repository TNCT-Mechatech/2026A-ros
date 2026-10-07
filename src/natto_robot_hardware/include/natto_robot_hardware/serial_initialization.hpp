#pragma once

#include "serial_frames.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <limits>
#include <vector>

namespace natto_serial {
// Defaults are the Encoder constructors in STM main.cpp, not robot calibration.
// All settings are explicit ROS parameters so each board can override them.
inline void declare_initialization(rclcpp::Node &node, SerialFrames &frames,
                                   uint8_t pid_state, bool stop_mode, double max_rps,
                                   const std::vector<int64_t> &default_polarities) {
    const auto ppr = node.declare_parameter<std::vector<int64_t>>("encoder_ppr", {2048, 2048, 2048, 2048});
    const auto gear = node.declare_parameter<std::vector<double>>("gear_rate", {71.0, 245.0/6.0, 245.0/6.0, 245.0/6.0});
    const auto polarity = node.has_parameter("encoder_polarities") ?
        node.get_parameter("encoder_polarities").as_integer_array() :
        node.declare_parameter<std::vector<int64_t>>("encoder_polarities", default_polarities);
    const auto maximum = node.declare_parameter<std::vector<double>>("max_motor_rps", std::vector<double>(MOTORS, max_rps));
    const auto pos_kp = node.declare_parameter<std::vector<double>>("pos_kp", {0.0, 0.0, 0.0, 0.0});
    const auto pos_ki = node.declare_parameter<std::vector<double>>("pos_ki", {0.0, 0.0, 0.0, 0.0});
    const auto pos_kd = node.declare_parameter<std::vector<double>>("pos_kd", {0.0, 0.0, 0.0, 0.0});
    const auto reset = node.declare_parameter<std::vector<bool>>("reset_encoder_origin", {false, false, false, false});
    const auto reset_switch = node.declare_parameter<std::vector<bool>>("reset_encoder_origin_with_switch", {false, false, false, false});
    const auto switch_state = node.declare_parameter<std::vector<bool>>("encoder_origin_switch_state", {true, true, true, true});
    if (ppr.size()!=MOTORS || gear.size()!=MOTORS || polarity.size()!=MOTORS || maximum.size()!=MOTORS ||
        pos_kp.size()!=MOTORS || pos_ki.size()!=MOTORS || pos_kd.size()!=MOTORS || reset.size()!=MOTORS ||
        reset_switch.size()!=MOTORS || switch_state.size()!=MOTORS)
        throw std::invalid_argument("STM initialization arrays must each contain four values");
    const auto finite_float = [](double value) {
        return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    };
    for (size_t i=0; i<MOTORS; ++i) {
        if (ppr[i]<1 || ppr[i]>65535 || !finite_float(gear[i]) || gear[i]<=0 ||
            !finite_float(maximum[i]) || maximum[i]<=0 || (polarity[i]!=1 && polarity[i]!=-1) ||
            !finite_float(pos_kp[i]) || !finite_float(pos_ki[i]) || !finite_float(pos_kd[i]))
            throw std::invalid_argument("invalid STM encoder, speed limit or position PID setting");
        auto &packet = frames.init[i].data;
        packet.reset_encoder_origin = reset[i];
        packet.reset_encoder_origin_with_switch = reset_switch[i];
        packet.encoder_origin_switch_state = switch_state[i];
        packet.pos_kp = pos_kp[i]; packet.pos_ki = pos_ki[i]; packet.pos_kd = pos_kd[i];
        packet.set_direction = polarity[i] < 0;
        packet.stop_mode = stop_mode;
        packet.pid_state = pid_state;
        packet.max_motor_rps = maximum[i];
        packet.encoder_ppr = ppr[i]; packet.gear_rate = gear[i];
    }
}
}  // namespace natto_serial
