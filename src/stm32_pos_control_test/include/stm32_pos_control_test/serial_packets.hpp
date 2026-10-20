#pragma once
#include <natto_robot_hardware/serial_frames.hpp>
namespace stm32_protocol
{
constexpr auto motor_count = natto_serial::MOTORS;
constexpr auto init_frame_id_base = natto_serial::INIT_FRAME_ID_BASE;
constexpr auto command_velocity_frame_id_base = natto_serial::CMD_VEL_FRAME_ID_BASE;
constexpr auto command_position_frame_id_base = natto_serial::CMD_POS_FRAME_ID_BASE;
constexpr auto command_duty_frame_id_base = natto_serial::CMD_DUTY_FRAME_ID_BASE;
constexpr auto feedback_frame_id_base = natto_serial::FEEDBACK_FRAME_ID_BASE;
using InitializationMessage = natto_serial::InitializationPacket;
using MotorCommandVelocityMessage = natto_serial::MotorCommandVelPacket;
using MotorCommandPositionMessage = natto_serial::MotorCommandPosPacket;
using MotorCommandDutyMessage = natto_serial::MotorCommandDutyPacket;
using MotorFeedbackMessage = natto_serial::MotorFeedbackPacket;
}  // namespace stm32_protocol
