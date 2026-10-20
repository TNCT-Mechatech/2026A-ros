#pragma once

#include <cstddef>
#include <cstdint>
#include <Message.hpp>

// ABI matches robocon2026_A_STM f12724ad316fd694e2bae334d4d9dada09359242.
// Native little-endian, natural alignment; never pack these structures.
namespace natto_serial {
inline constexpr size_t MOTORS = 4;
inline constexpr uint8_t INIT_FRAME_ID_BASE = 0;
inline constexpr uint8_t CMD_VEL_FRAME_ID_BASE = 4;
inline constexpr uint8_t CMD_POS_FRAME_ID_BASE = 8;
inline constexpr uint8_t CMD_DUTY_FRAME_ID_BASE = 12;
inline constexpr uint8_t FEEDBACK_FRAME_ID_BASE = 16;
inline constexpr uint8_t SWITCH_FEEDBACK_FRAME_ID = 20;
inline constexpr uint8_t INIT_ACK_FRAME_ID_BASE = 21;
inline constexpr int COMM_TIMEOUT_MS = 300;

typedef struct InitializationPackets{
//  true: Reset the encoder origin and PID states when applying this configuration.
	bool reset_encoder_origin;
//  true: Reset the encoder origin when the configured switch condition is met.
	bool reset_encoder_origin_with_switch;
//  Switch level used as the encoder origin reset condition.
	bool encoder_origin_switch_state;

//  In addition to being a PID gain for rotational speed control,
//	it also serves as a PID gain that controls the maximum speed during position control.
//	The two values can be the same.
	float vel_kp;
	float vel_ki;
	float vel_kd;

//	PID Gains for Position and Angle Control
	float pos_kp;
	float pos_ki;
	float pos_kd;

    bool set_direction;
    bool stop_mode;
    // 0: off, 1: velocity control, 2: position control
    uint8_t pid_state;
    float max_motor_rps;

//  Encoder settings used to calculate CPR.
//  cpr = encoder_ppr * 4 * gear_rate
    uint16_t encoder_ppr;
    float gear_rate;
    uint32_t session_id;
    uint32_t config_id;
}initialization_packet_t;

enum class InitializationResult : uint8_t {
    Applied = 0, AlreadyApplied = 1, Invalid = 2, Stale = 3, IdConflict = 4
};

typedef struct InitializationAck {
    uint32_t session_id;
    uint32_t config_id;
    uint32_t applied_session_id;
    uint32_t applied_config_id;
    InitializationResult result;
    uint8_t reserved[3];
}initialization_ack_packet_t;

typedef struct MotorCommandVel{
    float target_motor_rps;
}motor_command_vel_packet_t;

typedef struct MotorCommandPos{
    int64_t target_pos;
}motor_command_pos_packet_t;

typedef struct MotorCommandDuty{
	float duty; // signed duty from -1.0f to 1.0f
}motor_command_duty_packet_t;

typedef struct MotorFeedback{
    float measured_wheel_rps;
    float output;
    int32_t encoder_count;
    int64_t encoder_raw_count;
    float duty;
    bool dir;
    uint32_t applied_session_id;
    uint32_t applied_config_id;
}motor_feedback_packet_t;

typedef struct SwitchFeedback{
    bool state[MOTORS];
}switch_feedback_packet_t;


typedef sb::Message<initialization_packet_t> InitializationPacket;
typedef sb::Message<initialization_ack_packet_t> InitializationAckPacket;
typedef sb::Message<motor_command_vel_packet_t> MotorCommandVelPacket;
typedef sb::Message<motor_command_pos_packet_t> MotorCommandPosPacket;
typedef sb::Message<motor_command_duty_packet_t> MotorCommandDutyPacket;
typedef sb::Message<motor_feedback_packet_t> MotorFeedbackPacket;
typedef sb::Message<switch_feedback_packet_t> SwitchFeedbackPacket;

static_assert(sizeof(InitializationPackets) == 52, "InitializationPackets must stay 52 bytes");
static_assert(offsetof(InitializationPackets, session_id) == 44);
static_assert(offsetof(InitializationPackets, config_id) == 48);
static_assert(sizeof(InitializationAck) == 20, "InitializationAck must stay 20 bytes");
static_assert(offsetof(InitializationAck, result) == 16);
static_assert(sizeof(MotorCommandVel) == 4, "MotorCommandVel must stay 4 bytes");
static_assert(sizeof(MotorCommandPos) == 8, "MotorCommandPos must stay 8 bytes");
static_assert(sizeof(MotorCommandDuty) == 4, "MotorCommandDuty must stay 4 bytes");
static_assert(sizeof(MotorFeedback) == 40, "MotorFeedback must stay 40 bytes");
static_assert(offsetof(MotorFeedback, applied_session_id) == 32);
static_assert(offsetof(MotorFeedback, applied_config_id) == 36);
static_assert(sizeof(SwitchFeedback) == 4, "SwitchFeedback must stay 4 bytes");

static_assert(sizeof(bool) == 1 && sizeof(float) == 4);
static_assert(offsetof(InitializationPackets, vel_kp) == 4);
static_assert(offsetof(InitializationPackets, pos_kp) == 16);
static_assert(offsetof(InitializationPackets, set_direction) == 28);
static_assert(offsetof(InitializationPackets, pid_state) == 30);
static_assert(offsetof(InitializationPackets, max_motor_rps) == 32);
static_assert(offsetof(InitializationPackets, encoder_ppr) == 36);
static_assert(offsetof(InitializationPackets, gear_rate) == 40);
static_assert(offsetof(MotorFeedback, encoder_count) == 8);
static_assert(offsetof(MotorFeedback, encoder_raw_count) == 16);
static_assert(offsetof(MotorFeedback, duty) == 24);
static_assert(offsetof(MotorFeedback, dir) == 28);
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error STM serial protocol requires a little-endian host
#endif
}  // namespace natto_serial
