#pragma once

#include <cstdint>

struct MotorCommandPacket {
    float target_wheel_rps[4];
};

struct MotorFeedbackPacket {
    float measured_wheel_rps[4];
    float output[4] float current_rps[4] float encoder_count[4] float duty[4] bool dir[4]
};