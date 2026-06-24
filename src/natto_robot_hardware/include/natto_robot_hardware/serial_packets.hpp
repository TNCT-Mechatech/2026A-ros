#pragma once

#include <cstdint>

struct MotorCommandPacket {
    float target_wheel_rps[4];
};

struct MotorFeedbackPacket {
    float measured_wheel_rps[4];
};