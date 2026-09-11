#pragma once

#include <cstdint>

struct MotorCommand {
    float target_wheel_rps;
    float kp;  // ゼロでok
    float ki;
    float kd;
    bool  set_direction;
    bool  pid_state;  // falseにする
    bool  stop_mode;
};

struct MotorFeedback {
    float measured_wheel_rps;
    float output;
    float encoder_count;
    float duty;
    bool  dir;
};

struct MotorCommandPacket {
    MotorCommand mc[4];
};

struct MotorFeedbackPacket {
    MotorFeedback mfd[4];
};