#pragma once

#include <cmath>

#define AUTOMATIC 1
#define MANUAL    0

#define DIRECT    0
#define REVERSE   1

class PID {
public:
    PID(float* input, float* output, float* setpoint,
        float Kp, float Ki, float Kd, int direction)
        : input_(input),
          output_(output),
          setpoint_(setpoint),
          kp_(Kp),
          ki_(Ki),
          kd_(Kd),
          controllerDirection_(direction),
          inAuto_(false),
          sampleTimeMs_(10),
          outMin_(0.0f),
          outMax_(255.0f),
          iTerm_(0.0f),
          lastInput_(0.0f) {
        if (controllerDirection_ == REVERSE) {
            kp_ = -kp_;
            ki_ = -ki_;
            kd_ = -kd_;
        }
    }

    bool Compute() {
        if (!inAuto_) return false;

        float input = *input_;
        float error = *setpoint_ - input;

        iTerm_ += ki_ * error;
        if (iTerm_ > outMax_) iTerm_ = outMax_;
        else if (iTerm_ < outMin_) iTerm_ = outMin_;

        float dInput = input - lastInput_;

        float output = kp_ * error + iTerm_ - kd_ * dInput;

        if (output > outMax_) output = outMax_;
        else if (output < outMin_) output = outMin_;

        *output_ = output;
        lastInput_ = input;

        return true;
    }

    void SetTunings(float Kp, float Ki, float Kd) {
        if (Kp < 0 || Ki < 0 || Kd < 0) return;

        float sampleTimeSec = sampleTimeMs_ / 1000.0f;

        kp_ = Kp;
        ki_ = Ki * sampleTimeSec;
        kd_ = Kd / sampleTimeSec;

        if (controllerDirection_ == REVERSE) {
            kp_ = -kp_;
            ki_ = -ki_;
            kd_ = -kd_;
        }
    }

    void SetSampleTime(int newSampleTimeMs) {
        if (newSampleTimeMs <= 0) return;

        float ratio = static_cast<float>(newSampleTimeMs) / sampleTimeMs_;

        ki_ *= ratio;
        kd_ /= ratio;

        sampleTimeMs_ = newSampleTimeMs;
    }

    void SetOutputLimits(float min, float max) {
        if (min >= max) return;

        outMin_ = min;
        outMax_ = max;

        if (*output_ > outMax_) *output_ = outMax_;
        else if (*output_ < outMin_) *output_ = outMin_;

        if (iTerm_ > outMax_) iTerm_ = outMax_;
        else if (iTerm_ < outMin_) iTerm_ = outMin_;
    }

    void SetMode(int mode) {
        bool newAuto = (mode == AUTOMATIC);

        if (newAuto && !inAuto_) {
            initialize();
        }

        inAuto_ = newAuto;
    }

private:
    void initialize() {
        iTerm_ = *output_;
        lastInput_ = *input_;

        if (iTerm_ > outMax_) iTerm_ = outMax_;
        else if (iTerm_ < outMin_) iTerm_ = outMin_;
    }

private:
    float* input_;
    float* output_;
    float* setpoint_;

    float kp_;
    float ki_;
    float kd_;

    int controllerDirection_;
    bool inAuto_;

    int sampleTimeMs_;

    float outMin_;
    float outMax_;

    float iTerm_;
    float lastInput_;
};