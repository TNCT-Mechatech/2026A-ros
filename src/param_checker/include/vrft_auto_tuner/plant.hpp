#pragma once

class FirstOrderPlant {
   public:
    FirstOrderPlant (float K, float T, float dt) : K_ (K), T_ (T), dt_ (dt), y_ (0.0f) {}

    float update (float u) {
        // dy/dt = (-y + K*u) / T
        float dydt = (-y_ + K_ * u) / T_;
        y_ += dydt * dt_;
        return y_;
    }

    float output () const {
        return y_;
    }

    void reset (float y0 = 0.0f) {
        y_ = y0;
    }

   private:
    float K_;   // ゲイン
    float T_;   // 時定数
    float dt_;  // 制御周期
    float y_;   // 出力
};