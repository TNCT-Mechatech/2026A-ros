#include <iostream>
#include <array>

#include "../vrft_auto_tuner.hpp"
#include "../plant.hpp"
#include "../PID_v1.hpp"

int main() {
    const float dt = 0.01f;      // 10ms
    const int steps = 1000;      // 10秒
    const float target = 1.0f;

    FirstOrderPlant plant(1.0f, 0.5f, dt);

    float input = 0.0f;
    float output = 0.0f;
    float setpoint = target;

    float rt;

    PID pid(&input, &output, &setpoint, 2.0f, 1.0f, 0.01f, DIRECT);
    pid.SetOutputLimits(-10.0f, 10.0f);
    pid.SetSampleTime(static_cast<int>(dt * 1000));
    pid.SetMode(AUTOMATIC);
    pid.SetTunings(2.0f, 1.0f, 0.01f);

    std::cout << "responseTime = ";
    std::cin >> rt;

    VRFT vrft;
    vrft.timeStep(dt);
    vrft.responseTime(rt);

    for (int k = 0; k < steps; k++) {
        input = plant.output();

        pid.Compute();

        float y = plant.update(output);

        vrft.update(output, y);

        if (k % 50 == 0) {
            std::cout
                << "t=" << k * dt
                << ", u=" << output
                << ", y=" << y
                << std::endl;
        }
    }

    std::array<float, 3> gain = vrft.getPID();

    std::cout << "\nVRFT result" << std::endl;
    std::cout << "Kp = " << gain[0] << std::endl;
    std::cout << "Ki = " << gain[1] << std::endl;
    std::cout << "Kd = " << gain[2] << std::endl;



    return 0;
}