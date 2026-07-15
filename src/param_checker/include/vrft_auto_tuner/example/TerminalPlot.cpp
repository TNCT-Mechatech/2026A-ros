#include "../PID_v1.hpp"
#include "../plant.hpp"
#include "../terminal_plot.hpp"

int main () {
    const float dt     = 0.01f;
    const int   steps  = 500;
    const float target = 1.0f;

    FirstOrderPlant plant (1.0f, 0.5f, dt);

    float input    = 0.0f;
    float output   = 0.0f;
    float setpoint = target;

    float kp, ki, kd;

    std::cout << "Kp = ";
    std::cin >> kp;
    std::cout << "Ki = ";
    std::cin >> ki;
    std::cout << "Kd = ";
    std::cin >> kd;

    PID pid (&input, &output, &setpoint, kp, ki, kd, DIRECT);

    pid.SetOutputLimits (-10.0f, 10.0f);
    pid.SetSampleTime (static_cast<int> (dt * 1000));
    pid.SetMode (AUTOMATIC);
    pid.SetTunings (kp, ki, kd);

    TerminalPlot plot (0.0f, 1.5f, 60);

    for (int k = 0; k < steps; k++) {
        input = plant.output ();

        pid.Compute ();

        float y = plant.update (output);

        if (k % 5 == 0) {
            plot.plot (y, target, k * dt);
        }
    }

    return 0;
}