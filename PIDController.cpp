#include "PIDController.hpp"

PIDController::PIDController(float p, float i, float d, float smoothing)
    : Kp(p), Ki(i), Kd(d), integral(0), prevMeasured(0), lastDerivative(0), alpha(smoothing) {}

float PIDController::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    integral += error * dt;

    float rawDerivative = -(measured - prevMeasured) / dt;
    prevMeasured = measured;

    lastDerivative = alpha * lastDerivative + (1 - alpha) * rawDerivative;

    return (Kp * error) + (Ki * integral) + (Kd * lastDerivative);
}

void PIDController::setTunings(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
}

void PIDController::reset() {
    integral = 0;
    prevMeasured = 0;
    lastDerivative = 0;
}
