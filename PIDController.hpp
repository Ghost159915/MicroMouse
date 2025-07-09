#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
private:
  float Kp, Ki, Kd;
  float integral;
  float prevMeasured;
  float lastDerivative;
  float alpha;  // smoothing factor for derivative

public:
  PIDController(float p, float i, float d, float smoothing=0.8)
    : Kp(p), Ki(i), Kd(d), integral(0), prevMeasured(0), lastDerivative(0), alpha(smoothing) {}

  float compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    integral += error * dt;

    // Raw derivative on measurement
    float rawDerivative = -(measured - prevMeasured) / dt;
    prevMeasured = measured;

    // Apply smoothing filter
    lastDerivative = alpha * lastDerivative + (1 - alpha) * rawDerivative;

    return (Kp * error) + (Ki * integral) + (Kd * lastDerivative);
  }

  void setTunings(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
  }

  void reset() {
    integral = 0;
    prevMeasured = 0;
    lastDerivative = 0;
  }
};

#endif
