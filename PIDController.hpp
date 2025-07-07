#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
private:
  float Kp, Ki, Kd;
  float integral;
  float prevMeasured;

public:
  PIDController(float p, float i, float d)
    : Kp(p), Ki(i), Kd(d), integral(0), prevMeasured(0) {}

  float compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    integral += error * dt;

    // Derivative on measurement (for stability)
    float derivative = -(measured - prevMeasured) / dt;
    prevMeasured = measured;

    return (Kp * error) + (Ki * integral) + (Kd * derivative);
  }

  void setTunings(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
  }

  void reset() {
    integral = 0;
    prevMeasured = 0;
  }
};

#endif
