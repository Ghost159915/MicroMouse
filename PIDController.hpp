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

  PIDController(float p, float i, float d, float smoothing=0.8);
  float compute(float setpoint, float measured, float dt);
  void setTunings(float p, float i, float d);
  void reset();

};

#endif
