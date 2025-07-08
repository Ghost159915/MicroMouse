#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <Arduino.h>

class MotorController {
private:
  int mot1_pwm;
  int mot1_dir;
  int mot2_pwm;
  int mot2_dir;

public:
  MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir)
    : mot1_pwm(m1_pwm), mot1_dir(m1_dir), mot2_pwm(m2_pwm), mot2_dir(m2_dir) {}

  void begin() {
    pinMode(mot1_pwm, OUTPUT);
    pinMode(mot1_dir, OUTPUT);
    pinMode(mot2_pwm, OUTPUT);
    pinMode(mot2_dir, OUTPUT);
  }

  void moveForward(int pwmVal) {
    pwmVal = constrain(pwmVal, 0, 255);

    digitalWrite(mot1_dir, HIGH);
    analogWrite(mot1_pwm, pwmVal);

    digitalWrite(mot2_dir, LOW);
    analogWrite(mot2_pwm, pwmVal);
  }

  void moveBackward(int pwmVal) {
    pwmVal = constrain(pwmVal, 0, 255);

    digitalWrite(mot1_dir, LOW);
    analogWrite(mot1_pwm, pwmVal);

    digitalWrite(mot2_dir, HIGH);
    analogWrite(mot2_pwm, pwmVal);
  }

  void stop() {
    analogWrite(mot1_pwm, 0);
    analogWrite(mot2_pwm, 0);
  }

  void spinToHeading(IMU &imu, float targetHeading) {
    float kp = 1.5;
    float ki = 0.0;
    float kd = 0.2;

    float integral = 0;
    float lastError = 0;
    unsigned long lastTime = millis();

    while (true) {
      imu.update();
      float currentHeading = imu.getHeading();

      float error = targetHeading - currentHeading;
      if (error > 180) error -= 360;
      if (error < -180) error += 360;

      if (abs(error) < 3.0) {
        setMotorSpeeds(0, 0);
        Serial.println("Spin complete");
        break;
      }

      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0;
      lastTime = now;

      integral += error * dt;
      float derivative = (error - lastError) / dt;
      lastError = error;

      float output = kp * error + ki * integral + kd * derivative;
      output = constrain(output, -120, 120);

      int leftSpeed = -output;
      int rightSpeed = output;
      setMotorSpeeds(leftSpeed, rightSpeed);

      delay(10);
    }
  }

#endif

