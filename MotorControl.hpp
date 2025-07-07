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

  void spin(int pwmVal) {
    pwmVal = constrain(pwmVal, 0, 255);

    digitalWrite(mot1_dir, LOW);
    analogWrite(mot1_pwm, pwmVal);

    digitalWrite(mot2_dir, LOW);
    analogWrite(mot2_pwm, pwmVal);
  }
};

#endif

