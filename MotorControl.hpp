#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP
#define CELL_DISTANCE = 180

#include <Arduino.h>
#include <string>
#include "IMU.hpp"

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

  void spinCW(int pwmVal) (
    pwmVal = constrain(pwmVal, 0, 255);

    digitalWrite(mot1_dir, LOW);
    analogWrite(mot1_pwm, pwmVal);

    digitalWrite(mot2_dir, LOW);
    analogWrite(mot2_pwm, pwmVal);
  )

    void spinCCW(int pwmVal) (
    pwmVal = constrain(pwmVal, 0, 255);

    digitalWrite(mot1_dir, HIGH);
    analogWrite(mot1_pwm, pwmVal);

    digitalWrite(mot2_dir, HIGH);
    analogWrite(mot2_pwm, pwmVal);
  )

  void stop() {
    analogWrite(mot1_pwm, 0);
    analogWrite(mot2_pwm, 0);
  }

  void chainCommand (string cmd, PIDcontroller* controller, IMU* imu) {

    for (int i = 0; i < string.length(cmd); i++) {

      // add calibrate() after each movement
      //delay(50)
      if (cmd[i] == 'f') {
        float total_count = CELL_DISTANCE/(2*pi*16);
        Encoder encoders = encoder();
        while(encoders.getTicks() < total_count) {
          moveForward(255);
        }
        delay(100);

      } else if (cmd[i] == 'l') {

        float measuredAngle = imu.yaw();
        float DESIRED_ANGLE = 90.0;

        float angleError = wrap180(DESIRED_ANGLE - measuredAngle);
        float angleCmd   = controller.compute(0.0f, -angleError, dt);
        int   pwm        = constrain(abs(angleCmd), 0, 255);    

        if (angleError > 3) {
          spinCW(pwm);
        } else if (angleError < -3) {
          spinCCW(pwm);
        } else {
          stop();
        }

      } else if (cmd[i] == 'r') {
        float measuredAngle = imu.yaw();
        float DESIRED_ANGLE = -90.0;

        float angleError = wrap180(DESIRED_ANGLE - measuredAngle);
        float angleCmd   = controller.compute(0.0f, -angleError, dt);
        int   pwm        = constrain(abs(angleCmd), 0, 255);    

        if (angleError > 3) {
          spinCW(pwm);
        } else if (angleError < -3) {
          spinCCW(pwm);
        } else {
          stop();
        }
      }
    }
  }
}

#endif

