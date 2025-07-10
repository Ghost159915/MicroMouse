#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP
#define CELL_DISTANCE 180
#define PI 3.14159265

#define TICKS_PER_REV 700
#define RADIUS 16


#include <Arduino.h>
#include <math.h>
#include "IMU.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include <Wire.h>


enum states {
    STARTUP_TURN,
    WAIT_FOR_ROTATION,
    RETURN_TO_HEADING,
    WALL_APPROACH,
    COMMAND_CHAIN,
    COMPLETE
};

class MotorController {
private:
    int mot1_pwm;
    int mot1_dir;
    int mot2_pwm;
    int mot2_dir;

public:

    //Constructor
    MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir)
        : mot1_pwm(m1_pwm), mot1_dir(m1_dir), mot2_pwm(m2_pwm), mot2_dir(m2_dir) {}

    //Pin setup
    void begin() {
        pinMode(mot1_pwm, OUTPUT);
        pinMode(mot1_dir, OUTPUT);
        pinMode(mot2_pwm, OUTPUT);
        pinMode(mot2_dir, OUTPUT);
    }

  // Helper to keep angle in -180 to 180
    float wrap180(float a) {
        if (a > 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
    }

    // Basic movement
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

    void spinCW(int pwmVal) {
        pwmVal = constrain(pwmVal, 0, 255);

        digitalWrite(mot1_dir, LOW);
        analogWrite(mot1_pwm, pwmVal);

        digitalWrite(mot2_dir, LOW);
        analogWrite(mot2_pwm, pwmVal);
    }

    void spinCCW(int pwmVal) {
        pwmVal = constrain(pwmVal, 0, 255);

        digitalWrite(mot1_dir, HIGH);
        analogWrite(mot1_pwm, pwmVal);

        digitalWrite(mot2_dir, HIGH);
        analogWrite(mot2_pwm, pwmVal);
    }

    void stop() {
        analogWrite(mot1_pwm, 0);
        analogWrite(mot2_pwm, 0);
   }

void returnToHeading(IMU* imu, PIDController* turnPID, float targetOffset) {
    turnPID->reset();
    float startAngle = imu->yaw();
    float targetAngle = wrap180(startAngle - targetOffset);

    unsigned long lastTime = millis();

    while (true) {
      imu->update();

      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0;
      if (dt < 0.001) dt = 0.001;
      lastTime = now;

      float currentAngle = imu->yaw();
      float error = wrap180(targetAngle - currentAngle);

      if (abs(error) < 1.0) {
        stop();
        Serial.println("Heading Corrected!");

        break;
      }

      float control = turnPID->compute(0.0, -error, dt);
      int pwm = constrain(abs(control), 0, 255);

      if (error > 0) {
        spinCCW(pwm);
        Serial.print("Heading Correcting CCW  |  Error: "); Serial.println(error);

      } else {
        spinCW(pwm);
        Serial.print("Heading Correcting CW  |  Error: "); Serial.println(error);
      }
    delay(10);
    }
}

  // Precision 90-degree turn CW using PID + IMU
  void PIDturn90CW(IMU* imu, PIDController* turnPID) {
    turnPID->reset();
    float startAngle = imu->yaw();
    float targetAngle = wrap180(startAngle + 90.0);

    unsigned long lastTime = millis();

    while (true) {
      imu->update();

      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0;
      if (dt < 0.001) dt = 0.001;
      lastTime = now;

      float currentAngle = imu->yaw();
      float error = wrap180(targetAngle - currentAngle);

      if (abs(error) < 3.0) {
        stop();
        break;
      }

      float control = turnPID->compute(0.0, -error, dt);
      int pwm = constrain(abs(control), 0, 255);

      if (error > 0) {
        spinCCW(pwm);
        Serial.print("Heading Correcting CCW  |  Error: "); Serial.println(error);

      } else {
        spinCW(pwm);
        Serial.print("Heading Correcting CW  |  Error: "); Serial.println(error);

      }
    delay(10);
    }
    
  }

  void PIDturn90CCW(IMU* imu, PIDController* turnPID) {
    turnPID->reset();
    float startAngle = imu->yaw();
    float targetAngle = wrap180(startAngle - 90.0);
    unsigned long lastTime = millis();

    while (true) {
      imu->update();

      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0;
      if (dt < 0.001) dt = 0.001;
      lastTime = now;

      float currentAngle = imu->yaw();
      float error = wrap180(targetAngle - currentAngle);

      if (abs(error) < 3.0) {
        stop();
        break;
      }

      float control = turnPID->compute(0.0, -error, dt);
      int pwm = constrain(abs(control), 0, 255);

      if (error > 0) {
        spinCW(pwm);
      } else {
        spinCCW(pwm);
      }
    delay(10);
    }
  }

  void wallApproach (LidarSensor* lidar, PIDController* DistancePID, float dt, states* currentState) {
    Serial.println("WALL_APPROACH: Starting 30-second interaction");
    unsigned long startTime = millis();

    while (millis() - startTime < 30000) {

        float currentDistance = lidar->getFrontDistance();
        float error = 100.0 - currentDistance;

        float control = DistancePID->compute(0.0, -error, dt);
        int pwm = constrain(abs(control), 0, 255);

        Serial.print("LIDAR Distance: "); Serial.print(currentDistance);
        Serial.print("  |   Error: "); Serial.print(error);
        Serial.print("  |   PWM Command: "); Serial.println(pwm);

        if (error > 3) {
            moveBackward(pwm);
        } else if (error < -3) {
            moveForward(pwm);
        } else {
            stop();
        }

        delay(50);
    }

    // After timeout
    stop();
    Serial.println("WALL_APPROACH: Finished");
    *currentState = COMMAND_CHAIN;

  }

  void chainCommand (String cmd, PIDController* controller, Encoder* encoder, IMU* imu, float dt, states* currentState) {
    for (int i = 0; i < cmd.length(); i++) {
      encoder->reset();

      if (cmd[i] == 'f') {
        float total_count = CELL_DISTANCE / (2 * PI * RADIUS) * TICKS_PER_REV;
        while (encoder->getTicks() < total_count) {
          moveForward(150);
        }
        stop();
      } else if (cmd[i] == 'r') {
        PIDturn90CW(imu, controller);
      } else if(cmd[i] == 'l') {
        PIDturn90CCW(imu, controller);
      }
    }
    *currentState = COMPLETE;
  }


  float waitForRotation(IMU* imu) {
    // Take initial baseline acceleration

    stop();
    Serial.println("WAIT_FOR_ROTATION: Stop motors.");

    float startHeading = imu->yaw();

    Serial.print("Start Heading: "); Serial.println(startHeading);
    Serial.println("You have 10 seconds to rotate the robot!");

    unsigned long startMillis = millis();
    while (millis() - startMillis < 10000) {
      imu->update();
      float currentHeading = imu->yaw();
      Serial.println(currentHeading);
      delay(5);  // small delay to not spam too much
    }

    imu->update();

    float endHeading = imu->yaw();
    Serial.print("End Heading: "); Serial.println(endHeading);

    float delta = wrap180(endHeading - startHeading);
    Serial.print("Rotation Offset Detected: "); Serial.println(delta);

    return delta;
  }
};

#endif

