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

    bool wallApproachActive = false;
    unsigned long wallApproachStart = 0;

public:

    // Constructor
    MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir)
        : mot1_pwm(m1_pwm), mot1_dir(m1_dir), mot2_pwm(m2_pwm), mot2_dir(m2_dir) {}

    // Pin setup
    void begin() {
        pinMode(mot1_pwm, OUTPUT);
        pinMode(mot1_dir, OUTPUT);
        pinMode(mot2_pwm, OUTPUT);
        pinMode(mot2_dir, OUTPUT);
    }

    // Wrap -180 to +180
    float wrap180(float angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    // Basic motor control
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

    // === Precise Turns ===
    void PIDturn90CW(IMU* imu, PIDController* turnPID) {
        turnPID->reset();
        imu->update();
        float targetAngle = wrap180(imu->yaw() - 90.0);

        unsigned long lastTime = millis();

        while (true) {
            imu->update();
            unsigned long now = millis();
            float dt = max((now - lastTime) / 1000.0f, 0.001f);
            lastTime = now;

            float error = wrap180(targetAngle - imu->yaw());

            if (abs(error) < 3.0) {
                stop();
                break;
            }

            float control = turnPID->compute(0.0, -error, dt);
            int pwm = constrain(abs(control), 0, 255);

            if (error > 0) {
                spinCCW(pwm);
            } else {
                spinCW(pwm);
            }
            delay(10);
        }
    }

    void PIDturn90CCW(IMU* imu, PIDController* turnPID) {
        turnPID->reset();
        imu->update();
        float targetAngle = wrap180(imu->yaw() + 90.0);

        unsigned long lastTime = millis();

        while (true) {
            imu->update();
            unsigned long now = millis();
            float dt = max((now - lastTime) / 1000.0f, 0.001f);
            lastTime = now;

            float error = wrap180(targetAngle - imu->yaw());

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

    // === Return to Heading ===
    void returnToHeading(IMU* imu, PIDController* turnPID, float targetOffset) {
        turnPID->reset();
        imu->update();
        float targetHeading = wrap180(imu->yaw() - targetOffset);

        unsigned long lastTime = millis();

        while (true) {
            imu->update();
            unsigned long now = millis();
            float dt = max((now - lastTime) / 1000.0f, 0.001f);
            lastTime = now;

            float error = wrap180(targetHeading - imu->yaw());
            //Serial.print("ReturnToHeading | Error: "); Serial.println(error);

            if (abs(error) < 1.0) {
                stop();
                //Serial.println("ReturnToHeading: Corrected!");
                break;
            }

            float control = turnPID->compute(0.0, -error, dt);
            int pwm = constrain(abs(control), 0, 255);

            if (error > 0) {
                spinCCW(pwm);
            } else {
                spinCW(pwm);
            }
            delay(10);
        }

        imu->update();
        stop();
    }

    // === Wait for Rotation Detection ===
    float waitForRotation(IMU* imu) {
        stop();
        //Serial.println("WAIT_FOR_ROTATION: Idle and monitoring.");

        imu->update();
        float startHeading = imu->yaw();
        //Serial.print("Start Heading: "); Serial.println(startHeading);

        //Serial.println("10 seconds to rotate...");
        unsigned long startMillis = millis();
        while (millis() - startMillis < 10000) {
            imu->update();
            delay(5);
        }

        imu->update();
        float endHeading = imu->yaw();
        //Serial.print("End Heading: "); Serial.println(endHeading);

        float delta = wrap180(endHeading - startHeading);
        //Serial.print("Rotation Offset Detected: "); Serial.println(delta);

        return delta;
    }

    // === Wall Following for 30s ===
void wallApproach(LidarSensor* lidar, PIDController* DistancePID, float dt, states* currentState) {
    if (!wallApproachActive) {
        Serial.println("WALL_APPROACH: Starting 30-second interaction");
        wallApproachActive = true;
        wallApproachStart = millis();
    }

    // Check if time is up
    if (millis() - wallApproachStart >= 10000) {
        stop();
        //Serial.println("WALL_APPROACH: Finished - Transition to COMMAND_CHAIN");
        wallApproachActive = false;
        *currentState = COMMAND_CHAIN;
        return;
    }

    // Run PID control
    float currentDistance = lidar->getFrontDistance();
    float error = 100.0 - currentDistance;
    float control = DistancePID->compute(0.0, -error, dt);
    int pwm = constrain(abs(control), 0, 255);

    if (error > 3) {
        moveBackward(pwm);
    } else if (error < -3) {
        moveForward(pwm);
    } else {
        stop();
    }

    Serial.print("WALL_APPROACH | LIDAR: "); Serial.print(currentDistance);
    Serial.print(" Error: "); Serial.print(error);
    Serial.print(" PWM: "); Serial.println(pwm);
}

    // === Command Chain Parsing ===
    void chainCommand(String cmd, PIDController* controller, Encoder* encoder, IMU* imu, float dt, states* currentState) {
      Serial.println("Entered chainCommand");

        for (int i = 0; i < cmd.length(); i++) {
            encoder->reset();

            if (cmd[i] == 'f') {
                float total_count = CELL_DISTANCE / (2 * PI * RADIUS) * TICKS_PER_REV;
                while (encoder->getTicks() < total_count) {
                    Serial.print("ENCODER TICKS: ");
                    Serial.println(encoder->getTicks());
                    moveForward(150);
                }
            stop();
            } else if (cmd[i] == 'r') {
                PIDturn90CW(imu, controller);
            } else if (cmd[i] == 'l') {
                PIDturn90CCW(imu, controller);
            }
        }
        Serial.println("chainCommand done, setting COMPLETE");
        *currentState = COMPLETE;
    }
};

#endif
