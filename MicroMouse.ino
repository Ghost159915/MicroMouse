#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"
#include <Wire.h>

// RFRFFLFLFFRFFRFFLFFFRFLFRF
// LFLFLFLFLFLFLFLFLFLFLFLFLFLFLFLF
// RFRFRFRFRFRFRFRFRFRFRFRFRFRFRFRF

const float WHEEL_RADIUS = 16.0;
const char* command = "FFRFFLFF";

MotorController motors(11, 12, 9, 10);
Encoder encoder(2, 7);
Display display(WHEEL_RADIUS, TICKS_PER_REV);

PIDController DistancePID(1.5, 0.0, 0.25, 0.0);
PIDController TurningPID(2.0, 0.5, 0.0, 0.0);
PIDController HeadingPID(1.2, 0.0, 0.05, 0.0);

LidarSensor lidar;
IMU imu;

unsigned long lastTime = 0;
float rotationOffset = 0.0;

states currentState = TEST;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(2000);
    display.begin();
    imu.begin();
    motors.begin();
    encoder.begin();
    lidar.begin();
    imu.calibrate();

    lastTime = millis();

    motors.startCommandChain(command);
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    lastTime = now;

    imu.update();

    switch (currentState) {
        case STARTUP_TURN:
            display.showState("STARTUP_TURN");
            display.showIMUReading(imu.yaw());
            motors.PIDturn90CW(&imu, &TurningPID);
            currentState = WAIT_FOR_ROTATION;
            break;

        case WAIT_FOR_ROTATION:
            display.showState("WAIT_ROTATE");
            rotationOffset = motors.waitForRotation(&imu);
            currentState = RETURN_TO_HEADING;
            break;

        case RETURN_TO_HEADING:
            display.showState("RETURN_HEADING");
            motors.returnToHeading(&imu, &TurningPID, rotationOffset);
            delay(5000);
            currentState = WALL_APPROACH;
            break;

        case WALL_APPROACH:
            display.showLidarDistance(lidar.getLeftDistance(),
                                      lidar.getFrontDistance(),
                                      lidar.getRightDistance());
            motors.wallApproach(&lidar, &DistancePID, dt, &currentState);
            break;

        case COMMAND_CHAIN:
            display.showState("COMMAND_CHAIN");
            motors.processCommandStep(&TurningPID, &HeadingPID, &encoder, &imu, &currentState, dt);
            break;

        case COMPLETE:
            display.showState("COMPLETE");
            motors.stop();
            break;

        case TEST:
            display.showState("TEST");

            static bool started = false;
            static unsigned long moveStart = 0;

            if (!started) {
                encoder.reset();
                imu.update();
                float headingTarget = imu.yaw();
                HeadingPID.reset();
                moveStart = millis();
                started = true;
                Serial.println("TEST: Driving straight for 1m");
            }

            float distance = encoder.getTicks() / (float)TICKS_PER_REV * (2 * PI * RADIUS);

            if (distance < 1000.0f && (millis() - moveStart) < 4000) {
                motors.driveStraightIMU(&imu, &HeadingPID, dt, 150);
            } else {
                motors.stop();
                Serial.println("TEST: Done");
                currentState = COMPLETE;
            }
            break;

        default:
            display.showState("UNKNOWN");
            motors.stop();
            break;
    }
}
