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

const char* command = "FFRFFLFF";

MotorController motors(11, 12, 9, 10);
Encoder encoder(2, 7);
Display display(RADIUS, TICKS_PER_REV);

PIDController DistancePID(1.5, 0.0, 0.25, 0.0);
PIDController TurningPID(2.0, 0.5, 0.0, 0.0);
PIDController HeadingPID(1.2, 0.0, 0.05, 0.0);

LidarSensor lidar;
IMU imu;

unsigned long lastTime = 0;
float rotationOffset = 0.0;

states currentState = STARTUP_TURN;

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
            motors.startupTurn(&imu, &TurningPID, dt, currentState);
            display.showIMUReading(imu.yaw());
            break;

        case WAIT_FOR_ROTATION:
            display.showState("WAIT_ROTATE");
            motors.waitForRotation(&imu, &TurningPID, currentState);
            display.showIMUReading(imu.yaw());
            break;

        case RETURN_TO_HEADING:
            display.showState("RETURN_HEADING");
            motors.returnToHeading(&imu, &TurningPID, dt, currentState);
            display.showIMUReading(imu.yaw());
            break;

        case WALL_APPROACH:
            display.showLidarDistance(lidar.getLeftDistance(),
                                      lidar.getFrontDistance(),
                                      lidar.getRightDistance());
            motors.wallApproach(&lidar, &DistancePID, dt, &currentState, &imu, &HeadingPID);
            break;

        case COMMAND_CHAIN:
            display.showState("COMMAND_CHAIN");
            motors.processCommandStep(&TurningPID, &HeadingPID, &encoder, &imu, &currentState, dt);
            break;

        case COMPLETE:
            display.showState("COMPLETE");
            motors.stop();
            break;

        default:
            display.showState("UNKNOWN");
            motors.stop();
            break;
    }
}
