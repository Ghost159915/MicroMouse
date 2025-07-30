#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"
#include <Wire.h>

MotorController motors(11, 12, 9, 10);
DualEncoder encoder(2, 7, 3, 8);
Display display(RADIUS, TICKS_PER_REV);

PIDController DistancePID(0.9, 0.0, 0.3, 0.8);
PIDController TurningPID(1.92, 0.32, 0.0, 0.8);
PIDController HeadingPID(1.0, 0.0, 0.2, 0.8);

LidarSensor lidar;
IMU imu;
states currentState;

unsigned long lastTime = 0;
float rotationOffset = 0.0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    display.begin();
    imu.begin();
    motors.begin();
    encoder.begin();
    lidar.begin();
    encoder.reset();
    lastTime = millis();
    currentState = COMMAND_CHAIN;
    motors.startCommandChain("FFLFRFF");
    imu.calibrate();
    delay(3000);
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    lastTime = now;
    imu.update();

    switch (currentState) {
        case COMMAND_CHAIN: {
            char activeCmd = motors.getCurrentCommand();
            float heading = imu.yaw();
            motors.processCommandStep(&TurningPID, &HeadingPID, &encoder, &imu, &currentState, dt);
            display.showCommandStatus("COMMAND_CHAIN", activeCmd, heading);
            break; 
        }

        case WALL_APPROACH: {
            display.showLidarDistance(lidar.getLeftDistance(), lidar.getFrontDistance(), lidar.getRightDistance());
    		motors.wallApproachDirect(&lidar, &DistancePID, dt, &currentState, &imu);
            break;
        }

        case COMPLETE: {
            motors.stop();
            break;
        }

        case STARTUP_TURN: {
            // motors.startupTurn(&imu, &TurningPID, dt, currentState);
            // display.showIMUReading(imu.yaw());
            break;
        }

        case WAIT_FOR_ROTATION: {
            // motors.waitForRotation(&imu, &TurningPID, currentState);
            // display.showIMUReading(imu.yaw());
            break;
        }

        case RETURN_TO_HEADING: {
            // motors.returnToHeading(&imu, &TurningPID, dt, currentState);
            // display.showIMUReading(imu.yaw());
            break;
        }

        case TEST: {
            motors.stop();
            break;
        }

        default: {
            motors.stop();
            break;
        }
    }
}