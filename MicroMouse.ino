#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"
#include <Wire.h>

MotorController motors(11, 12, 9, 10);
DualEncoder encoder(2, 7, 3, 8);  // Left encoder on 2,7; Right on 3,8
Display display(RADIUS, TICKS_PER_REV);

PIDController DistancePID(0.9, 0.0, 0.3, 0.8);
PIDController TurningPID(1.92, 0.32, 0.0, 0.8);
PIDController HeadingPID(1.0, 0.0, 0.2, 0.8);
PIDController WallPID(0.5, 0.0, 0.3);

LidarSensor lidar;
IMU imu;

unsigned long lastTime = 0;
float rotationOffset = 0.0;

states currentState = COMMAND_CHAIN;

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

    currentState = FORWARD;
    motors.startCommandChain("RR");

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
        case COMMAND_CHAIN:
            char activeCmd = motors.getCurrentCommand();
            float heading = imu.yaw();
            motors.processCommandStep(&TurningPID, &HeadingPID, &encoder, &imu, &currentState, dt);
            display.showCommandStatus("COMMAND_CHAIN", activeCmd, heading);
            break;

<<<<<<< HEAD
        case COMPLETE: {
            char activeCmd = motors.getCurrentCommand();
            float heading = imu.yaw();
=======
        case COMPLETE:
            // display.showState("COMPLETE");
>>>>>>> main
            motors.stop();
            display.showCommandStatus("COMPLETE", activeCmd, heading);
            break;

<<<<<<< HEAD
        case FORWARD: {
           static bool init = false;
            if (!init) {
                imu.zeroYaw();         // lock heading for this segment
                HeadingPID.reset();
                WallPID.reset();
                init = true;
            }

            const int basePWM = 120;   // or your DEFAULT_FORWARD_PWM
            motors.forwardPWMsWithWalls(&encoder, &imu, &HeadingPID, &WallPID, &lidar,
                basePWM, dt);
            delay(5000);
            break;

        }

        default: {
=======
        case STARTUP_TURN:
            // motors.startupTurn(&imu, &TurningPID, dt, currentState);
            // display.showIMUReading(imu.yaw());
            break;

        case WAIT_FOR_ROTATION:
            // motors.waitForRotation(&imu, &TurningPID, currentState);
            // display.showIMUReading(imu.yaw());
            break;

        case RETURN_TO_HEADING:
            // motors.returnToHeading(&imu, &TurningPID, dt, currentState);
            // display.showIMUReading(imu.yaw());
            break;

        case WALL_APPROACH:
            // display.showLidarDistance(lidar.getLeftDistance(),
            //                           lidar.getFrontDistance(),
            //                           lidar.getRightDistance());
            // motors.wallApproachDirect(&lidar, &DistancePID, dt, &currentState, &imu, &HeadingPID, &encoder);
            break;

        case TEST:
            motors.stop();
            break;

        default:
>>>>>>> main
            motors.stop();
            break;
    }
}