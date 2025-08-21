#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"
#include <Wire.h>

MotorController motors(11, 12, 9, 10);
DualEncoder encoder(3, 8, 2, 7);  // Left encoder on 2,7; Right on 3,8 HAD TO SWAP THE ORDER TO GET THEM TO WORK, INITIALLY IT WAS encoder(2, 7, 3, 8)
Display display(RADIUS, TICKS_PER_REV);

PIDController DistancePID(0.9, 0.0, 0.3, 0.8);
PIDController TurningPID(1, 0.5, 0.32, 0.8);
PIDController HeadingPID(1.0, 0.0, 0.2, 0.8);
PIDController WallPID(0.3, 0.0, 0.3);

LidarSensor lidar;
IMU imu;

unsigned long lastTime = 0;
float rotationOffset = 0.0;

states currentState = FORWARD;

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
    motors.startCommandChain("F");
    // FRFFRFLFFLFLFRFRFLFLFFFF

    imu.calibrate();
    delay(1500);
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
            Serial.print("Heading: ");
            Serial.println(heading);

            motors.processCommandStep(&TurningPID, &HeadingPID, &WallPID, &encoder, &imu, &currentState, &lidar, dt);
            display.showCommandStatus("COMMAND_CHAIN", activeCmd, heading);
            break;
        }

        case COMPLETE: {
            char activeCmd = motors.getCurrentCommand();
            float heading = motors.getFusedYaw();
            motors.stop();
            display.showCommandStatus("COMPLETE", activeCmd, heading);
            break;
        }

        case FORWARD: {
            char activeCmd = motors.getCurrentCommand();
            float heading = imu.yaw();
            static bool init = false;
            if (!init) {
                imu.zeroYaw();         // lock heading for this segment
                HeadingPID.reset();
                WallPID.reset();
                init = true;
            }

            const int basePWM = 60;   // or your DEFAULT_FORWARD_PWM
            motors.forwardPWMsWithWalls(&encoder, &imu, &HeadingPID, &WallPID, &lidar,
                                        basePWM, dt);
            display.showCommandStatus("FORWARD", activeCmd, heading);
            break;
        }

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
            static unsigned long t0 = millis();
            if (millis() - t0 > 300) {
            t0 = millis();
            Serial.print("L="); Serial.print(encoder.getLeftTicks());
            Serial.print("  R="); Serial.println(encoder.getRightTicks());
}

            break;

        default:
            motors.stop();
            break;
    }
}
