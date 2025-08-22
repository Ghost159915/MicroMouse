#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"
#include <Wire.h>

bool AUTONOMOUS_MAPPING = false;

MotorController motors(11, 12, 9, 10);
DualEncoder encoder(2, 7, 3, 8);  // Left encoder on 2,7; Right on 3,8
Display display(RADIUS, TICKS_PER_REV);

PIDController DistancePID(0.9, 0.0, 0.3, 0.8);
PIDController TurningPID(2.2, 0.0, 0.0, 0.8);
PIDController HeadingPID(1.0, 0.0, 0.2, 0.8);
PIDController WallPID(0.5, 0.0, 0.0, 0.8);

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
    motors.startCommandChain("FFLFFLFLFRFRFFRFF");
    imu.calibrate();
    delay(1500);
}
//FRFFRFLFFLFLFRFRFLFLFFFF
//FFFRFFRFFRFLFLFFLFRFLFFLFLF
//FFFRFFRFLFRFFRFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void loop() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    lastTime = now;
    imu.update();

    switch (currentState) {
        case COMMAND_CHAIN:
            char activeCmd = motors.getCurrentCommand();
            if (AUTONOMOUS_MAPPING) {
                currentState = FLOOD_FILL_MAPPING;
            }
            else if (activeCmd == '\0' || !(motors.getCommandActive())) {
                currentState = COMPLETE;
            }
            float heading = imu.yaw();
            motors.processCommandStep(&TurningPID, &HeadingPID, &WallPID, &encoder, &imu, &currentState, &lidar, dt);
            display.showCommandStatus("COMMAND_CHAIN", activeCmd, heading);

            // static unsigned long t0 = millis();
            // if ((millis() - t0) > 300) {
            //     t0 = millis();
            // }
            // Serial.print(encoder.getLeftTicks()); Serial.print("    "); Serial.println(encoder.getRightTicks());

            break;

        case COMPLETE:
            motors.stop();
            break;

        case FLOOD_FILL_MAPPING:
            break;

        default:
            motors.stop();
            break;
    }
}