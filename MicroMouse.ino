#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"
#include <Wire.h>

//RFRFFLFLFFRFFRFFLFFFRFLFRF
//LFLFLFLFLFLFLFLFLFLFLFLFLFLFLFLF
//RFRFRFRFRFRFRFRFRFRFRFRFRFRFRFRF

const float WHEEL_RADIUS = 16.0;
const char* command = "RFRFFLFLFFRFFRFFLFFFRFLFRF"; // 4X 32 CMD

MotorController motors(11, 12, 9, 10);
Encoder encoder(2, 7);
Display display(WHEEL_RADIUS, TICKS_PER_REV);
PIDController DistancePID(2.0, 0.0, 0.0, 1.0);
PIDController TurningPID(2.0, 0.48, 0.0, 1.0);
LidarSensor lidar;
IMU imu;

unsigned long lastTime = 0;
float rotationOffset = 0.0;
states currentState = COMMAND_CHAIN;

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
    currentState = COMMAND_CHAIN;
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    lastTime = now;

    imu.update();

    switch(currentState) {
        case STARTUP_TURN:
            display.showIMUReading(imu.yaw());
            motors.PIDturn90CW(&imu, &TurningPID);
            currentState = WAIT_FOR_ROTATION;
            break;

        case WAIT_FOR_ROTATION:
            rotationOffset = motors.waitForRotation(&imu);
            currentState = RETURN_TO_HEADING;
            break;

        case RETURN_TO_HEADING:
            motors.returnToHeading(&imu, &TurningPID, rotationOffset);
            currentState = WALL_APPROACH;
            break;

        case WALL_APPROACH:
            display.showLidarDistance(lidar.getLeftDistance(), lidar.getFrontDistance(), lidar.getRightDistance());
            motors.wallApproach(&lidar, &DistancePID, dt, &currentState);
            break;

        case COMMAND_CHAIN:
            motors.processCommandStep(&TurningPID, &encoder, &imu, &currentState);
            break;

        case COMPLETE:
            motors.stop();
            break;

        default:
            motors.stop();
            break;
    }

    

}
 