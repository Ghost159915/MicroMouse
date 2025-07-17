#include "include/MotorControl.hpp"
#include "include/Encoder.hpp"
#include "include/Display.hpp"
#include "include/PIDController.hpp"
#include "include/LidarSensor.hpp"
#include "include/IMU.hpp"
#include <Wire.h>

//RFRFFLFLFFRFFRFFLFFFRFLFRF
//LFLFLFLFLFLFLFLFLFLFLFLFLFLFLFLF
//RFRFRFRFRFRFRFRFRFRFRFRFRFRFRFRF

const float WHEEL_RADIUS = 16.0;
const char* command = "RLF"; // 4X 32 CMD

MotorController motors(11, 12, 9, 10);
Encoder encoder(2, 7);
Display display(WHEEL_RADIUS, TICKS_PER_REV);
PIDController DistancePID(1.5, 0.0, 0.25, 0.0);
PIDController TurningPID(2.0, 0.48, 0.0, 0.0);
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
    // currentState = COMMAND_CHAIN;
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    lastTime = now;

    imu.update();

    switch(currentState) {
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
            // display.showState("WALL_APPROACH");
            display.showLidarDistance(lidar.getLeftDistance(),
                                      lidar.getFrontDistance(),
                                      lidar.getRightDistance());
            motors.wallApproach(&lidar, &DistancePID, dt, &currentState);
            break;

        case COMMAND_CHAIN:
            display.showState("COMMAND_CHAIN");
            motors.processCommandStep(&TurningPID, &encoder, &imu, &currentState);
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
 