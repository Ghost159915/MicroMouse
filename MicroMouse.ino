#include "MotorControl.hpp"
#include "Encoder.hpp"
#include "Display.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"
#include "IMU.hpp"
#include <Wire.h>

#define TICKS_PER_REV 360
const float WHEEL_RADIUS = 16.0;

MotorController motors(11, 12, 9, 10);
Encoder encoder(2, 7);
Display display(WHEEL_RADIUS, TICKS_PER_REV);
PIDController DistancePID(2.0, 0.0, 0.0, 1.0);
PIDController TurningPID(2.0, 0.48, 0.0, 1.0);
LidarSensor lidar;
IMU imu;

unsigned long lastTime = 0;
float rotationOffset = 0.0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(2000);

    display.begin();           // Initialize OLED display

    imu.begin();
    motors.begin();
    encoder.begin();
    lidar.begin();
    imu.calibrate();

    lastTime = millis();
}

static float wrap180(float a) {
    if (a > 180.0) a -= 360.0;
    if (a < -180.0) a += 360.0;
    return a;
}

states currentState = COMMAND_CHAIN;

void loop() {
    unsigned long now = millis();

    float dt = (now - lastTime) / 1000.0;
    if (dt < 0.001) {
        dt = 0.001;
    }

    lastTime = now;
    String command = "frr";
    imu.update();

    switch (currentState) {
        case STARTUP_TURN:
            Serial.println(F("State: STARTUP_TURN"));
            display.showIMUReading((int)imu.yaw());
            motors.PIDturn90CW(&imu, &TurningPID);
            currentState = WAIT_FOR_ROTATION;
            break;

        case WAIT_FOR_ROTATION:
            Serial.println(F("State: WAIT_FOR_ROTATION"));
						display.showIMUReading((int)imu.yaw());
            rotationOffset = motors.waitForRotation(&imu);
            currentState = RETURN_TO_HEADING;
            break;

        case RETURN_TO_HEADING:
            Serial.println(F("State: RETURN_TO_HEADING"));
						display.showIMUReading((int)imu.yaw());
            motors.returnToHeading(&imu, &TurningPID, rotationOffset);
            currentState = WALL_APPROACH;
            break;

        case WALL_APPROACH:
            Serial.println(F("State: WALL_APPROACH"));
            display.showLidarDistance(
                lidar.getLeftDistance(),
                lidar.getFrontDistance(),
                lidar.getRightDistance()
            );
            motors.wallApproach(&lidar, &DistancePID, dt, &currentState);
            break;

        case COMMAND_CHAIN:
            Serial.println(F("State: COMMAND_CHAIN"));
            motors.chainCommand(command, &TurningPID, &encoder, &imu, dt, &currentState);
            currentState = COMPLETE;
            break;

        case COMPLETE:
            Serial.println(F("State: COMPLETE"));
            motors.stop();
            break;

        default:
            motors.stop();
            break;
    }
}

 