#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <Arduino.h>
#include "IMU.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"

#define CELL_DISTANCE 180
#define PI 3.14159265
#define TICKS_PER_REV 700
#define RADIUS 16
#define MOVE_TIMEOUT 3000

enum states {
    STARTUP_TURN,
    WAIT_FOR_ROTATION,
    RETURN_TO_HEADING,
    WALL_APPROACH,
    COMMAND_CHAIN,
    TURNING,
    RETURNING,
    COMPLETE,
    TEST
};

class MotorController {
private:
    int mot1_pwm;
    int mot1_dir;
    int mot2_pwm;
    int mot2_dir;

    bool wallApproachActive;
    unsigned long wallApproachStart;

    char commandBuffer[64];
    int commandIndex;
    bool commandActive;
    bool moveInProgress;
    unsigned long moveStartTime;
    float headingTarget;

    // Turn state
    bool turnInProgress;
    float turnTargetYaw;
    PIDController* turnPID;

public:
    MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir);
	void begin();

	//Basic motion
    void driveStraightIMU(IMU* imu, PIDController* headingPID, float dt, float basePWM);
    void moveForward(int pwmVal);
    void moveBackward(int pwmVal);
    void spinCW(int pwmVal);
    void spinCCW(int pwmVal);
    void stop();

	//Rotations
    void PIDturn90CW(IMU* imu, PIDController* turnPID);
    void PIDturn90CCW(IMU* imu, PIDController* turnPID);
    void startTurn(char direction, IMU* imu, PIDController* turnPID);
    bool updateTurn(IMU* imu, float dt);

	//Other rotations
    void returnToHeading(IMU* imu, PIDController* turnPID, float targetOffset);
    float waitForRotation(IMU* imu);

	//Command chain
    void startCommandChain(const char* cmd);
    bool isCommandActive();
    void processCommandStep(PIDController* turnPID, PIDController* headingPID, Encoder* encoder, IMU* imu, states* currentState, float dt);

	//Wall following
    void wallApproach(LidarSensor* lidar, PIDController* DistancePID, float dt, states* currentState, IMU* imu, PIDController* headingPID);

	//Other
    float wrap180(float angle);
};

#endif
