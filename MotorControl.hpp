#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <Arduino.h>
#include "IMU.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"

static constexpr float CELL_DISTANCE   = 180.0f;
static constexpr int   TICKS_PER_REV   = 700;
static constexpr float RADIUS          = 16.0f;
static constexpr unsigned long MOVE_TIMEOUT = 3000;  // ms
static constexpr unsigned long TURN_DURATION_MS = 1300; // ms
static constexpr unsigned long WALL_APPROACH_MS = 20000;
static constexpr unsigned long DEFAULT_FORWARD_PWM = 100;

enum states {
    COMMAND_CHAIN,
    COMPLETE,
	PLANNING,
	EXECUTING
};

class MotorController {
public:
    MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir);
	void begin();

    void spinCW(int pwmVal);
    void spinCCW(int pwmVal);
    void stop();

    void startTurn(char direction, IMU* imu, PIDController* turnPID);
    bool updateTurn(IMU* imu, PIDController* turnPID, float dt);


    void driveStraightDualEncoder(DualEncoder* encoder, IMU* imu, PIDController* headingPID, float dt, float basePWM);

    void startCommandChain(const char* cmd);
    void resetInternalState();
    void processCommandStep(PIDController* turnPID, PIDController* headingPID, DualEncoder* encoder, IMU* imu, states* currentState, float dt);

    char getCurrentCommand();

    void driveForwards(int pwmVal);
    void driveBackwards(int pwmVal);


	// void startupTurn(IMU* imu, PIDController* turnPID, float dt, states& currentState);
    // void waitForRotation(IMU* imu, PIDController* turnPID, states& currentState);
    // void returnToHeading(IMU* imu, PIDController* turnPID, float dt, states& currentState);

void wallApproachDirect(LidarSensor* lidar, PIDController* DistancePID, float dt, states* currentState, IMU* imu);

private:
    int mot1_pwm, mot1_dir, mot2_pwm, mot2_dir;

    char commandBuffer[64];
    int commandIndex;
    bool commandActive;
    bool moveInProgress;
    unsigned long moveStartTime;
    float headingTarget;

    bool turnInProgress;
    float turnTargetYaw;
	unsigned long turnStartTime;

    bool startupInitiated;
    float startupYaw;
    unsigned long waitStart;
    float rotationOffset;

    bool wallApproachActive;
    unsigned long wallApproachStart;
    float wallDistanceBuffer[5];
    int wallBufferIndex;

    float wrap180(float angle);

    int forwardRunLength = 0;
    unsigned long forwardTargetTicks = 0;
    int rotationRound = 0;

};

#endif
