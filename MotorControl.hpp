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
static constexpr unsigned long TURN_DURATION_MS = 1500; // ms
static constexpr unsigned long WALL_APPROACH_MS = 20000;

enum states {
    STARTUP_TURN,
    WAIT_FOR_ROTATION,
    RETURN_TO_HEADING,
    WALL_APPROACH,
    COMMAND_CHAIN,
    COMPLETE
};

class MotorController {
public:
    MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir);
	void begin();

	//Basic motion
    void driveStraightIMU(IMU* imu, PIDController* headingPID, float dt, float basePWM);
    void spinCW(int pwmVal);
    void spinCCW(int pwmVal);
    void stop();

    // Non-blocking 90° turns
    void startTurn(char direction, IMU* imu, PIDController* turnPID);
    bool updateTurn(IMU* imu, float dt);

    // Wall following
    void wallApproach(LidarSensor* lidar, PIDController* DistancePID, float dt, states* currentState, IMU* imu, PIDController* headingPID);

    // Command chain
    void startCommandChain(const char* cmd);
    void resetInternalState();
    void processCommandStep(PIDController* turnPID, PIDController* headingPID, Encoder* encoder, IMU* imu, states* currentState, float dt);

    // Startup sequence handlers
	void startupTurn(IMU* imu, PIDController* turnPID, float dt, states& currentState);
    void waitForRotation(IMU* imu, PIDController* turnPID, states& currentState);
    void returnToHeading(IMU* imu, PIDController* turnPID, float dt, states& currentState);

private:
    int mot1_pwm, mot1_dir, mot2_pwm, mot2_dir;

    char commandBuffer[64];
    int commandIndex;
    bool commandActive;
    bool moveInProgress;
    unsigned long moveStartTime;
    float headingTarget;

    //Turn state
    bool turnInProgress;
    float turnTargetYaw;
    PIDController* turnPID;
	unsigned long turnStartTime;

    // Startup helper state
    bool startupInitiated;
    float startupYaw;
    unsigned long waitStart;
    float rotationOffset;

    // Wall approach state
    bool wallApproachActive;
    unsigned long wallApproachStart;
    float wallDistanceBuffer[5];
    int wallBufferIndex;

    // Helpers
    float wrap180(float angle);

    int forwardRunLength = 0;
    unsigned long forwardTargetTicks = 0;
    int rotationRound = 0;

};

#endif
