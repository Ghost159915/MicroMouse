#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <Arduino.h>
#include "IMU.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "LidarSensor.hpp"

static constexpr float CELL_DISTANCE   = 180.0f;
static constexpr int   TICKS_PER_REV   = 700;
static constexpr int   WALL_THRESHOLD   = 100;
static constexpr float RADIUS          = 16.0f;
static constexpr unsigned long MOVE_TIMEOUT = 3000;  // ms
static constexpr unsigned long TURN_DURATION_MS = 1300; // ms
static constexpr unsigned long WALL_APPROACH_MS = 20000;
static constexpr unsigned long DEFAULT_FORWARD_PWM = 60;
static constexpr float WHEEL_BASE = 97.5;

enum states {
    COMMAND_CHAIN,
    COMPLETE,
    TEST,
    FLOOD_FILL_MAPPING
};

class MotorController {
public:
    MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir);
	void begin();

    void spinCW(int pwmVal);
    void spinCCW(int pwmVal);
    void stop();

    void startTurn(char direction, IMU* imu, PIDController* turnPID);
    void startTurn45(char direction, IMU* imu, PIDController* turnPID);
    bool updateTurn(IMU* imu, PIDController* turnPID, float dt, DualEncoder* encoder);


    void driveStraightDualEncoder(DualEncoder* encoder, IMU* imu, PIDController* headingPID, PIDController* wallPID, float dt, float basePWM);

    void startCommandChain(const char* cmd);
    void resetInternalState();
    void processCommandStep(PIDController* turnPID, PIDController* headingPID, PIDController* wallPID, DualEncoder* encoder, IMU* imu, states* currentState, LidarSensor* lidar, float dt);

    char getCurrentCommand();
    bool getCommandActive();

    bool MotorController::isWallRight(LidarSensor* lidar);
    bool MotorController::isWallLeft(LidarSensor* lidar);
    bool MotorController::isWallBoth(LidarSensor* lidar);
    bool MotorController::isWallfront(LidarSensor* lidar);


	// void startupTurn(IMU* imu, PIDController* turnPID, float dt, states& currentState);
    // void waitForRotation(IMU* imu, PIDController* turnPID, states& currentState);
    // void returnToHeading(IMU* imu, PIDController* turnPID, float dt, states& currentState);

    // void wallApproachDirect(LidarSensor* lidar, PIDController* DistancePID,
    //                        float dt, states* currentState, IMU* imu,
    //                        PIDController* headingPID, DualEncoder* encoder);

private:
    int mot1_pwm, mot1_dir, mot2_pwm, mot2_dir;

    char commandBuffer[64]; //INCREASE POSSIBLY
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
