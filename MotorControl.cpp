#include "MotorControl.hpp"

static constexpr float wEnc       = 0.6f;   // blend weights: encoders > IMU > walls
static constexpr float wIMU       = 0.3f;
static constexpr float wWall      = 0.1f;

static constexpr float kEncGain   = 0.50f;  // ticks -> steering scale
static constexpr float kIMUClamp  = 45.0f;  // deg clamp before PID
static constexpr float kWallClamp = 40.0f;  // max PWM from wall PID
static constexpr float kSteerMax  = 60.0f;  // overall steering clamp
static constexpr float kSpeedMax  = 30.0f;  // (optional) common speed clamp
static constexpr int   kMinPWM    = 60;     // overcome static friction
static constexpr int   kMaxPWM    = 255;    // PWM ceiling

static inline bool validSideMM(float d) { return isfinite(d) && d > 40.f && d < 1200.f; }
static inline float clampf(float x, float lo, float hi){ return x < lo ? lo : (x > hi ? hi : x); }

MotorController::MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir)
    : mot1_pwm(m1_pwm), mot1_dir(m1_dir),
      mot2_pwm(m2_pwm), mot2_dir(m2_dir), commandIndex(0), commandActive(false),
      moveInProgress(false), moveStartTime(0), turnInProgress(false), turnTargetYaw(0.0f),
      turnStartTime(0), startupInitiated(false), startupYaw(0.0f),
      waitStart(0), rotationOffset(0.0f), wallApproachActive(false), wallApproachStart(0),
      wallBufferIndex(0)
{
    for (int i = 0; i < 5; ++i) wallDistanceBuffer[i] = 100.0f;
}

void MotorController::begin() {
    pinMode(mot1_pwm, OUTPUT);
    pinMode(mot1_dir, OUTPUT);
    pinMode(mot2_pwm, OUTPUT);
    pinMode(mot2_dir, OUTPUT);
}

float MotorController::wrap180(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

void MotorController::spinCW(int pwmVal) {
    pwmVal = constrain(pwmVal, 0, 255);
    digitalWrite(mot1_dir, LOW);
    analogWrite(mot1_pwm, pwmVal);
    digitalWrite(mot2_dir, LOW);
    analogWrite(mot2_pwm, pwmVal);
}

void MotorController::spinCCW(int pwmVal) {
    pwmVal = constrain(pwmVal, 0, 255);
    digitalWrite(mot1_dir, HIGH);
    analogWrite(mot1_pwm, pwmVal);
    digitalWrite(mot2_dir, HIGH);
    analogWrite(mot2_pwm, pwmVal);
}

void MotorController::stop() {
    analogWrite(mot1_pwm, 0);
    analogWrite(mot2_pwm, 0);
}

void MotorController::startTurn(char direction, IMU* imu, PIDController* turnPID) {

    float startYaw = imu->yaw();
    float delta = (direction == 'R') ? -90.0f : +90.0f;
    turnTargetYaw = wrap180(startYaw + delta);
    turnInProgress = true;
    turnStartTime = millis();
    turnPID->reset();
}

bool MotorController::updateTurn(IMU* imu, PIDController* turnPID, float dt) {
    if (!turnInProgress) return true;

    imu->update();
    float currentYaw = imu->yaw();
    float err = wrap180(currentYaw - turnTargetYaw);

    float corr = turnPID->compute(0.0, err, dt);
    int pwm = constrain(abs(corr), 0, 255);

    if (corr > 0) {
        spinCCW(pwm);
    } else {
        spinCW(pwm);
    }

    //|| millis() - turnStartTime >= TURN_DURATION_MS
    Serial.println(err);

    if (abs(err) < 2) {
        stop();
        turnInProgress = false;
        imu->zeroYaw();
        return true;
    }
    return false;
}

void MotorController::startCommandChain(const char* cmd) {
    strncpy(commandBuffer, cmd, sizeof(commandBuffer) - 1);
    commandBuffer[sizeof(commandBuffer) - 1] = '\0';
    commandIndex = 0;
    commandActive = true;
    moveInProgress = false;
    resetInternalState();
}

void MotorController::processCommandStep(PIDController* turnPID, PIDController* headingPID,
                                         DualEncoder* encoder, IMU* imu, states* currentState, float dt) {
    if (!commandActive || commandBuffer[commandIndex] == '\0') {
        stop();
        *currentState = COMPLETE;
        return;
    }

    char cmd = commandBuffer[commandIndex];

    if (!moveInProgress) {
        if (cmd == 'F') {
            forwardRunLength = 1;
            while (commandBuffer[commandIndex + forwardRunLength] == 'F' &&
                   commandIndex + forwardRunLength < sizeof(commandBuffer) - 1 &&
                   forwardRunLength < 10) {
                forwardRunLength++;
            }

            float totalDist = forwardRunLength * CELL_DISTANCE;
            float rotations = totalDist / (2.0f * PI * RADIUS);
            forwardTargetTicks = round(rotations * TICKS_PER_REV);

            moveInProgress = true;
            moveStartTime = millis();
            imu->update();
            headingTarget = imu->yaw();
            headingPID->reset();
            encoder->reset();
        }
        else if (cmd == 'R' || cmd == 'L') {
            startTurn(cmd, imu, turnPID);
            moveInProgress = true;
        }
        else {
            // Unknown command; skip it
            commandIndex++;
        }
    }
    else {
        if (cmd == 'F') {
            long currentTicks = encoder->getAverageTicks();
            if (currentTicks < forwardTargetTicks && (millis() - moveStartTime) < MOVE_TIMEOUT) {
                driveStraightDualEncoder(encoder, imu, headingPID, dt, DEFAULT_FORWARD_PWM);
            } else {
                stop();
                commandIndex += forwardRunLength;
                moveInProgress = false;
            }
        }
        else if (cmd == 'R' || cmd == 'L') {
            if (updateTurn(imu, turnPID, dt)) {
                stop();
                commandIndex++;
                moveInProgress = false;
            }
        }
    }
}

void MotorController::resetInternalState() {
    turnInProgress = false;
    startupInitiated = false;
    wallApproachActive = false;
    wallApproachStart = 0;
    waitStart = 0;
    rotationOffset = 0.0f;
    headingTarget = 0.0f;
    forwardTargetTicks = 0;
    forwardRunLength = 0;
    wallBufferIndex = 0;
    for (int i = 0; i < 5; ++i) wallDistanceBuffer[i] = 100.0f;
}

void MotorController::driveStraightDualEncoder(DualEncoder* encoder, IMU* imu,
                                               PIDController* headingPID, float dt, float basePWM) {

    bool movingForward = basePWM >= 0;
    int absPWM = abs(basePWM);

    long leftTicks = encoder->getLeftTicks();
    long rightTicks = encoder->getRightTicks();
    long tickDiff = leftTicks - rightTicks;

    imu->update();
    float headingError = wrap180(headingTarget - imu->yaw());

    float encoderCorrection = tickDiff * 0.5;
    float imuCorrection = headingPID->compute(0.0, -headingError, dt);

    float totalCorrection = (encoderCorrection * 0.7) + (imuCorrection * 0.3);
    totalCorrection = constrain(totalCorrection, -50, 50);

    int leftPWM = basePWM - totalCorrection;
    int rightPWM = basePWM + totalCorrection;

    if (leftPWM > 0 && leftPWM < 60) leftPWM = 60;
    if (rightPWM > 0 && rightPWM < 60) rightPWM = 60;

    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    digitalWrite(mot1_dir, HIGH);
    digitalWrite(mot2_dir, LOW);
    analogWrite(mot1_pwm, leftPWM);
    analogWrite(mot2_pwm, rightPWM);
}

char MotorController::getCurrentCommand() {
    return commandBuffer[commandIndex];
}

void MotorController::wallApproachDirect(LidarSensor* lidar, PIDController* DistancePID, float dt, states* currentState, IMU* imu) {
    if (!wallApproachActive) {
        wallApproachActive = true;
        wallApproachStart = millis();

        imu->update();
        float headinTarget = 0.0;

        float initialDist = lidar->getFrontDistance();
        for (int i = 0; i < 5; i++) {
            wallDistanceBuffer[i] = lidar->getFrontDistance();
        }
        wallBufferIndex = 0;

        DistancePID->reset();
    }

    if (millis() - wallApproachStart >= WALL_APPROACH_MS) {
        stop();
        wallApproachActive = false;
        *currentState = COMMAND_CHAIN;
        return;
    }

    float rawDist = lidar->getFrontDistance();
    wallDistanceBuffer[wallBufferIndex] = rawDist;
    wallBufferIndex = (wallBufferIndex + 1) % 5;

    float sum = 0.0f;
    for (int i = 0; i < 5; i++) {
