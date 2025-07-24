#include "MotorControl.hpp"

MotorController::MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir)
    : mot1_pwm(m1_pwm), mot1_dir(m1_dir),
    mot2_pwm(m2_pwm), mot2_dir(m2_dir), commandIndex(0), commandActive(false),
    moveInProgress(false), moveStartTime(0), turnInProgress(false), turnTargetYaw(0.0f),
    turnPID(nullptr), turnStartTime(0), startupInitiated(false), startupYaw(0.0f),
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

void MotorController::driveStraightIMU(IMU* imu, PIDController* headingPID, float dt, float basePWM) {
	imu->update();
    float error = wrap180(headingTarget - imu->yaw());
    if (abs(error) < 1.0) error = 0.0;
    float correction = headingPID->compute(0.0, -error, dt);

    int pwmLeft = basePWM + correction;
    int pwmRight = basePWM - correction;

    int leftPWM = constrain(basePWM + correction, 0, 255);
    int rightPWM = constrain(basePWM - correction, 0, 255);

    bool leftForward = pwmLeft >= 0;
    bool rightForward = pwmRight >= 0;

    pwmLeft = constrain(abs(pwmLeft), 0, 255);
    pwmRight = constrain(abs(pwmRight), 0, 255);

    digitalWrite(mot1_dir, leftForward ? HIGH : LOW);
    digitalWrite(mot2_dir, rightForward ? LOW : HIGH);
    analogWrite(mot1_pwm, pwmLeft);
    analogWrite(mot2_pwm, pwmRight);
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

// === Precise Turns ===
void MotorController::startTurn(char direction, IMU* imu, PIDController* turnPID) {
    imu->update();
    float startYaw = imu->yaw();
    float delta    = (direction == 'R') ? +90.0f : -90.0f;
    turnTargetYaw  = wrap180(startYaw + delta);
    turnInProgress = true;
    turnStartTime  = millis();
    turnPID->reset();
}

bool MotorController::updateTurn(IMU* imu, float dt) {
    if (!turnInProgress) return true;
    imu->update();
    float err = wrap180(turnTargetYaw - imu->yaw());
    float corr = turnPID->compute(0.0f, -err, dt);
    int pwm = constrain((int)fabs(corr), 0, 255);

    if (err > 0) spinCCW(pwm);
    else         spinCW(pwm);

    if (millis() - turnStartTime >= TURN_DURATION_MS) {
        stop();
        turnInProgress = false;
        return true;
    }
    return false;
}

void MotorController::startupTurn(IMU* imu, PIDController* turnPID, float dt, states& currentState) {
    if (!startupInitiated) {
        imu->update();
        startupYaw = imu->yaw();
        startTurn('R', imu, turnPID);
        startupInitiated = true;
        return;
    }

    if (updateTurn(imu, dt)) {
        imu->update();
        rotationOffset = wrap180(imu->yaw() - startupYaw);
        startupInitiated = false;
        waitStart = millis();
        currentState = WAIT_FOR_ROTATION;
    }
}

void MotorController::waitForRotation(IMU* imu, PIDController* turnPID, states& currentState) {
    if (millis() - waitStart >= 5000) {
        char dir = (rotationOffset > 0) ? 'R' : 'L';
        startTurn(dir, imu, turnPID);
        currentState = RETURN_TO_HEADING;
    }
}

void MotorController::returnToHeading(IMU* imu, PIDController* turnPID, float dt, states& currentState) {
    if (updateTurn(imu, dt)) {
		if (rotationRound < 1) {
			rotationRound++;
            currentState = WAIT_FOR_ROTATION;
            waitStart = millis();
		} else {
        currentState = WALL_APPROACH;
		}
    }
}

void MotorController::wallApproach(LidarSensor* lidar, PIDController* DistancePID, float dt, states* currentState, IMU* imu, PIDController* headingPID) {
    if (!wallApproachActive) {
        wallApproachActive = true;
        wallApproachStart  = millis();
        for (int i = 0; i < 5; ++i) {
            wallDistanceBuffer[i] = lidar->getFrontDistance();
        }
        wallBufferIndex = 0;
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
    for (float v : wallDistanceBuffer) sum += v;
    float avg = sum / 5.0f;
    float error = 100.0f - avg;

    if (fabs(error) <= 3.0f) {
        stop();
        return;
    }

    float pidOut = DistancePID->compute(0.0f, -error, dt);
    int   pwm    = constrain((int)fabs(pidOut), 0, 255);
    float basePWM = (error > 0.0f) ? -pwm : +pwm;

    driveStraightIMU(imu, headingPID, dt, basePWM);
}

// === Command Chain Control ===
void MotorController::startCommandChain(const char* cmd) {
    strncpy(commandBuffer, cmd, sizeof(commandBuffer) - 1);
    commandBuffer[sizeof(commandBuffer) - 1] = '\0';
    commandIndex = 0;
    commandActive = true;
    moveInProgress = false;
    resetInternalState();
}

void MotorController::processCommandStep(PIDController* turnPID, PIDController* headingPID, Encoder* encoder, IMU* imu, states* currentState, float dt) {
    if (!commandActive) {
        stop();
        *currentState = COMPLETE;
        return;
    }
    if (commandIndex >= sizeof(commandBuffer) - 1 || commandBuffer[commandIndex] == '\0') {
        commandActive = false;
        stop();
        *currentState = COMPLETE;
        return;
    }

    char currentCmd = commandBuffer[commandIndex];

    if (!moveInProgress) {
        if (currentCmd == 'F') {
            forwardRunLength = 1;
			int checkIndex = commandIndex + 1;

            while (checkIndex < sizeof(commandBuffer) - 1 &&    // Don't go past buffer
                   commandBuffer[checkIndex] != '\0' &&          // Stop at null terminator
                   commandBuffer[checkIndex] == 'F' &&           // Check if it's an F
                   forwardRunLength < 10) {                      // Reasonable limit
                forwardRunLength++;
                checkIndex++;
            }

            float cellsToMove = forwardRunLength;
            float wheelCircumference = 2.0f * PI * RADIUS;
            float distanceToMove = cellsToMove * CELL_DISTANCE;
            float rotations = distanceToMove / wheelCircumference;
            forwardTargetTicks = (unsigned long)round(rotations * TICKS_PER_REV);

            moveInProgress = true;
            moveStartTime = millis();

            imu->update();
            headingTarget = imu->yaw();

            headingPID->reset();
            encoder->reset();
        }
        else if (currentCmd=='R' || currentCmd=='L') {
            startTurn(currentCmd, imu, turnPID);
            moveInProgress = true;
        }
        else {
            commandIndex++;
        }
    }
    else {
        if (currentCmd == 'F') {
			long currentTicks = encoder->getTicks();
			unsigned long elapsed = millis() - moveStartTime;

			if (currentTicks < forwardTargetTicks && elapsed < MOVE_TIMEOUT) {
				driveStraightIMU(imu, headingPID, dt, DEFAULT_FORWARD_PWM);
			}
        	else {
            	stop();
            	moveInProgress = false;
				commandIndex += forwardRunLength;
				forwardRunLength = 0;
            	forwardTargetTicks = 0;
	    	}
    	}
		else if (currentCmd == 'R' || currentCmd == 'L') {
			if (updateTurn(imu, dt)) {
				stop();
            	moveInProgress = false;
            	commandIndex++;
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
