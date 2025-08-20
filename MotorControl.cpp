#include "MotorControl.hpp"

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

// void MotorController::startupTurn(IMU* imu, PIDController* turnPID, float dt, states& currentState) {
//     if (!startupInitiated) {
//         startupYaw = 0;
//         startTurn('R', imu, turnPID);
//         startupInitiated = true;
//         return;
//     }

//     if (updateTurn(imu, turnPID, dt)) {
//         imu->update();
//         rotationOffset = wrap180(imu->yaw() - startupYaw);
//         startupInitiated = false;
//         waitStart = millis();
//         currentState = WAIT_FOR_ROTATION;
//     }
// }

// void MotorController::waitForRotation(IMU* imu, PIDController* turnPID, states& currentState) {
//     if (millis() - waitStart >= 5000) {
//         currentState = RETURN_TO_HEADING;
//     }
// }

// void MotorController::returnToHeading(IMU* imu, PIDController* turnPID, float dt, states& currentState) {
//     static bool returnStarted = false;

//     if (!returnStarted) {
//         // First time in this state - calculate how to get back
//         imu->update();
//         float currentYaw = imu->yaw();
//         float angleDifference = wrap180(startupYaw - currentYaw);

//         // Set up turn to return to original heading
//         turnTargetYaw = startupYaw;
//         turnInProgress = true;
//         turnStartTime = millis();
//         turnPID->reset();

//         returnStarted = true;
//         return;
//     }

//     if (updateTurn(imu, turnPID, dt)) {
//         // Turn complete
//         returnStarted = false;

//         if (rotationRound < 1) {
//             rotationRound++;
//             waitStart = millis();
//             currentState = WAIT_FOR_ROTATION;
//         } else {
//             currentState = WALL_APPROACH;
//         }
//     }
// }

// void MotorController::wallApproachDirect(LidarSensor* lidar, PIDController* DistancePID,
//                                         float dt, states* currentState, IMU* imu,
//                                         PIDController* headingPID, DualEncoder* encoder) {

// 	if (!wallApproachActive) {
// 		wallApproachActive = true;
// 		wallApproachStart = millis();

// 		imu->update();
// 		float headinTarget = 0.0;
// 		encoder->reset();

// 		float initialDist = lidar->getFrontDistance();
// 		for (int i = 0; i < 5; i++) {
// 			wallDistanceBuffer[i] = lidar->getFrontDistance();
// 		}
// 		wallBufferIndex = 0;

//         DistancePID->reset();
//         headingPID->reset();
// 	}

//     if (millis() - wallApproachStart >= WALL_APPROACH_MS) {
//         stop();
//         wallApproachActive = false;
//         *currentState = COMMAND_CHAIN;
//         return;
//     }

//     float rawDist = lidar->getFrontDistance();
//     wallDistanceBuffer[wallBufferIndex] = rawDist;
//     wallBufferIndex = (wallBufferIndex + 1) % 5;

// 	float sum = 0.0f;
// 	for (int i = 0; i < 5; i++) {
//        	sum += wallDistanceBuffer[i];
//    	}
//    	float avgDistance = sum / 5.0f;

//     float distanceError = avgDistance - 100.0f;

//     if (fabs(distanceError) < 5.0f) {
//         stop();
//         return;
//     }

//     float distancePIDOutput = DistancePID->compute(0.0f, distanceError, dt);
//     distancePIDOutput = constrain(distancePIDOutput, -150, 150);
//     int basePWM = -(int)distancePIDOutput;
//  	imu->update();
//     float currentHeading = imu->yaw();
//     float headingError = wrap180(headingTarget - currentHeading);

//     if (fabs(headingError) > 10.0f) {
//         Serial.print("Large heading error detected: ");
//         Serial.println(headingError);
//     }

//     driveStraightDualEncoder(encoder, imu, headingPID, dt, basePWM);

// }