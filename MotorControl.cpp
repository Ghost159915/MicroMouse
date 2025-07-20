#include "MotorControl.hpp"

MotorController::MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir)
    : mot1_pwm(m1_pwm), mot1_dir(m1_dir),
      mot2_pwm(m2_pwm), mot2_dir(m2_dir),
      wallApproachActive(false), wallApproachStart(0),
      commandIndex(0), commandActive(false), moveInProgress(false), moveStartTime(0),
	  turnInProgress(false), turnTargetYaw(0.0f), turnPID(nullptr) {}

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

    int leftPWM = constrain(basePWM + correction, 0, 255);
    int rightPWM = constrain(basePWM - correction, 0, 255);

    digitalWrite(mot1_dir, HIGH);
    digitalWrite(mot2_dir, LOW);
    analogWrite(mot1_pwm, leftPWM);
    analogWrite(mot2_pwm, rightPWM);
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
    if (fabs(err) < 1.0f) {
        stop();
        turnInProgress = false;
        return true;
    }
    return false;
}

void MotorController::PIDturn90CW(IMU* imu, PIDController* turnPID) {
    startTurn('R', imu, turnPID);
}

void MotorController::PIDturn90CCW(IMU* imu, PIDController* turnPID) {
    startTurn('L', imu, turnPID);
}

// === Return to Heading ===
void MotorController::returnToHeading(IMU* imu, PIDController* turnPID, float targetOffset) {
    turnPID->reset();
    imu->update();
    float targetHeading = wrap180(imu->yaw() - targetOffset);
    unsigned long lastTime = millis();

    while (true) {
        imu->update();
        unsigned long now = millis();
        float dt = max((now - lastTime) / 1000.0f, 0.001f);
        lastTime = now;
        float error = wrap180(targetHeading - imu->yaw());
        if (abs(error) < 0.5) { stop(); break; }
        float control = turnPID->compute(0.0, -error, dt);
        int pwm = constrain(abs(control), 0, 255);
        if (error > 0) spinCCW(pwm);
        else spinCW(pwm);
    }

    imu->update();
    stop();
}

// === Wait for Rotation ===
float MotorController::waitForRotation(IMU* imu) {
    stop();
    imu->update();
    float startHeading = imu->yaw();
    unsigned long startMillis = millis();
    while (millis() - startMillis < 10000) {
        imu->update();
        //delay(5);
    }
    imu->update();
    float endHeading = imu->yaw();
    return wrap180(endHeading - startHeading);
}

// === Wall Following ===


void MotorController::wallApproach(LidarSensor* lidar, PIDController* DistancePID, float dt, states* currentState, IMU* imu, PIDController* headingPID) {
    if (!wallApproachActive) {
        Serial.println("WALL_APPROACH: Starting 20-second interaction");
        wallApproachActive = true;
        wallApproachStart = millis();
    }

    if (millis() - wallApproachStart >= 20000) {
        stop();
        wallApproachActive = false;
        *currentState = COMMAND_CHAIN;
        return;
    }
    // === Moving average smoothing ===

    const int WINDOW_SIZE = 5;

    static float distanceWindow[WINDOW_SIZE] = {100, 100, 100, 100, 100};
    static int windowIndex = 0;

    float rawDistance = lidar->getFrontDistance();
    distanceWindow[windowIndex] = rawDistance;
    windowIndex = (windowIndex + 1) % WINDOW_SIZE;

    float smoothedDistance = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; ++i) {
        smoothedDistance += distanceWindow[i];
    }
    smoothedDistance /= WINDOW_SIZE;

    float error = 100.0 - smoothedDistance;

    // === Deadzone handling ===
    if (abs(error) < 3.0) {
        stop();
    } else {
        float control = DistancePID->compute(0.0f, -error, dt);
        float pwm = constrain(abs(control), 0, 255);
        float basePWM = (error > 0) ? -pwm : pwm;
        driveStraightIMU(imu, headingPID, dt, basePWM);
    }
}

// === Command Chain Control ===
void MotorController::startCommandChain(const char* cmd) {
    strncpy(commandBuffer, cmd, sizeof(commandBuffer) - 1);
    commandBuffer[sizeof(commandBuffer) - 1] = '\0';
    commandIndex = 0;
    commandActive = true;
    moveInProgress = false;
    Serial.println("COMMAND_CHAIN: Started");
}

bool MotorController::isCommandActive() {
    return commandActive;
}

void MotorController::processCommandStep(PIDController* turnPID, PIDController* headingPID, Encoder* encoder, IMU* imu, states* currentState, float dt) {
    Serial.print("PROCESS_COMMAND: active=");
    Serial.println(commandActive ? "true" : "false");

    if (!commandActive) {
        stop();
        *currentState = COMPLETE;
        return;
    }

    if (commandBuffer[commandIndex] == '\0') {
        commandActive = false;
        stop();
        *currentState = COMPLETE;
        Serial.println("COMMAND_CHAIN: Finished!");
        return;
    }

    char currentCmd = commandBuffer[commandIndex];

    if (!moveInProgress) {
        encoder->reset();
        Serial.print("Executing command letter: "); Serial.println(currentCmd);

        if (currentCmd == 'F') {
            moveInProgress = true;
            moveStartTime = millis();
            imu->update();
            headingTarget = imu->yaw();  // Set target heading
            headingPID->reset();         // Reset heading PID
        }
        else if (currentCmd=='R' || currentCmd=='L') {
            if (!turnInProgress) {
				startTurn(currentCmd, imu, turnPID);
			} else if (updateTurn(imu, dt)) {
				commandIndex++;
			}
        }

        else {
            commandIndex++;
        }
    }
    else {
        float total_count = CELL_DISTANCE / (2 * PI * RADIUS) * TICKS_PER_REV;
        if (encoder->getTicks() < total_count && (millis() - moveStartTime) < MOVE_TIMEOUT) {
            driveStraightIMU(imu, headingPID, dt, 150);
        }
        else {
            stop();
            moveInProgress = false;
            commandIndex++;
        }
    }
}
