#include "../include/MotorControl.hpp"

MotorController::MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir)
    : mot1_pwm(m1_pwm), mot1_dir(m1_dir),
      mot2_pwm(m2_pwm), mot2_dir(m2_dir),
      wallApproachActive(false), wallApproachStart(0),
      commandIndex(0), commandActive(false), moveInProgress(false), moveStartTime(0) {}

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

void MotorController::moveForward(int pwmVal) {
    pwmVal = constrain(pwmVal, 0, 255);
    digitalWrite(mot1_dir, HIGH);
    analogWrite(mot1_pwm, pwmVal);
    digitalWrite(mot2_dir, LOW);
    analogWrite(mot2_pwm, pwmVal);
}

void MotorController::moveBackward(int pwmVal) {
    pwmVal = constrain(pwmVal, 0, 255);
    digitalWrite(mot1_dir, LOW);
    analogWrite(mot1_pwm, pwmVal);
    digitalWrite(mot2_dir, HIGH);
    analogWrite(mot2_pwm, pwmVal);
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
void MotorController::PIDturn90CW(IMU* imu, PIDController* turnPID) {
    turnPID->reset();
    imu->update();
    float targetAngle = wrap180(imu->yaw() - 90.0);
    unsigned long lastTime = millis();

    while (true) {
        imu->update();
        unsigned long now = millis();
        float dt = max((now - lastTime) / 1000.0f, 0.001f);
        lastTime = now;
        float error = wrap180(targetAngle - imu->yaw());
        if (abs(error) < 0.5) { stop(); break; }
        float control = turnPID->compute(0.0, -error, dt);
        int pwm = constrain(abs(control), 0, 255);
        if (error > 0) spinCCW(pwm);
        else spinCW(pwm);
        delay(10);
    }
}

void MotorController::PIDturn90CCW(IMU* imu, PIDController* turnPID) {
    turnPID->reset();
    imu->update();
    float targetAngle = wrap180(imu->yaw() + 90.0);
    unsigned long lastTime = millis();

    while (true) {
        imu->update();
        unsigned long now = millis();
        float dt = max((now - lastTime) / 1000.0f, 0.001f);
        lastTime = now;
        float error = wrap180(targetAngle - imu->yaw());
        if (abs(error) < 0.5) { stop(); break; }
        float control = turnPID->compute(0.0, -error, dt);
        int pwm = constrain(abs(control), 0, 255);
        if (error > 0) spinCCW(pwm);
        else spinCW(pwm);
        delay(10);
    }
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
        delay(10);
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
        delay(5);
    }
    imu->update();
    float endHeading = imu->yaw();
    return wrap180(endHeading - startHeading);
}

// === Wall Following ===
void MotorController::wallApproach(LidarSensor* lidar, PIDController* DistancePID, float dt, states* currentState) {
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

    float currentDistance = lidar->getFrontDistance();
    float error = 100.0 - currentDistance;
    float control = DistancePID->compute(0.0, -error, dt);
    int pwm = constrain(abs(control), 0, 255);

    if (error > 3) moveBackward(pwm);
    else if (error < -3) moveForward(pwm);
    else stop();

    Serial.print(" LIDAR: "); Serial.print(currentDistance);
    Serial.print(" Error: "); Serial.print(error);
    Serial.print(" PWM: "); Serial.println(pwm);
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

void MotorController::processCommandStep(PIDController* controller, Encoder* encoder, IMU* imu, states* currentState) {
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
        }
        else if (currentCmd == 'R') {
            PIDturn90CW(imu, controller);
            commandIndex++;
        }
        else if (currentCmd == 'L') {
            PIDturn90CCW(imu, controller);
            commandIndex++;
        }
        else {
            commandIndex++;
        }
    }
    else {
        float total_count = CELL_DISTANCE / (2 * PI * RADIUS) * TICKS_PER_REV;
        if (encoder->getTicks() < total_count && (millis() - moveStartTime) < MOVE_TIMEOUT) {
            moveForward(150);
        }
        else {
            stop();
            moveInProgress = false;
            commandIndex++;
        }
    }
}
