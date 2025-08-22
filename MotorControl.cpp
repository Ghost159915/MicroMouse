#include "MotorControl.hpp"
#include "LidarSensor.hpp"
#include "Encoder.hpp"
#include "KalmanFilter.hpp"

static constexpr float kEncGain   = 0.50f;  // ticks -> steering scale
static constexpr float kIMUClamp  = 45.0f;  // deg clamp before PID
static constexpr float kWallClamp = 80.0f;  // max PWM from wall PID
static constexpr float kSteerMax  = 60.0f;  // overall steering clamp
static constexpr float kSpeedMax  = 30.0f;  // (optional) common speed clamp
static constexpr int   kMinPWM    = 50;     // overcome static friction
static constexpr int   kMaxPWM    = 255;

static constexpr float LIDAR_EMA_ALPHA = 0.15f;

extern LidarSensor lidar;

MotorController::MotorController(int m1_pwm, int m1_dir, int m2_pwm, int m2_dir)
    : mot1_pwm(m1_pwm), mot1_dir(m1_dir),
    mot2_pwm(m2_pwm), mot2_dir(m2_dir), commandIndex(0), commandActive(false),
    moveInProgress(false), moveStartTime(0), turnInProgress(false), turnTargetYaw(0.0f),
    turnStartTime(0), startupInitiated(false), startupYaw(0.0f),
    waitStart(0), rotationOffset(0.0f), wallBufferIndex(0)
{
    for (int i = 0; i < 5; ++i) wallDistanceBuffer[i] = 100.0f;
}

static KalmanFilter yawKF(/*Q*/0.7f, /*R*/8.0f, /*P0*/1.0f, /*x0*/0.0f);


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
    // Target in degrees: R (CW) = -90, L (CCW) = +90
    turnTargetYaw  = (direction == 'R') ? -93.0f : +87.0f;
    turnInProgress = true;
    turnStartTime  = millis();

    // Reset controllers & sensors for a RELATIVE frame
    turnPID->reset();
    imu->update();
    imu->zeroYaw();               // so imu->yawRel() starts at 0 deg

    // Init Kalman filter state at 0 deg in this relative frame
    yawKF.init(0.0f);
}

void MotorController::startTurn45(char direction, IMU* imu, PIDController* turnPID) {
    // Target in degrees: R (CW) = -90, L (CCW) = +90
    turnTargetYaw  = (direction == 'r') ? -45.0f : +45.0f;
    turnInProgress = true;
    turnStartTime  = millis();

    // Reset controllers & sensors for a RELATIVE frame
    turnPID->reset();
    imu->update();
    imu->zeroYaw();               // so imu->yawRel() starts at 0 deg

    // Init Kalman filter state at 0 deg in this relative frame
    yawKF.init(0.0f);
}

bool MotorController::updateTurn(IMU* imu, PIDController* turnPID, float dt, DualEncoder* encoder) {
    if (!turnInProgress) return true;

    // per-turn state
    static bool  inited = false;
    static long  prevL = 0, prevR = 0;
    static float prevFused = 0.0f;
    static float rateLP = 0.0f;           // NEW: low-pass rate
    static int   lastPWM = 0;
    static int   lastDir = 0;             // NEW: +1 CCW, -1 CW, 0 none
    static unsigned long dwellStart = 0;

    if (!inited) {
        prevL = encoder->getLeftTicks();
        prevR = encoder->getRightTicks();
        prevFused = 0.0f;
        rateLP = 0.0f;
        lastPWM = 0;
        lastDir = 0;
        dwellStart = 0;
        inited = true;
    }

    // --- encoder incremental yaw (deg) ---
    long L = encoder->getLeftTicks(), R = encoder->getRightTicks();
    long dL = L - prevL, dR = R - prevR;
    prevL = L; prevR = R;

    float leftDist  = (2.0f * PI * RADIUS) * ((float)dL / (float)TICKS_PER_REV);
    float rightDist = (2.0f * PI * RADIUS) * ((float)dR / (float)TICKS_PER_REV);
    float dYawEnc   = ((rightDist - leftDist) / WHEEL_BASE) * (180.0f / PI); // CW negative

    // --- KF: predict with enc, update with IMU yawRel ---
    yawKF.predict(dYawEnc);
    imu->update();
    yawKF.update(imu->yawRel());

    float fusedYaw = yawKF.getState();
    while (fusedYaw > 180.f) fusedYaw -= 360.f;
    while (fusedYaw < -180.f) fusedYaw += 360.f;

    // --- control: PD with derivative on measurement (rate damping) ---
    float err = (turnTargetYaw - fusedYaw);        // deg

    float instRate = (dt > 1e-4f) ? (fusedYaw - prevFused) / dt : 0.0f; // deg/s
    prevFused = fusedYaw;
    const float RATE_ALPHA = 0.2f;                            // 0.15–0.3 works well
    rateLP = RATE_ALPHA * instRate + (1.0f - RATE_ALPHA) * rateLP;

    // Use your PID's P only; do D here on measurement for robustness
    float u_p  = turnPID->compute(0.0f, -err, dt); // with Ki=0 in the PID
    const float KD_RATE = 0.55f;                   // tune 0.45–0.75
    float u = u_p - KD_RATE * rateLP;

    // tiny predictive cutoff (very conservative)
    // const float LOOKAHEAD_S = 0.06f, ANG_SOFT = 0.8f;
    // if (fabsf(err) < LOOKAHEAD_S * fabsf(fusedRate) + ANG_SOFT) {
    //     stop(); turnInProgress = false; inited = false; return true;
    // }

    // --- PWM shaping (don’t starve torque near target) ---
    auto pwmCapFromErr = [](float eAbs)->int {
        if (eAbs >= 60.f) return 130;
        if (eAbs >= 30.f) return 110;
        if (eAbs >= 15.f) return 90;
        if (eAbs >=  6.f) return 75;
        return 65;
    };
    int pwmCap = pwmCapFromErr(fabsf(err));

    int dynMin = (fabsf(err) < 12.0f) ? 40 : 65;               // was hard 65
    int pwm    = constrain((int)fabsf(u), dynMin, min(pwmCap, kMaxPWM));

    int cmdDir = (u > 0) ? +1 : -1;

    const int FLIP_THRESH = 50;                         // was 85 (too high)
    const int BRAKE_THRESH = 45;
                        // require strong command to reverse
    if (lastDir != 0 && cmdDir != lastDir) {
        if (pwm < BRAKE_THRESH) {
            // one-frame brake/coast to bleed speed without flipping
            analogWrite(mot1_pwm, 0);
            analogWrite(mot2_pwm, 0);
            lastPWM = 0;
            // don't update lastDir; skip driving this frame
            // dwell logic still runs below
        } else if (pwm < FLIP_THRESH) {
            // allow gentle flip, but cap the pwm so it's not a snap
            pwm = FLIP_THRESH;
        }
    }

    // slew limit
    const int maxStep = 10;
    if (pwm > lastPWM + maxStep) pwm = lastPWM + maxStep;
    else if (pwm < lastPWM - maxStep) pwm = lastPWM - maxStep;
    lastPWM = pwm;

    // drive
    if (cmdDir > 0) { spinCCW(pwm); } else { spinCW(pwm); }
    lastDir = cmdDir;

    // --- stop when in window for 150 ms (dwell) ---
    const float ANG_EPS = 2.0f;           // a touch tighter since cutoff removed
    const float RATE_EPS = 20.0f;
    if (fabsf(err) < ANG_EPS && fabsf(rateLP) < RATE_EPS) {
        if (dwellStart == 0) dwellStart = millis();
        if (millis() - dwellStart >= 150) {
            stop(); turnInProgress = false; inited = false; return true;
        }
    } else {
        dwellStart = 0;
    }

    // safety
    if (millis() - turnStartTime > 15000) {
        stop(); turnInProgress = false; inited = false; return true;
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

void MotorController::processCommandStep(PIDController* turnPID, PIDController* headingPID, PIDController* wallPID, 
                                         DualEncoder* encoder, IMU* imu, states* currentState,  LidarSensor* lidar, float dt) {
    char cmd = commandBuffer[commandIndex];

    // if (!commandActive || commandBuffer[commandIndex] == '\0') {
    //     stop();
    //     *currentState = COMPLETE;
    //     return;
    // }

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
        else if (cmd == 'f') {
            forwardRunLength = 1;
            while (commandBuffer[commandIndex + forwardRunLength] == 'f' &&
                commandIndex + forwardRunLength < sizeof(commandBuffer) - 1 &&
                forwardRunLength < 10) {
                forwardRunLength++;
            }

            float totalDist = forwardRunLength * (CELL_DISTANCE / 2);
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
        else if (cmd == 'r' || cmd == 'l') {
            startTurn45(cmd, imu, turnPID);
            moveInProgress = true;
        }
    }
    else {
        if (cmd == 'F') {
            long currentTicks = encoder->getAverageTicks();
            if (currentTicks < forwardTargetTicks && lidar->getFrontDistance() > 55) {
                driveStraightDualEncoder(encoder, imu, headingPID, wallPID, dt, DEFAULT_FORWARD_PWM);
            } else {
                stop();
                commandIndex += forwardRunLength;
                moveInProgress = false;
            }
        }
        else if (cmd == 'f') {
            long currentTicks = encoder->getAverageTicks();
            if (currentTicks < forwardTargetTicks && lidar->getFrontDistance() > 55) {
                driveStraightDualEncoder(encoder, imu, headingPID, wallPID, dt, DEFAULT_FORWARD_PWM);
            } else {
                stop();
                commandIndex += forwardRunLength;
                moveInProgress = false;
            }
        }
        else if (cmd == 'R' || cmd == 'r'|| cmd == 'L'|| cmd == 'l') {
            if (updateTurn(imu, turnPID, dt, encoder)) {
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
                                              PIDController* headingPID, PIDController* wallPID, float dt, float basePWM) {

    bool movingForward = basePWM >= 0;
    int absPWM = abs(basePWM);

	long leftTicks = encoder->getLeftTicks();
    long rightTicks = encoder->getRightTicks();
    long tickDiff = leftTicks - rightTicks;

    long leftDistance = lidar.getLeftDistance();
    long rightDistance = lidar.getRightDistance();

    // --- Exponential Moving Average (EMA) smoothing of side distances ---
    static bool emaInit = false;
    static float leftEMA = 0.0f, rightEMA = 0.0f;

    if (!emaInit) {
        leftEMA  = (float)leftDistance;
        rightEMA = (float)rightDistance;
        emaInit = true;
    }

    // EMA update
    leftEMA  = leftEMA  + LIDAR_EMA_ALPHA * ((float)leftDistance  - leftEMA);
    rightEMA = rightEMA + LIDAR_EMA_ALPHA * ((float)rightDistance - rightEMA);

    // Use smoothed values for the rest of this function
    leftDistance  = (long)leftEMA;
    rightDistance = (long)rightEMA;

    float error = 0.0f;
    float error2 = 0.0f;
    float lidarCorrection = 0.0f;
    float distanceDiff = 0.0f;
    
    imu->update();

    float headingError = wrap180(headingTarget - imu->yaw());
    float encoderCorrection = tickDiff * 0.6;  
    float imuCorrection = headingPID->compute(0.0, -headingError, dt);

    if (isWallBoth(&lidar)) {
        distanceDiff = leftDistance - rightDistance;
        lidarCorrection = wallPID->compute(0.0, -distanceDiff, dt);
    }
    else if (isWallLeft(&lidar) && !(isWallRight(&lidar))) {
        error = leftDistance - 55;
        lidarCorrection = wallPID->compute(0.0, error, dt);
    }
    else if (isWallRight(&lidar) && !(isWallLeft(&lidar))) {
        error2 = rightDistance - 55;
        lidarCorrection = wallPID->compute(0.0, error2, dt);
    }
    // else if (isWallRight(&lidar) && isWallFront(&lidar)) {
        
    // }
    else {
        lidarCorrection = 0.0f;
        wallPID->reset(); 
    }

    float totalCorrection = (encoderCorrection * 0.4) + (imuCorrection * 0.1) + (lidarCorrection * 0.6);
    totalCorrection = constrain(totalCorrection, -50, 50);
    
    int leftPWM = basePWM - totalCorrection;
    int rightPWM = basePWM + totalCorrection;
    
    if (leftPWM > 0 && leftPWM < 60) leftPWM = 60;
    if (rightPWM > 0 && rightPWM < 60) rightPWM = 60;
    
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);
    
    digitalWrite(mot1_dir, LOW);  
    digitalWrite(mot2_dir, HIGH);   
    analogWrite(mot1_pwm, leftPWM);
    analogWrite(mot2_pwm, rightPWM);
}

char MotorController::getCurrentCommand() {
    return commandBuffer[commandIndex];
}

bool MotorController::isWallRight(LidarSensor* lidar){
    return (lidar->getRightDistance() < 100);
}

bool MotorController::isWallLeft(LidarSensor* lidar){
    return (lidar->getLeftDistance() < 100);
}

bool MotorController::isWallBoth(LidarSensor* lidar){
    return (lidar->getRightDistance() < 100 && lidar->getLeftDistance() < 100);
}

bool MotorController::isWallfront(LidarSensor* lidar) {
        return (lidar->getFrontDistance() < 100);
}

bool MotorController::getCommandActive() {
    return commandActive;
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