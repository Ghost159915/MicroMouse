#include "Encoder.hpp"

DualEncoder* DualEncoder::instance = nullptr;

// Constants for your robot
const float WHEEL_DIAMETER = 32.0;  // mm
const float WHEELBASE = 100.0;       // mm between wheels
const float TICKS_PER_REV = 700.0;

DualEncoder::DualEncoder(int aL, int bL, int aR, int bR)
    : encA_L(aL), encB_L(bL), encA_R(aR), encB_R(bR), 
      ticksLeft(0), ticksRight(0) {
    instance = this;
}

void DualEncoder::begin() {
    pinMode(encA_L, INPUT_PULLUP);
    pinMode(encB_L, INPUT_PULLUP);
    pinMode(encA_R, INPUT_PULLUP);
    pinMode(encB_R, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(encA_L), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(encA_R), isrRight, RISING);
}

long DualEncoder::getLeftTicks() const { return ticksLeft; }
long DualEncoder::getRightTicks() const { return ticksRight; }

long DualEncoder::getAverageTicks() const { 
    return (ticksLeft + ticksRight) / 2; 
}

long DualEncoder::getDifferentialTicks() const { 
    return ticksRight - ticksLeft; 
}

float DualEncoder::getHeadingChange() const {
    // Calculate heading change in degrees based on differential
    float diff = getDifferentialTicks();
    float wheelCirc = PI * WHEEL_DIAMETER;
    float distDiff = (diff / TICKS_PER_REV) * wheelCirc;
    float angleRad = distDiff / WHEELBASE;
    return angleRad * 180.0 / PI;
}

float DualEncoder::getDriftRatio() const {
    if (ticksRight == 0) return 0;
    return (float)ticksLeft / (float)ticksRight;
}

void DualEncoder::reset() {
    ticksLeft = 0;
    ticksRight = 0;
}

void DualEncoder::resetLeft() { ticksLeft = 0; }
void DualEncoder::resetRight() { ticksRight = 0; }

// --- Encoder.cpp additions (place near the top, after instance pointer) ---
static constexpr int8_t LEFT_B_HIGH_FORWARD  = -1;  // set to -1 if forward gives negative ticks on LEFT
static constexpr int8_t RIGHT_B_HIGH_FORWARD = 1;  // set to -1 if forward gives negative ticks on RIGHT

// --- Replace the two ISRs with the versions below ---
void DualEncoder::isrLeft() {
    if (!instance) return;
    int8_t step = digitalRead(instance->encB_L) ? +1 : -1;   // B=HIGH => +1, B=LOW => -1
    instance->ticksLeft += (LEFT_B_HIGH_FORWARD * step);
}

void DualEncoder::isrRight() {
    if (!instance) return;
    int8_t step = digitalRead(instance->encB_R) ? +1 : -1;   // B=HIGH => +1, B=LOW => -1
    instance->ticksRight += (RIGHT_B_HIGH_FORWARD * step);
}