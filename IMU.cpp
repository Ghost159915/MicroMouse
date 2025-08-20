#include "IMU.hpp"

// MPU-6050 default gyro full-scale ±250 dps -> 131 LSB/(deg/s)
static inline float lsbToDps(int16_t raw) { return raw / 131.0f; }

IMU::IMU()
: addr(0x68),
  yawDeg(0.0f),
  yawZeroDeg(0.0f),
  lastMillis(0),
  hasLast(false)
{}

void IMU::begin() {
    // Wake device (PWR_MGMT_1 = 0)
    Wire.beginTransmission(addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();
}

float IMU::readGzDps() {
    // Read 6 bytes starting at GYRO_XOUT_H (0x43), discard X/Y, keep Z
    Wire.beginTransmission(addr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)6, (uint8_t)true);

    // X
    (void)Wire.read(); (void)Wire.read();
    // Y
    (void)Wire.read(); (void)Wire.read();
    // Z
    int16_t gzRaw = (Wire.read() << 8) | Wire.read();
    return lsbToDps(gzRaw); // deg/s
}

void IMU::update() {
    float gz = readGzDps();                 // deg/s about Z

    unsigned long now = millis();
    if (!hasLast) {
        lastMillis = now;
        hasLast = true;
        return;                              // no integration on first call
    }
    float dt = (now - lastMillis) / 1000.0f; // seconds
    lastMillis = now;

    // Integrate yaw (no bias removal; will drift over time)
    yawDeg += gz * dt;

    // Wrap to [-180, 180]
    yawDeg = wrap180(yawDeg);
}

float IMU::yaw() const {
    return yawDeg;
}

float IMU::yawRel() const {
    return wrap180(yawDeg - yawZeroDeg);
}

void IMU::zeroYaw() {
    yawZeroDeg = yawDeg;
}

void IMU::setYaw(float deg) {
    // set absolute yaw; keep relative zero continuous
    float old = yawDeg;
    yawDeg = wrap180(deg);
    yawZeroDeg = wrap180(yawZeroDeg + (yawDeg - old));
}

float IMU::wrap180(float a) {
    while (a > 180.f) a -= 360.f;
    while (a < -180.f) a += 360.f;
    return a;
}
