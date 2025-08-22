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
    unsigned long now = millis();
    float dt = 0.0f;
    if (!hasLast) {
        hasLast = true;
    } else {
        dt = (now - lastMillis) * 1e-3f;      // seconds
        if (dt > 0.25f) dt = 0.25f;           // guard against long pauses
    }
    lastMillis = now;

    // integrate bias-corrected gyro rate
    float gz = gzDps();                        // deg/s
    yawDeg = wrap180(yawDeg + gz * dt);
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

void IMU::calibrateGyroBias(unsigned long ms) {
    unsigned long t0 = millis();
    long   n   = 0;
    double acc = 0.0;
    // small delay loop to sample without blocking I2C queues too hard
    while (millis() - t0 < ms) {
        acc += readGzDps();  // raw (unbiased) rate
        n++;
        delay(2);
    }
    biasZ = (n > 0) ? (float)(acc / n) : 0.0f;
}

float IMU::gzDps() {
    return readGzDps() - biasZ;
}