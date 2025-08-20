#include "IMU.hpp"
#include <Wire.h>

IMU::IMU()
: _addr(0x68),
  _rawGyroZ(0.0f),
  _gyroBiasZ(0.0f),
  _yaw(0.0f),
  _yawZeroDeg(0.0f),
  _prevMicros(0),
  _hasPrevTime(false)
{}

void IMU::begin() {
    Wire.beginTransmission(_addr);
    Wire.write(0x6B);      // PWR_MGMT_1
    Wire.write(0);         // wake up device
    Wire.endTransmission();
}

void IMU::calibrate(uint16_t samples) {
    // Average raw gyro Z to estimate bias; keep robot still
    float sumZ = 0.0f;
    for (uint16_t i = 0; i < samples; ++i) {
        _readRaw();
        sumZ += _rawGyroZ;
        delay(1); // small spacing helps stability
    }
    _gyroBiasZ = sumZ / (float)samples;
}

void IMU::update() {
    _readRaw();

    unsigned long now = micros();
    if (!_hasPrevTime) {
        _prevMicros = now;
        _hasPrevTime = true;
        return; // no integration on first call
    }

    float dt = (now - _prevMicros) * 1e-6f; // seconds
    _prevMicros = now;

    // Guard against huge or negative dt (USB pauses etc.)
    if (dt <= 0.0f || dt > 0.05f) { // >50ms? skip to avoid big jumps
        return;
    }

    // Integrate gyro (deg/s * s = deg)
    const float gz = (_rawGyroZ - _gyroBiasZ);
    _yaw += gz * dt;

    // Wrap to [-180, 180]
    if (_yaw > 180.0f)  _yaw -= 360.0f;
    if (_yaw < -180.0f) _yaw += 360.0f;
}

float IMU::yaw() const {
    return _yaw;
}

float IMU::yawRel() const {
    // Relative to the last zeroYaw()
    float rel = _yaw - _yawZeroDeg;
    // Wrap to [-180, 180]
    while (rel > 180.f) rel -= 360.f;
    while (rel < -180.f) rel += 360.f;
    return rel;
}

void IMU::zeroYaw() {
    // Soft-zero: make current absolute yaw become 0 in yawRel()
    _yawZeroDeg = _yaw;
}

void IMU::setYaw(float deg) {
    // Force absolute yaw (mainly for testing)
    // Also keeps yawRel continuous by shifting zero with it.
    float oldYaw = _yaw;
    _yaw = deg;
    // keep relative zeroed direction the same
    _yawZeroDeg += (_yaw - oldYaw);
    // Wrap both
    if (_yaw > 180.0f)  _yaw -= 360.0f;
    if (_yaw < -180.0f) _yaw += 360.0f;
    if (_yawZeroDeg > 180.0f)  _yawZeroDeg -= 360.0f;
    if (_yawZeroDeg < -180.0f) _yawZeroDeg += 360.0f;
}

void IMU::_readRaw() {
    // Read GYRO_X/Y/Z starting at 0x43 (MPU-6050)
    Wire.beginTransmission(_addr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, static_cast<uint8_t>(6), static_cast<uint8_t>(true));

    // Skip X, Y
    (void)Wire.read(); (void)Wire.read();
    (void)Wire.read(); (void)Wire.read();

    // Read Z high/low
    int16_t gzRaw = (Wire.read() << 8) | Wire.read();

    // Sensitivity 131 LSB/(deg/s) at ±250 dps
    _rawGyroZ = gzRaw / 131.0f; // deg/s
}
