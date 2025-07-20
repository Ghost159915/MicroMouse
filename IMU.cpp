#include "IMU.hpp"
#include <Wire.h>

IMU::IMU() : _addr(0x68), _rawGyroZ(0), _gyroBiasZ(0), _yaw(0), _prevMicros(0) {}

void IMU::begin() {
    Wire.beginTransmission(_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();
    delay(100);
}

void IMU::calibrate(uint16_t samples) {
    float sumZ = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        _readRaw();
        sumZ += _rawGyroZ;
        delay(2);
    }
    _gyroBiasZ = sumZ / samples;
}

void IMU::update() {
    _readRaw();
    unsigned long now = micros();
    float dt = (now - _prevMicros) * 1e-6f;
    _prevMicros = now;

    float gz = (_rawGyroZ - _gyroBiasZ);
    _yaw += gz * dt;

    if (_yaw > 180.0f)  _yaw -= 360.0f;
    if (_yaw < -180.0f) _yaw += 360.0f;
}

float IMU::yaw() const {
    return _yaw;
}

void IMU::_readRaw() {
    Wire.beginTransmission(_addr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, static_cast<uint8_t>(6), static_cast<uint8_t>(true));

    Wire.read(); // skip X
    Wire.read();
    Wire.read(); // skip Y
    Wire.read();
    int16_t gzRaw = (Wire.read() << 8) | Wire.read();

    _rawGyroZ = gzRaw / 131.0f;
}
