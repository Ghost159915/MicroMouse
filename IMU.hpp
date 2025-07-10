#ifndef IMU_HPP
#define IMU_HPP

#include "MPU6050.h"
#include <Wire.h>


class IMU {
public:
    IMU() : _addr(0x68) {}

    void calibrate(uint16_t samples = 200) {
        float sumZ = 0;
        for (uint16_t i = 0; i < samples; ++i) {
            _readRaw();
            sumZ += _rawGyroZ;
            delay(2);
        }
        _gyroBiasZ = sumZ / samples;
    }

    void update() {
        _readRaw();
        unsigned long now = micros();
        float dt = (now - _prevMicros) * 1e-6f;
        _prevMicros = now;

        float gz = (_rawGyroZ - _gyroBiasZ);
        _yaw += gz * dt;

        if (_yaw > 180.0f)  _yaw -= 360.0f;
        if (_yaw < -180.0f) _yaw += 360.0f;
    }

    float yaw() const { 
        return _yaw; 
    }
    
    void begin() {
        Wire.beginTransmission(0x68);
        Wire.write(0x6B);
        Wire.write(0);
        Wire.endTransmission();
        delay(100);
    }

private:

    const uint8_t _addr;
    float _rawGyroZ;
    float _gyroBiasZ = 0;
    float _yaw = 0;
    unsigned long _prevMicros = 0;

    void _readRaw() {
        Wire.beginTransmission(_addr);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(_addr, 6, true);

        Wire.read();
        Wire.read();
        Wire.read();
        Wire.read();
        int16_t gzRaw = (Wire.read() << 8) | Wire.read();

        _rawGyroZ = gzRaw / 131.0;
    }
};


#endif