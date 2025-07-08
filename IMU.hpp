#ifndef IMU_HPP
#define IMU_HPP

#include "MPU6050.h"
#include <Wire.h>

namespace mtrn3100 {

// ────────────── Simple IMU class (yaw only) ──────────────
class IMU {
public:
    IMU() : _addr(0x68) {}

    /* Call once in setup() while the IMU is stationary */
    void calibrate(uint16_t samples = 200) {
        float sumZ = 0;
        for (uint16_t i = 0; i < samples; ++i) {
            _readRaw();
            sumZ += _rawGyroZ;
            delay(2);                      // ~500 Hz sampling
        }
        _gyroBiasZ = sumZ / samples;       // bias/error
    }

    /* Call every loop() */
    void update() {
        _readRaw();
        unsigned long now = micros();
        float dt = (now - _prevMicros) * 1e-6f;   // seconds
        _prevMicros = now;

        float gz = (_rawGyroZ - _gyroBiasZ);      // °/s bias-corrected
        _yaw += gz * dt;                          // integrate to °

        /* keep yaw in [-180, 180] for readability */
        if (_yaw > 180.0f)  _yaw -= 360.0f;
        if (_yaw < -180.0f) _yaw += 360.0f;
    }

    float yaw() const { return _yaw; }
    void begin() {
      Wire.begin();

      /* Wake up MPU-6050 (clear sleep bit) */
      Wire.beginTransmission(0x68);
      Wire.write(0x6B);
      Wire.write(0);
      Wire.endTransmission();
      delay(100);
    }

private:
    const uint8_t _addr;

    /* Raw sensor readings (scaled to physical units) */
    float _rawGyroZ;

    /* Bias (error) */
    float _gyroBiasZ = 0;

    /* State */
    float _yaw = 0;                 // integrated heading (°)
    unsigned long _prevMicros = 0;

    /* Low-level read: accel & gyro, but we only need gyro Z */
    void _readRaw() {
        Wire.beginTransmission(_addr);
        Wire.write(0x43);           // GYRO_XOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom(_addr, 6, true);

        Wire.read(); Wire.read();   // skip Gx
        Wire.read(); Wire.read();   // skip Gy
        int16_t gzRaw = (Wire.read() << 8) | Wire.read();

        _rawGyroZ = gzRaw / 131.0f; // ±250 °/s → scale = 131
    }
};

} // namespace mtrn3100

#endif