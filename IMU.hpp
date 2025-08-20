#pragma once
#include <Arduino.h>

class IMU {
public:
    IMU();

    void begin();
    void calibrate(uint16_t samples = 1000);
    void update();

    // Absolute fused gyro yaw in degrees [-180, 180]
    float yaw() const;

    // Relative yaw w.r.t. last zeroYaw() call: wrap180(yaw() - yawZeroDeg)
    float yawRel() const;

    // Set current yaw() as new zero for yawRel()
    void zeroYaw();

    // Optional: force absolute yaw (e.g., for tests)
    void setYaw(float deg);

private:
    void _readRaw();
    static inline float _wrap180(float a) {
        while (a > 180.f) a -= 360.f;
        while (a < -180.f) a += 360.f;
        return a;
    }

    uint8_t _addr;
    float   _rawGyroZ;     // deg/s
    float   _gyroBiasZ;    // deg/s
    float   _yaw;          // absolute yaw (deg)
    float   _yawZeroDeg;   // soft zero offset for yawRel()
    unsigned long _prevMicros;
    bool    _hasPrevTime;
};
