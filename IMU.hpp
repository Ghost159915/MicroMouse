#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>

class IMU {
public:
    IMU();

    void begin();
    void calibrate(uint16_t samples = 200);
    void update();
    float yaw() const;

private:
    const uint8_t _addr;
    float _rawGyroZ;
    float _gyroBiasZ;
    float _yaw;
    unsigned long _prevMicros;

    void _readRaw();
};

#endif