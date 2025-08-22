#pragma once
#include <Arduino.h>
#include <Wire.h>

class IMU {
public:
    IMU();

    void begin();
    void calibrate(uint16_t = 0) {}
    void update();

    float yaw() const;
    float yawRel() const;
    void  zeroYaw();
    void  setYaw(float deg);

    // --- NEW: expose bias-corrected gyro rate & a bias calibration ---
    void  calibrateGyroBias(unsigned long ms = 1200); // call when robot is still
    float gzDps();                                    // deg/s, bias-corrected

private:
    uint8_t addr;
    float   yawDeg;
    float   yawZeroDeg;

    unsigned long lastMillis;
    bool    hasLast;

    // --- NEW: persistent gyro Z bias (deg/s) ---
    float   biasZ = 0.0f;

    float   readGzDps();           // raw (unbiased) deg/s from sensor
    static float wrap180(float a);
};
