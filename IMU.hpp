#pragma once
#include <Arduino.h>
#include <Wire.h>

class IMU {
public:
    IMU();

    void begin();                    // wake MPU-6050
    void calibrate(uint16_t = 0) {}  // no-op (kept for API compatibility)
    void update();                   // integrate Z gyro -> yaw (deg)

    float yaw() const;               // absolute heading in degrees [-180, 180]
    float yawRel() const;            // relative to last zeroYaw()
    void  zeroYaw();                 // set current heading as 0 for yawRel()
    void  setYaw(float deg);         // force absolute yaw (deg)

private:
    uint8_t addr;
    float   yawDeg;
    float   yawZeroDeg;

    unsigned long lastMillis;
    bool    hasLast;

    // read MPU-6050 GYRO_Z (deg/s) using raw register access
    float   readGzDps();
    static float wrap180(float a);
};
