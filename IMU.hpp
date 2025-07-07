#ifndef IMU_HPP
#define IMU_HPP

#include "MPU6050.h"
#include <Wire.h>

class IMU {

  private:
  MPU6050 mpu;
  float heading;
  float lastGyroZ;
    float gyroZ_offset;

  static constexpr float GYRO_SENSITIVITY = 131.0;


  public:
  IMU() : heading(0), lastGyroZ(0), gyroZ_offset(0) {}

  void begin() {
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
      while (1) { delay(10); }
    }


    Serial.println("MPU6050 ready!");
  }

  void calibrate() {
    long sum = 0;
    const int samples = 100;

    Serial.println("Calibrating gyro Z... Keep robot still!");
    for (int i = 0; i < samples; i++) {
      int16_t gx, gy, gz;
      mpu.getRotation(&gx, &gy, &gz);
      sum += gz;
      delay(5);
    }
    gyroZ_offset = (float)sum / samples;
    Serial.print("Gyro Z offset = ");
    Serial.println(gyroZ_offset);
  }
  void update(float dt) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    lastGyroZ = static_cast<float>(gz) / GYRO_SENSITIVITY;
    heading += lastGyroZ * dt;
  }

  float getHeading() const{
    return heading;
  }

  float getGyroZ() const{
    return lastGyroZ;
  }
};

#endif