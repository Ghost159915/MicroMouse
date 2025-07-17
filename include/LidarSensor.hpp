#ifndef LIDAR_SENSOR_HPP
#define LIDAR_SENSOR_HPP

#include <VL6180X.h>

class LidarSensor {
private:
    VL6180X FrontLidar;
    VL6180X RightLidar;
    VL6180X LeftLidar;

    int FrontPin = A1;
    int LeftPin  = A0;
    int RightPin = A2;

public:
    void begin();

    float getFrontDistance();
    float getRightDistance();
    float getLeftDistance();
};

#endif
