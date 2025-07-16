#ifndef LIDAR_SENSOR_HPP
#define LIDAR_SENSOR_HPP

#include <Wire.h>
#include <VL6180X.h>

class LidarSensor {
private:
    VL6180X FrontLidar;
    VL6180X RightLidar;
    VL6180X LeftLidar;

    int FrontPin = A1;
    int LeftPin = A0;
    int RightPin = A2;

public:
    void begin() {
        Serial.begin(9600);

        pinMode(FrontPin, OUTPUT);
        pinMode(LeftPin, OUTPUT);
        pinMode(RightPin, OUTPUT);

        digitalWrite(FrontPin, LOW);
        digitalWrite(LeftPin, LOW);
        digitalWrite(RightPin, LOW);
        delay(10);

        digitalWrite(FrontPin, HIGH);
        delay(50);
        FrontLidar.init();
        FrontLidar.configureDefault();
        FrontLidar.setTimeout(250);
        FrontLidar.setAddress(0x54);
        delay(50);

        digitalWrite(LeftPin, HIGH);
        delay(50);
        LeftLidar.init();
        LeftLidar.configureDefault();
        LeftLidar.setTimeout(250);
        LeftLidar.setAddress(0x56);
        delay(50);

        

        digitalWrite(RightPin, HIGH);
        delay(50);
        RightLidar.init();
        RightLidar.configureDefault();
        RightLidar.setTimeout(250);
        RightLidar.setAddress(0x58);
    }

    float getFrontDistance() {
        uint16_t range = FrontLidar.readRangeSingleMillimeters();
        return range;
    }

    float getRightDistance() {
        uint16_t range = RightLidar.readRangeSingleMillimeters();
        return range;
    }

    float getLeftDistance() {
        uint16_t range = LeftLidar.readRangeSingleMillimeters();
        return range;
    }
};

#endif
