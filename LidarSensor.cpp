#include "LidarSensor.hpp"
#include <Arduino.h>

void LidarSensor::begin() {
    Serial.begin(9600);

    pinMode(FrontPin, OUTPUT);
    pinMode(LeftPin, OUTPUT);
    pinMode(RightPin, OUTPUT);

    digitalWrite(FrontPin, LOW);
    digitalWrite(LeftPin, LOW);
    digitalWrite(RightPin, LOW);

    digitalWrite(FrontPin, HIGH);

    FrontLidar.init();
    FrontLidar.configureDefault();
    FrontLidar.setTimeout(250);
    FrontLidar.setAddress(0x54);

    digitalWrite(LeftPin, HIGH);

    LeftLidar.init();
    LeftLidar.configureDefault();
    LeftLidar.setTimeout(250);
    LeftLidar.setAddress(0x56);

    digitalWrite(RightPin, HIGH);

    RightLidar.init();
    RightLidar.configureDefault();
    RightLidar.setTimeout(250);
    RightLidar.setAddress(0x58);
}

float LidarSensor::getFrontDistance() {
    uint16_t range = FrontLidar.readRangeSingleMillimeters();
    return range;
}

float LidarSensor::getRightDistance() {
    uint16_t range = RightLidar.readRangeSingleMillimeters();
    return range;
}

float LidarSensor::getLeftDistance() {
    uint16_t range = LeftLidar.readRangeSingleMillimeters();
    return range;
}
