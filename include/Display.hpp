#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <U8g2lib.h>

class Display {
private:
    U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled;
    float wheelRadius;
    int ticksPerRev;

public:
    Display(float radius, int ticks);

    void begin();
    void showLidarDistance(int LeftDistance, int FrontDistance, int RightDistance);
    void showIMUReading(int angle);
};

#endif
