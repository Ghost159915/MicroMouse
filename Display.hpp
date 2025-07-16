#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <U8g2lib.h>

class Display {
private:
    // Page mode constructor! Ultra-low RAM use.
    U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled;
    float wheelRadius;
    int ticksPerRev;

public:
    Display(float radius, int ticks)
      : oled(U8G2_R0), wheelRadius(radius), ticksPerRev(ticks) {}

    void begin() {
        oled.begin();

        oled.firstPage();
        do {
            oled.setFont(u8g2_font_5x8_tr);
            oled.drawStr(0, 12, "OLED Booting!");
        } while (oled.nextPage());

        delay(500);
    }




    void showLidarDistance(int LeftDistance, int FrontDistance, int RightDistance) {
        char bufferL[8], bufferF[8], bufferR[8];
        snprintf(bufferL, sizeof(bufferL), "%d", LeftDistance);
        snprintf(bufferF, sizeof(bufferF), "%d", FrontDistance);
        snprintf(bufferR, sizeof(bufferR), "%d", RightDistance);

        oled.firstPage();
        do {
            oled.setFont(u8g2_font_5x8_tr);
            oled.drawStr(0, 12, "L:");
            oled.drawStr(20, 12, bufferL);

            oled.drawStr(0, 28, "F:");
            oled.drawStr(20, 28, bufferF);

            oled.drawStr(0, 44, "R:");
            oled.drawStr(20, 44, bufferR);
        } while (oled.nextPage());
    }

    void showIMUReading(int angle) {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%d deg", angle);

        oled.firstPage();
        do {
            oled.setFont(u8g2_font_5x8_tr);
            oled.drawStr(0, 12, "IMU Heading:");
            oled.drawStr(0, 28, buffer);
        } while (oled.nextPage());
    }
};

#endif
