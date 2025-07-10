#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <Wire.h>
#include <U8g2lib.h>

class Display {
private:
    U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled;
    float wheelRadius;
    int ticksPerRev;

public:
    Display(float radius, int ticks)
      : oled(U8G2_R0, U8X8_PIN_NONE), wheelRadius(radius), ticksPerRev(ticks) {}

    void begin() {
        oled.begin();
        oled.clearBuffer();
        oled.setFont(u8g2_font_ncenB08_tr);
        oled.drawStr(0, 12, "OLED Booting!");
        oled.sendBuffer();
        delay(500);
    }

    void showEncoderDistance(long encoderTicks) {
        float distPerTick = (2 * PI * wheelRadius) / ticksPerRev;
        float distance = encoderTicks * distPerTick;

        oled.clearBuffer();
        oled.setFont(u8g2_font_ncenB08_tr);
        oled.drawStr(0, 12, "Distance:");
        
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%.1f mm", distance);
        oled.drawStr(0, 28, buffer);
        
        oled.sendBuffer();
    }

    void showLidarDistance(int LeftDistance, int FrontDistance, int RightDistance) {
        oled.clearBuffer();
        oled.setFont(u8g2_font_ncenB08_tr);

        oled.drawStr(0, 12, "L:");
        oled.setCursor(20, 12);
        oled.print(LeftDistance);

        oled.drawStr(0, 28, "F:");
        oled.setCursor(20, 28);
        oled.print(FrontDistance);

        oled.drawStr(0, 44, "R:");
        oled.setCursor(20, 44);
        oled.print(RightDistance);

        oled.sendBuffer();
    }

    void ShowIMUReading(int angle) {
        oled.clearBuffer();
        oled.setFont(u8g2_font_ncenB08_tr);
        
        oled.drawStr(0, 12, "IMU Heading:");
        
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%d deg", angle);
        oled.drawStr(0, 28, buffer);
        
        oled.sendBuffer();
    }
};

#endif
