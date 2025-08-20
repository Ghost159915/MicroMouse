#include "Display.hpp"
#include <Arduino.h>

Display::Display(float radius, int ticks)
    : oled(U8G2_R0), wheelRadius(radius), ticksPerRev(ticks) {}

void Display::begin() {
    oled.begin();
    oled.firstPage();
    do {
        oled.setFont(u8g2_font_5x8_tr);
        oled.drawStr(0, 12, "OLED Booting!");
    } while (oled.nextPage());
}

void Display::showLidarDistance(int LeftDistance, int FrontDistance, int RightDistance) {
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

void Display::showIMUReading(int angle) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d deg", angle);
    oled.firstPage();
    do {
        oled.setFont(u8g2_font_5x8_tr);
        oled.drawStr(20, 12, "IMU Heading:");
        oled.drawStr(20, 28, buffer);
    } while (oled.nextPage());
}

void Display::showCommandStatus(const char* stateName, char currentCmd, float heading) {
    char buffer1[32], buffer2[32], buffer3[32], headingStr[10];
    snprintf(buffer1, sizeof(buffer1), "STATE: %s", stateName);
    snprintf(buffer2, sizeof(buffer2), "COMMAND: %c", currentCmd);
    dtostrf(heading, 5, 1, headingStr);
    snprintf(buffer3, sizeof(buffer3), "HEADING: %s °", headingStr);

    oled.firstPage();
    do {
        oled.setFont(u8g2_font_5x8_tr);
        oled.drawStr(0, 12, buffer1);
        oled.drawStr(0, 28, buffer2);
        oled.drawStr(0, 44, buffer3);
    } while (oled.nextPage());
}