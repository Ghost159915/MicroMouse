#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Display {
private:
  Adafruit_SSD1306 oled;
  float wheelRadius;
  int ticksPerRev;

public:
  Display(float radius, int ticks)
    : oled(128, 64, &Wire, -1), wheelRadius(radius), ticksPerRev(ticks) {}

  void begin() {
    // if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    //   Serial.println("OLED not found!");
    //   while (1);
    // }

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("OLED Booting!!");
    oled.display();
  }

  void showEncoderDistance(long encoderTicks) {
    float distPerTick = (2 * PI * wheelRadius) / ticksPerRev;
    float distance = encoderTicks * distPerTick;

    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("Distance:");
    oled.setCursor(0, 16);
    oled.print(distance, 1);
    oled.println(" mm");
    oled.display();
  }

  void showLidarDistance(int LeftDistance, int FrontDistance, int RightDistance) {
    oled.clearDisplay();
    oled.setCursor(32, 0);
    oled.print("L");
    oled.setCursor(64, 0);
    oled.print("F");
    oled.setCursor(96, 0);
    oled.print("R");

    oled.setCursor(27, 20);
    oled.print(LeftDistance, 1);

    oled.setCursor(59, 20);
    oled.print(FrontDistance, 1);

    oled.setCursor(91, 20);
    oled.print(RightDistance, 1);

    oled.display();
  }

  void ShowIMUReading(int angle) {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.print("Reading: ");
    oled.print(angle, 1);
  }
};

#endif
