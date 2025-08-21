#pragma once
#include <Arduino.h>

class DualEncoder {
public:
  DualEncoder(int aL, int bL, int aR, int bR);

  void begin();
  void reset();
  void resetLeft();
  void resetRight();

  long getLeftTicks() const;
  long getRightTicks() const;
  long getAverageTicks() const;

  // ISR wrappers must be static
  static void isrLeftA();
  static void isrRightA();

private:
  // instance handler methods
  void handleLeftA();
  void handleRightA();

  // pin mapping
  int encA_L, encB_L, encA_R, encB_R;

  // tick counters (must be volatile, 32-bit)
  volatile long ticksLeft;
  volatile long ticksRight;

  // singleton pointer used by static ISRs
  static DualEncoder* instance;
};
