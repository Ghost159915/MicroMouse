#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class Encoder {
private:
  int encA;
  int encB;
  volatile long ticks;
  static Encoder* instance;

  static void isr() {
    if (instance) instance->ticks++;
  }

public:
  Encoder(int a, int b) : encA(a), encB(b), ticks(0) {
    instance = this;
  }

  void begin() {
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encA), isr, RISING);
  }

  long getTicks() {
    return ticks;
  }
};

Encoder* Encoder::instance = nullptr;

#endif
