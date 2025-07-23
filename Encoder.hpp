#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class Encoder {
private:
    int encA, encB;
    volatile long ticks;

    static Encoder* instance;
    static void isr();

public:
    Encoder(int a = 2, int b = 7);

    void begin();
    long getTicks();
    void reset();
};

#endif
