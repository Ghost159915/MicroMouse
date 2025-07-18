#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class Encoder {
private:
    int encA;
    int encB;
    volatile long ticks;

    static Encoder* instance;
    static void isr();  // declared here, defined in .cpp

public:
    Encoder(int a = 2, int b = 7);

    void begin();
    long getTicks();
    void reset();
};

#endif
