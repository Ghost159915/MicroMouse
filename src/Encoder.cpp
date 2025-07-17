#include "../include/Encoder.hpp"

Encoder* Encoder::instance = nullptr;

Encoder::Encoder(int a, int b)
    : encA(a), encB(b), ticks(0) {
    instance = this;
}

void Encoder::begin() {
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encA), isr, RISING);
}

long Encoder::getTicks() {
    return ticks;
}

void Encoder::reset() {
    ticks = 0;
}

void Encoder::isr() {
    if (instance) instance->ticks++;
}
