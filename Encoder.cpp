#include "Encoder.hpp"

DualEncoder* DualEncoder::instance = nullptr;

DualEncoder::DualEncoder(int aL, int bL, int aR, int bR)
: encA_L(aL), encB_L(bL), encA_R(aR), encB_R(bR),
  ticksLeft(0), ticksRight(0) {
  instance = this;
}

void DualEncoder::begin() {
  pinMode(encA_L, INPUT_PULLUP);
  pinMode(encB_L, INPUT_PULLUP);
  pinMode(encA_R, INPUT_PULLUP);
  pinMode(encB_R, INPUT_PULLUP);

  // A channels drive the interrupts; each A → its own handler
  attachInterrupt(digitalPinToInterrupt(encA_L), DualEncoder::isrLeftA,  RISING);
  attachInterrupt(digitalPinToInterrupt(encA_R), DualEncoder::isrRightA, RISING);
}

void DualEncoder::reset()       { noInterrupts(); ticksLeft = 0; ticksRight = 0; interrupts(); }
void DualEncoder::resetLeft()   { noInterrupts(); ticksLeft = 0;                      interrupts(); }
void DualEncoder::resetRight()  { noInterrupts();                ticksRight = 0;      interrupts(); }

long DualEncoder::getLeftTicks() const  { noInterrupts(); long v = ticksLeft;  interrupts(); return v; }
long DualEncoder::getRightTicks() const { noInterrupts(); long v = ticksRight; interrupts(); return v; }
long DualEncoder::getAverageTicks() const {
  noInterrupts(); long l = ticksLeft, r = ticksRight; interrupts();
  return (l + r) / 2;
}

// ---- Static ISR wrappers ----
void DualEncoder::isrLeftA()  { if (instance) instance->handleLeftA(); }
void DualEncoder::isrRightA() { if (instance) instance->handleRightA(); }

// ---- Direction logic ----
// On each edge of A, read B to infer direction.
// If your counts go the wrong way, just swap ++ and -- in the handlers.
void DualEncoder::handleLeftA() {
  bool b = digitalRead(encB_L);
  if (b) --ticksLeft;           // or --ticksLeft; if your left is inverted
  else    ++ticksLeft;
}

void DualEncoder::handleRightA() {
  bool b = digitalRead(encB_R);
  if (b) ++ticksRight;          // or --ticksRight; if your right is inverted
  else    --ticksRight;
}
