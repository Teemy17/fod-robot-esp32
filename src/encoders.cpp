#include "encoders.h"
#include "quadrature.h"

Encoder gLeftEncoder;
Encoder gRightEncoder;

void Encoder::begin(int pinA, int pinB) {
  pinA_ = pinA;
  pinB_ = pinB;
  // Plain INPUT: GPIO 34-39 have no pull-ups, and these encoders drive their outputs.
  pinMode(pinA_, INPUT);
  pinMode(pinB_, INPUT);
  prev_ = (uint8_t)((digitalRead(pinA_) << 1) | digitalRead(pinB_));
}

void IRAM_ATTR Encoder::isr() {
  const uint8_t cur = (uint8_t)((digitalRead(pinA_) << 1) | digitalRead(pinB_));
  const uint8_t prev = prev_;
  if (quadIllegal(prev, cur)) errors_++;
  ticks_ += quadDelta(prev, cur);
  prev_ = cur;
}

namespace {
void IRAM_ATTR leftIsr() { gLeftEncoder.isr(); }
void IRAM_ATTR rightIsr() { gRightEncoder.isr(); }
}  // namespace

void encodersBegin(int leftA, int leftB, int rightA, int rightB) {
  gLeftEncoder.begin(leftA, leftB);
  gRightEncoder.begin(rightA, rightB);
  attachInterrupt(digitalPinToInterrupt(leftA), leftIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftB), leftIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightA), rightIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightB), rightIsr, CHANGE);
}
