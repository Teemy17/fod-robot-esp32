#include "encoders.h"
#include "config.h"

Encoder gLeftEncoder;
Encoder gRightEncoder;

void Encoder::begin(int pinA, int pinB) {
  pinA_ = pinA;
  pinB_ = pinB;
  pinMode(pinA_, INPUT_PULLUP);
  pinMode(pinB_, INPUT_PULLUP);
}

void IRAM_ATTR Encoder::isr() {
  // x4 quadrature decode on A's rising/falling edge: direction from B's
  // level at the edge. Simple and cheap enough for an ISR; swap for the
  // ESP32 hardware PCNT peripheral later if jitter under load proves this
  // insufficient (§2.3 asks the quadrature mode used to be recorded either
  // way, so whichever you ship, note it in HARDWARE.md).
  bool a = digitalRead(pinA_);
  bool b = digitalRead(pinB_);
  ticks_ += (a == b) ? 1 : -1;
}

namespace {
void IRAM_ATTR leftIsr() { gLeftEncoder.isr(); }
void IRAM_ATTR rightIsr() { gRightEncoder.isr(); }
}  // namespace

void encodersBegin() {
  gLeftEncoder.begin(pins::kLeftEncA, pins::kLeftEncB);
  gRightEncoder.begin(pins::kRightEncA, pins::kRightEncB);
  attachInterrupt(digitalPinToInterrupt(pins::kLeftEncA), leftIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::kRightEncA), rightIsr, CHANGE);
}
