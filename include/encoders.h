#pragma once
// Quadrature encoder tick counting, ISR-driven, x4 decoding.
//
// Ticks are CUMULATIVE and reported as signed 32-bit; the Pi differences them itself and
// handles the wrap (fodnav.odom). Do not send deltas -- a dropped telemetry line would
// then lose distance permanently, which is the exact bug cumulative counters avoid.
//
// A missed transition permanently corrupts odometry, so the ISRs must be able to
// pre-empt everything else. Each encoder interrupts on BOTH phases and decodes with a
// state table (quadrature.h). Transitions that cannot happen in a clean signal (A and B
// changing together) are counted in errors(): a steady climb while the shaft turns
// slowly means a wiring or noise problem, not a tuning one.
//
// Only built by the nav_v1 environments; the v2 bench build does not use this file.

#include <Arduino.h>

class Encoder {
 public:
  void begin(int pinA, int pinB);

  // Signed cumulative count. Safe from any task: a single 32-bit load is atomic on ESP32.
  int32_t ticks() const { return ticks_; }
  // Count of impossible transitions seen so far (see above).
  uint32_t errors() const { return errors_; }

  // Called from the ISR trampolines only.
  void IRAM_ATTR isr();

 private:
  int pinA_ = -1, pinB_ = -1;
  volatile uint8_t prev_ = 0;
  volatile int32_t ticks_ = 0;
  volatile uint32_t errors_ = 0;
};

// Two fixed instances: attachInterrupt wants a free function per pin.
extern Encoder gLeftEncoder;
extern Encoder gRightEncoder;

void encodersBegin(int leftA, int leftB, int rightA, int rightB);
