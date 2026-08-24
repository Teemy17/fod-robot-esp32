#pragma once
// Quadrature encoder tick counting, ISR-driven.
//
// CLAUDE.md/protocol.md: ticks are CUMULATIVE and reported as signed 32-bit;
// the Pi differences them itself and handles the wrap (fodnav.odom). Do not
// be clever here and try to send deltas -- a dropped telemetry line would
// then lose distance permanently, which is the exact bug cumulative counters
// exist to avoid.
//
// A missed encoder transition permanently corrupts odometry (CLAUDE.md
// section 5's "encoder ISRs and task priority" note), so these ISRs must be
// allowed to pre-empt everything else -- that is the whole reason this
// project uses FreeRTOS tasks instead of a bare Arduino loop.

#include <Arduino.h>

class Encoder {
 public:
  void begin(int pinA, int pinB);

  // Signed cumulative count. Safe to call from any task; reads are a single
  // 32-bit load so no critical section is needed on ESP32 (Xtensa word
  // access is atomic).
  int32_t ticks() const { return ticks_; }

  // Called from the ISR trampoline only.
  void IRAM_ATTR isr();

 private:
  int pinA_ = -1, pinB_ = -1;
  volatile int32_t ticks_ = 0;
};

// Two fixed instances -- simplest thing that works with the Arduino
// attachInterrupt API, which wants a free function per pin.
extern Encoder gLeftEncoder;
extern Encoder gRightEncoder;

void encodersBegin();
