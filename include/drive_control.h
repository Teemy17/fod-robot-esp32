#pragma once
// Small pure helpers for the v1 control loop. No Arduino dependency (host-testable).

#include <stdint.h>

namespace drive {

constexpr float kPi = 3.14159265358979f;

// unicycle (v, omega) -> per-wheel linear speed setpoints in m/s. omega is CCW-positive,
// so a positive omega speeds the right wheel up. Mirrors fodnav.control.unicycle_to_wheels
// on the Pi, so both sides agree what "V 0.25 -0.4" means at the wheel.
inline void unicycleToWheels(float v, float omega, float trackWidthM, float& left,
                             float& right) {
  const float half = 0.5f * trackWidthM;
  left = v - omega * half;
  right = v + omega * half;
}

// First-order low-pass on a measured speed. One tick of an encoder is a visible step in
// the raw speed at 50 Hz, so the PID sees the filtered value.
struct SpeedFilter {
  float value = 0.0f;
  float update(float raw, float alpha) {
    value += alpha * (raw - value);
    return value;
  }
  void reset() { value = 0.0f; }
};

// Detects "driven hard but not turning": a stalled motor (1.2 A on these motors, more than
// one TB6612 channel can carry continuously) or an encoder that is not producing ticks.
struct StallDetector {
  uint32_t stalledMs = 0;

  // Returns true once the condition has held for timeMs.
  bool update(float dutyAbs, float speedAbs, uint32_t dtMs, float dutyMin,
              float speedMax, uint32_t timeMs) {
    if (dutyAbs >= dutyMin && speedAbs < speedMax) {
      stalledMs += dtMs;
    } else {
      stalledMs = 0;
    }
    return stalledMs >= timeMs;
  }
  void reset() { stalledMs = 0; }
};

}  // namespace drive
