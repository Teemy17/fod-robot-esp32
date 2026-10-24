#pragma once
// Per-wheel velocity controller: feedforward + PI, optional D on the measurement.
// One instance per wheel. Setpoint and measurement are in m/s, output in PWM counts.
// Pure C++ (no Arduino), so it is host-testable.
//
// FR-5: closed-loop velocity is not optional -- magnet drag rises as debris accumulates
// during a trial, so open-loop PWM drifts from the commanded speed.
//
// Changes from the first version:
//  * feedforward term (kff): most of the output comes from the setpoint, so the integral
//    only trims; without it Ki alone has to build the whole output.
//  * derivative is taken on the measurement, so a new V command does not kick the output.
//  * conditional integration (no accumulation while the output is saturated), and the
//    integral is bounded without dividing by ki, so ki = 0 is safe.

#include <cmath>

namespace ctl {
inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
}  // namespace ctl

struct Pid {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  float kff = 0.0f;      // feedforward: output counts per unit of setpoint (counts per m/s)
  float outMax = 0.0f;   // symmetric output limit, counts
  float integral = 0.0f;
  float prevMeasured = 0.0f;
  bool primed = false;

  float update(float setpoint, float measured, float dtSeconds) {
    const float err = setpoint - measured;
    float derivative = 0.0f;
    if (primed && dtSeconds > 1e-6f) {
      derivative = -(measured - prevMeasured) / dtSeconds;
    }
    prevMeasured = measured;
    primed = true;

    const float base = kff * setpoint + kp * err + kd * derivative;
    if (ki > 0.0f) {
      const float trial = base + ki * integral;
      const bool pushingPastMax = (trial >= outMax && err > 0.0f);
      const bool pushingPastMin = (trial <= -outMax && err < 0.0f);
      if (!pushingPastMax && !pushingPastMin) integral += err * dtSeconds;
      const float bound = outMax / ki;
      integral = ctl::clampf(integral, -bound, bound);
    } else {
      integral = 0.0f;
    }
    return ctl::clampf(base + ki * integral, -outMax, outMax);
  }

  void reset() {
    integral = 0.0f;
    prevMeasured = 0.0f;
    primed = false;
  }
};
