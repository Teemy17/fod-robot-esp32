#pragma once
// Textbook PID with integral clamping. One instance per wheel.
// FR-5: closed-loop velocity is not optional here -- magnet drag rises as
// debris accumulates during a trial, so open-loop PWM drifts from the
// commanded speed on exactly the axis every §11b result is plotted against.

struct Pid {
  float kp, ki, kd, outMax;
  float integral = 0.0f;
  float prevError = 0.0f;

  float update(float setpoint, float measured, float dtSeconds) {
    float error = setpoint - measured;
    integral += error * dtSeconds;
    // Clamp the integral term directly (in output units) rather than the
    // accumulator, so kI can be re-tuned without re-deriving the clamp.
    float iTerm = constrain(ki * integral, -outMax, outMax);
    if (iTerm != ki * integral) {
      // Anti-windup: undo the accumulation that just got clamped away.
      integral = iTerm / ki;
    }
    float derivative = (dtSeconds > 1e-6f) ? (error - prevError) / dtSeconds : 0.0f;
    prevError = error;
    float out = kp * error + iTerm + kd * derivative;
    return constrain(out, -outMax, outMax);
  }

  void reset() {
    integral = 0.0f;
    prevError = 0.0f;
  }
};
