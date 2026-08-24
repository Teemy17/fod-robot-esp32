#pragma once
// One TB6612FNG channel: PWM + two direction pins. Used for the left and
// right drive channels (each driving one side's wheel pair in parallel,
// per the BOM) and, on its own small driver, the drum motor.

#include <Arduino.h>

class MotorChannel {
 public:
  void begin(int pwmPin, int in1Pin, int in2Pin, int ledcChannel);

  // signed_duty in [-max, +max] where max = 2^kPwmResolutionBits - 1.
  // Positive drives the wheel/drum "forward" as wired; fix sign here, once,
  // rather than at every call site (HARDWARE.md §2.4 sign convention).
  void write(int32_t signedDuty);

  void brake();   // both direction pins low, coast
  void stop() { write(0); }

 private:
  int pwmPin_ = -1, in1_ = -1, in2_ = -1, ledcChannel_ = 0;
};
