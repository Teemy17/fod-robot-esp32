#pragma once
#include <Arduino.h>

// Yellow TT prototype: no encoders and no collector. Verify wiring first.
namespace pins {
// One TB6612FNG, channel A left / channel B right.
constexpr int kLeftPwm = 25, kLeftIn1 = 26, kLeftIn2 = 27;
constexpr int kRightPwm = 14, kRightIn1 = 12, kRightIn2 = 13;
// STBY must be wired to this GPIO (do not strap it HIGH).
constexpr int kStandby = 23;
constexpr int kEstopSense = 19;  // active LOW; physical switch also cuts VM
constexpr int kI2cSda = 21, kI2cScl = 22;
constexpr int kTofXshutFront = 4, kTofXshutLeft = 5;
constexpr int kPwmResolutionBits = 10;
constexpr int kPwmFreqHz = 20000;
}
namespace prototype {
// Duty cap is a starting limit, NOT a voltage regulator or measured speed.
constexpr float kMaxDuty = 0.5f;
constexpr int kLeftSign = 1, kRightSign = 1;  // set -1 for reversed wiring
static_assert(kMaxDuty > 0 && kMaxDuty <= 1, "Duty limit must be (0, 1]");
static_assert(kLeftSign * kLeftSign == 1 && kRightSign * kRightSign == 1,
              "Motor signs must be +1 or -1");
}
namespace timing {
constexpr uint32_t kControlLoopHz = 50;
constexpr uint32_t kControlPeriodMs = 1000 / kControlLoopHz;
constexpr uint32_t kTofPollHz = 30;
}
constexpr float kTofThresholdM = 0.25f;
