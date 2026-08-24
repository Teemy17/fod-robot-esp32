#pragma once
// Everything the firmware needs to know about the physical robot, in one
// place -- mirroring how fod-robot-pathing splits config/robot.yaml (Teemy's
// measurements) from config/nav.yaml (tuning). This file is this repo's
// equivalent of robot.yaml: it should be filled in from docs/HARDWARE.md and
// nothing here should be a guess.
//
// CRITICAL: ticks_per_rev / wheel_radius_m / track_width_m below are sent to
// the Pi in the `I` handshake reply (protocol.md section 5, in millimetres)
// and the Pi refuses to run if they disagree with config/robot.yaml by more
// than 1e-6 m. If you change a value here, change robot.yaml the same day
// (HARDWARE.md section 7) -- Esp32Link.verify() on the Pi enforces this.

#include <Arduino.h>

// ---------------------------------------------------------------------
// Measured constants -- fill in from docs/HARDWARE.md sections 2.1-2.3.
// Placeholders below are NOT measurements; treat them as compile errors
// waiting to happen, the same way robot.yaml ships full of `null`.
// ---------------------------------------------------------------------
namespace robot {
constexpr float kTicksPerRev   = 1440.0f;   // §2.3: 10 hand-revolutions, verified
constexpr float kWheelRadiusM  = 0.0325f;   // §2.1: 10-revolution roll test
constexpr float kTrackWidthM   = 0.2000f;   // §2.2: spin-test corrected

constexpr float kVMaxMps       = 0.45f;     // §2.5, measured not spec
constexpr float kOmegaMaxRadps = 3.0f;      // §2.5
}  // namespace robot

// ---------------------------------------------------------------------
// Pins. Placeholders -- wire to the real chassis and update here. Keep this
// the only file that names a GPIO number.
// ---------------------------------------------------------------------
namespace pins {
// Drive motors -- TB6612FNG x1 per side, each driving one side's wheel pair
// in parallel (BOM: 4WD, all wheels driven and encoded).
constexpr int kLeftPwm = 25, kLeftIn1 = 26, kLeftIn2 = 27;
constexpr int kRightPwm = 14, kRightIn1 = 12, kRightIn2 = 13;

// Quadrature encoders, one pair of channels per side.
constexpr int kLeftEncA = 34, kLeftEncB = 35;   // input-only pins on most devkits
constexpr int kRightEncA = 36, kRightEncB = 39;

// Drum motor (JGB37-520, PRD §9) -- fixed RPM, never slaved to ground speed.
// A separate small driver channel, not TB6612FNG (that's committed to drive).
constexpr int kDrumPwm = 33, kDrumIn1 = 32, kDrumIn2 = 15;
constexpr int kDrumEncA = -1;  // TODO: wire if the drum motor is the encoded
                                // variant (HARDWARE.md recommends it) -- lets
                                // M-2 (RPM vs release reliability) close the
                                // loop instead of running the drum open-loop.

// ToF (VL53L0X x2-4 on one I2C bus, XSHUT per sensor to assign addresses --
// PRD FR-10). I2C default pins (SDA 21 / SCL 22 on most ESP32 devkits).
constexpr int kI2cSda = 21, kI2cScl = 22;
constexpr int kTofXshutFront = 4;   // covers the lidar's ~15-20cm dead zone
constexpr int kTofXshutLeft  = 5;   // 45 deg front-corner
constexpr int kTofXshutRight = 18;  // 45 deg front-corner
// constexpr int kTofXshutDown = 17;  // optional cliff sensor

// Physical e-stop, NFR-3: cuts motor power in hardware AND is read here so
// the firmware can report/act on it. Software must never be the only thing
// standing between a command and the motors.
constexpr int kEstopSense = 19;

constexpr int kPwmResolutionBits = 10;   // 0-1023
constexpr int kPwmFreqHz = 20000;        // above audible range
}  // namespace pins

// ---------------------------------------------------------------------
// Loop rates. protocol.md section 6 fixes telemetry/command handling at
// 50 Hz; the control loop runs at the same rate so a tick is a tick.
// ---------------------------------------------------------------------
namespace timing {
constexpr uint32_t kControlLoopHz = 50;
constexpr uint32_t kControlPeriodMs = 1000 / kControlLoopHz;
constexpr uint32_t kTofPollHz = 30;
}  // namespace timing

// PID gains -- tune on hardware. Start conservative; these are placeholders.
namespace pid {
constexpr float kWheelKp = 40.0f;
constexpr float kWheelKi = 120.0f;
constexpr float kWheelKd = 0.2f;
constexpr float kOutMax = (1 << pins::kPwmResolutionBits) - 1;
}  // namespace pid

// Obstacle flag threshold, PRD FR-10.
constexpr float kTofThresholdM = 0.25f;
