#pragma once
// Constants for the nav-compatible (protocol v1) build. Pins shared with the v2 bench
// build (motor driver, STBY, e-stop, direction signs, duty cap) stay in config.h.
//
// NOTHING BELOW IS MEASURED. Every number is either a datasheet value or a conservative
// bring-up choice, and is marked as such. Until the geometry is measured on the real
// chassis:
//   * the firmware uses these nominal values to run the wheel-speed loop (enough to bench
//     test and tune), but
//   * the `I` handshake reports 0.000 for ticks/rev, wheel radius and track width, so
//     nav refuses to run on odometry that would be fiction (same approach as the v2
//     prototype, and finding #7 in the harness repo).
//
// To flip it: measure, then in the SAME change update fod-robot-pathing's robot.yaml
// (the handshake asserts both sides agree to 1e-6 m), then build with
// -D ROBOT_CONSTANTS_MEASURED=1.

#include <stdint.h>
#include "drive_control.h"

#ifndef ROBOT_CONSTANTS_MEASURED
#define ROBOT_CONSTANTS_MEASURED 0
#endif

// Encoder GPIOs. 34/35/36/39 are input-only and have no internal pull-ups; that is fine
// for push-pull Hall outputs. If your encoder turns out to be open-drain, add external
// 10k pull-ups to 3V3. Starting wiring choices from main's config.h, not verified.
namespace encpins {
constexpr int kLeftA = 34, kLeftB = 35;
constexpr int kRightA = 36, kRightB = 39;
}  // namespace encpins

namespace robot {

constexpr bool kConstantsMeasured = (ROBOT_CONSTANTS_MEASURED != 0);

// ---- Encoder ------------------------------------------------------------------------
// x4 decoding: both edges of both phases (encoders.cpp attaches A and B).
// DATASHEET NOMINAL = 13 pulses/motor rev x 90 gear x 4 edges = 4680. NOT MEASURED. The
// two datasheets for this motor family disagree on the gear count (13x48 vs 13x45 for
// the "1:45"), so measure it: whatever you measure must match the decoding mode above.
constexpr float kTicksPerRev = 13.0f * 90.0f * 4.0f;
// Sign so that positive ticks = the robot moving forward. Verify on the bench: lift the
// wheels, drive forward, and check `T` ticks increase. Motor direction signs
// (prototype::kLeftSign/kRightSign in config.h) are separate: set those first.
constexpr int kLeftEncSign = 1;
constexpr int kRightEncSign = 1;

// ---- Geometry (NOMINAL, not measured) -------------------------------------------------
constexpr float kWheelRadiusM = 0.0325f;  // nominal 65 mm wheel; roll-test it
constexpr float kTrackWidthM = 0.200f;    // placeholder; spin-test it

// Conservative command limits for bring-up (the duty cap in config.h also applies).
constexpr float kVMaxMps = 0.25f;
constexpr float kOmegaMaxRadps = 1.5f;

// What the `I` handshake reports.
constexpr float kReportedTicksPerRev = kConstantsMeasured ? kTicksPerRev : 0.0f;
constexpr float kReportedWheelRadiusM = kConstantsMeasured ? kWheelRadiusM : 0.0f;
constexpr float kReportedTrackWidthM = kConstantsMeasured ? kTrackWidthM : 0.0f;

// ---- Wheel-speed loop ----------------------------------------------------------------
// Datasheet: 175 RPM at 6 V, no load. Four AAs sag under load, so real top speed is lower.
constexpr float kNoLoadRpm = 175.0f;
constexpr float kNoLoadMps = kNoLoadRpm / 60.0f * 2.0f * drive::kPi * kWheelRadiusM;
constexpr float kFullScaleCounts = 1023.0f;  // 10-bit PWM, see pins::kPwmResolutionBits

// Starting gains. TUNE ON HARDWARE (README, "Tuning"). Units: PWM counts per (m/s) of
// setpoint or error.
//
// Feedforward is deliberately 75% of what the datasheet speed implies. The datasheet is
// a no-load figure at 6 V; a loaded robot on four sagging AAs is slower, so the full
// value would overdrive and overshoot. Under-driving lets Kp and Ki trim upward and the
// speed never exceeds the target. Measure speed at a known duty and set this to ~1.0
// of the measured value. (Simulated against a first-order wheel with the speed filter
// and one tick of actuation delay: ~3% overshoot, 5% settling in ~0.25 s for the
// datasheet plant. Those numbers are a model, not a measurement.)
constexpr float kFeedforwardScale = 0.75f;
constexpr float kFeedforward = kFeedforwardScale * kFullScaleCounts / kNoLoadMps;  // ~1290
constexpr float kKp = 700.0f;
constexpr float kKi = 3000.0f;
constexpr float kKd = 0.0f;  // quantised speed signal: derivative mostly adds noise

constexpr float kSpeedFilterAlpha = 0.3f;     // low-pass on the measured wheel speed
constexpr float kSetpointDeadbandMps = 0.005f;  // below this the wheel is commanded to rest
constexpr float kStictionCounts = 0.0f;       // duty where the wheel just starts; measure

// ---- Stall protection ----------------------------------------------------------------
// A wheel driven at >= kStallDutyCounts that stays below kStallSpeedMps for kStallTimeMs
// is stalled (or its encoder is dead): cut the motors and raise kFlagEncoderFault. The
// TT motors draw ~1.2 A at stall and one TB6612 channel is rated ~1.2 A continuous.
// Starting values; raise kStallTimeMs if breakaway on your carpet trips it.
constexpr float kStallDutyCounts = 250.0f;
constexpr float kStallSpeedMps = 0.01f;
constexpr uint32_t kStallTimeMs = 600;

}  // namespace robot
