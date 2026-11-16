#pragma once
// Constants for the nav-compatible (protocol v1) build. Pins shared with the v2 bench
// build (motor driver, STBY, e-stop, direction signs, duty cap) stay in config.h.
//
// The geometry below is MEASURED on the real chassis (16 Nov 2026, tools/drive_measure.py
// in the harness repo) and the nav_v1 build sets -D ROBOT_CONSTANTS_MEASURED=1, so the
// `I` handshake reports these numbers. Without the flag it reports 0.000 for ticks/rev,
// wheel radius and track width, and nav refuses to run (finding #7 in the harness repo).
//
// To change a number: in the SAME change update the harness repo's config/robot.yaml
// (nav's handshake asserts both sides agree to 1e-6) and its config/sim_robot.yaml
// (tests/contract/test_firmware_constants.py checks both).

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
// MEASURED by hand, 10 turns x 2 runs per wheel (tools/ticks_count.py in the harness repo):
// left 4733.1 / 4744.5, right 4725.9 / 4708.5, mean 4728. That is 1% above the datasheet
// 13 pulses x 90 gear x 4 edges = 4680: the "1:90" gearbox is ~1:90.9. It confirms x4.
constexpr float kTicksPerRev = 4728.0f;
// Sign so that positive ticks = the robot moving forward. Verify on the bench: lift the
// wheels, drive forward, and check `T` ticks increase. Motor direction signs
// (prototype::kLeftSign/kRightSign in config.h) are separate: set those first.
constexpr int kLeftEncSign = 1;
constexpr int kRightEncSign = -1;

// ---- Geometry (MEASURED) -------------------------------------------------------------
// Roll test: 0.10 m/s on the floor, distance by tape. Runs 2 and 3 gave 32.98 / 33.47 mm
// (run 1 curved, L/R -2.7%, left out). The datasheet 65 mm wheel rolls ~2% further.
// Distances were read to 1 cm, so this is good to ~1.5%; re-run read to the mm to refine.
constexpr float kWheelRadiusM = 0.0332f;
// Spin test at 1.5 rad/s (1.0 stalls on the floor), effective skid-steer track width
// at kWheelRadiusM: CCW 0.1978 m (6 runs, +-2.6%), CW 0.2226 m (5 runs, +-1.3%). The
// chassis slips more turning clockwise; this is the mean of the two directions, so
// odometry heading is ~6% off per direction. Scales with kWheelRadiusM if that changes.
constexpr float kTrackWidthM = 0.210f;

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
// Feedforward MEASURED on the bench (wheels raised, 4xAA, settled counts over the last
// second of a 5 s run): left 261 / 539 and right 251 / 509 counts at 0.10 / 0.20 m/s.
// Counts more than double from 0.10 to 0.20, so a 2-point line has a negative intercept
// (likely battery sag at the higher current, not friction); the least-squares line
// through zero is 2608 and predicts both points within 2%. The integral trims the rest,
// including the left wheel needing ~5% more than the right. The datasheet value
// (kFullScaleCounts / kNoLoadMps, ~1720) was half of this, which is why the untuned
// loop took ~1 s to rise.
constexpr float kFeedforward = 2600.0f;
// Kp/Ki: with Kp 700 / Ki 3000 and the measured feedforward, a 0.20 step rose in ~0.22 s
// but the integral wound up during the rise: 6% / 10% overshoot (L/R) and a ~2 s tail.
// Lower Ki stops the windup, higher Kp keeps the rise fast. Chosen on a first-order model
// fitted to the bench logs (tau ~0.16 s); confirm on the bench.
constexpr float kKp = 1400.0f;
constexpr float kKi = 1000.0f;
constexpr float kKd = 0.0f;  // quantised speed signal: derivative mostly adds noise

constexpr float kSpeedFilterAlpha = 0.3f;     // low-pass on the measured wheel speed
constexpr float kSetpointDeadbandMps = 0.005f;  // below this the wheel is commanded to rest
constexpr float kStictionCounts = 0.0f;       // the 2-point fit above gave < 0: none to add

// ---- Stall protection ----------------------------------------------------------------
// A wheel driven at >= kStallDutyCounts that stays below kStallSpeedMps for kStallTimeMs
// is stalled (or its encoder is dead): cut the motors and raise kFlagEncoderFault. The
// TT motors draw ~1.2 A at stall and one TB6612 channel is rated ~1.2 A continuous.
// Starting values; raise kStallTimeMs if breakaway on your carpet trips it.
constexpr float kStallDutyCounts = 250.0f;
constexpr float kStallSpeedMps = 0.01f;
constexpr uint32_t kStallTimeMs = 600;

}  // namespace robot
