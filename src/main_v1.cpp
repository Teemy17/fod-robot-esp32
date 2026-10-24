// fod-robot-esp32 -- nav-compatible firmware, protocol v1 (V/T/I), with encoders and a
// closed-loop wheel-speed controller. Built by the nav_v1 environments; the v2 bench
// prototype (main.cpp) is unchanged.
//
// Same two-task shape as the bench build:
//   commTask    (core 0) -- parses lines, updates shared state, answers H and ? inline.
//   controlTask (core 1) -- fixed 50 Hz: encoders -> wheel speeds -> per-wheel PID ->
//               motors, watchdog, e-stop, stall protection, then one T line.
// Encoder ISRs (encoders.cpp) pre-empt both.
//
// Safety behaviour carried over from the v2 build: STBY is gated by GPIO23 and only
// raised while motion is allowed; e-stop and watchdog faults latch until an explicit E;
// commands received while disabled or faulted are discarded so they cannot restart the
// wheels later.

#include <Arduino.h>
#include "config.h"
#include "robot_v1.h"
#include "protocol.h"
#include "protocol_v1.h"
#include "encoders.h"
#include "motor.h"
#include "pid.h"
#include "drive_control.h"

namespace {

const char kFwVersion[] = "esp32-fod-1.0.0-dev";

MotorChannel leftMotor, rightMotor;
SemaphoreHandle_t stateMutex, serialMutex;

struct State {
  float v = 0.0f, omega = 0.0f;  // last accepted command, m/s and rad/s
  bool enabled = false;
  bool armed = false;            // watchdog arms on the FIRST command, not at power-on
  bool watchdog = false;         // latched until E
  bool estop = false;            // latched until E with the switch released
  bool encoderFault = false;     // latched until E (stall / dead encoder)
  bool clamped = false;
  bool magnetOn = false;         // accepted but there is no drum hardware in this build
  bool resetControl = false;     // E received: restart PIDs and filters from zero
  uint32_t lastCommandMs = 0;
} state;

struct Snapshot {
  uint16_t seq = 0;
  uint32_t tMs = 0;
  int32_t ticksL = 0, ticksR = 0;
  float v = 0.0f, omega = 0.0f;
  uint8_t flags = 0;
} lastTelemetry;

// One Serial write per line, under the mutex, so lines from the two tasks never mix.
void writeLine(const String& line) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.print(line);
  xSemaphoreGive(serialMutex);
}

void logMessage(char level, const char* message) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  sendLog(Serial, level, message);
  xSemaphoreGive(serialMutex);
}

uint8_t flagsFrom(const State& s) {
  uint8_t f = 0;
  if (s.enabled) f |= kFlagMotorsEnabled;
  if (s.watchdog) f |= kFlagWatchdogFired;
  if (s.encoderFault) f |= kFlagEncoderFault;
  if (s.clamped) f |= kFlagClamped;
  if (s.estop) f |= kFlagFaultState;
  return f;
}

void handleLine(const String& line) {
  const v1::ParsedCommand cmd = v1::parseCommand(line);
  if (cmd.type == v1::CmdType::kNone) return;  // unknown tag (incl. v2 "P"): ignored
  if (cmd.type == v1::CmdType::kMalformed) {
    logMessage('W', "malformed command ignored");
    return;
  }
  if (cmd.type == v1::CmdType::kHandshake) {  // H does not arm the watchdog
    writeLine(v1::formatInfo(kFwVersion, robot::kReportedTicksPerRev,
                             robot::kReportedWheelRadiusM, robot::kReportedTrackWidthM));
    return;
  }

  bool enableRefused = false;
  bool sendTelemetryNow = false;
  Snapshot snap;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  if (v1::feedsWatchdog(cmd.type)) {
    state.armed = true;
    state.lastCommandMs = millis();
  }
  switch (cmd.type) {
    case v1::CmdType::kVelocity:
      // Discarded while disabled or faulted, so a stale V cannot restart the wheels
      // after a later E.
      if (state.enabled && !state.watchdog && !state.estop && !state.encoderFault) {
        state.v = cmd.v;
        state.omega = cmd.omega;
      }
      break;
    case v1::CmdType::kStop:
      state.v = state.omega = 0.0f;
      break;
    case v1::CmdType::kEnable:
      if (digitalRead(pins::kEstopSense) == LOW) {
        enableRefused = true;
      } else {
        state.enabled = true;
        state.watchdog = false;
        state.estop = false;
        state.encoderFault = false;
        state.v = state.omega = 0.0f;
        state.resetControl = true;
      }
      break;
    case v1::CmdType::kDisable:
      state.enabled = false;
      state.v = state.omega = 0.0f;
      break;
    case v1::CmdType::kMagnet:
      state.magnetOn = cmd.magnetOn;  // no drum fitted: recorded, no output
      break;
    case v1::CmdType::kTelemetryReq:
      sendTelemetryNow = true;
      snap = lastTelemetry;
      break;
    default:
      break;
  }
  xSemaphoreGive(stateMutex);

  if (enableRefused) logMessage('W', "E ignored: e-stop is pressed");
  if (sendTelemetryNow) {
    writeLine(v1::formatTelemetry(snap.seq, snap.tMs, snap.ticksL, snap.ticksR, snap.v,
                                  snap.omega, snap.flags));
  }
}

void commTask(void*) {
  char buf[kMaxLineBytes];
  size_t len = 0;
  bool dropping = false;
  for (;;) {
    // Bounded batch so continuous input cannot monopolise this task.
    for (int n = 0; n < 256 && Serial.available(); ++n) {
      char c = (char)Serial.read();
      if (c == '\n') {
        if (!dropping) {
          buf[len] = '\0';
          handleLine(String(buf));
        }
        len = 0;
        dropping = false;
      } else if (c != '\r' && !dropping) {
        if (c < 32 || c > 126 || len >= kMaxLineBytes - 1) dropping = true;
        else buf[len++] = c;
      }
    }
    vTaskDelay(1);
  }
}

// One wheel's PWM count (before the direction sign) for a speed setpoint.
float wheelCounts(Pid& pid, float setpoint, float measured, float dt, float outMax) {
  if (fabsf(setpoint) < robot::kSetpointDeadbandMps) {
    pid.reset();
    return 0.0f;
  }
  float u = pid.update(setpoint, measured, dt);
  if (u > 0.0f) u += robot::kStictionCounts;
  else if (u < 0.0f) u -= robot::kStictionCounts;
  return ctl::clampf(u, -outMax, outMax);
}

void controlTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(timing::kControlPeriodMs);
  TickType_t wake = xTaskGetTickCount();
  const float dt = timing::kControlPeriodMs / 1000.0f;
  const float metresPerTick = 2.0f * drive::kPi * robot::kWheelRadiusM / robot::kTicksPerRev;
  const float outMax = prototype::kMaxDuty * robot::kFullScaleCounts;

  Pid pidL, pidR;
  for (Pid* p : {&pidL, &pidR}) {
    p->kp = robot::kKp;
    p->ki = robot::kKi;
    p->kd = robot::kKd;
    p->kff = robot::kFeedforward;
    p->outMax = outMax;
  }
  drive::SpeedFilter filtL, filtR;
  drive::StallDetector stallL, stallR;

  int32_t prevL = gLeftEncoder.ticks() * robot::kLeftEncSign;
  int32_t prevR = gRightEncoder.ticks() * robot::kRightEncSign;
  uint16_t seq = 0;

  for (;;) {
    vTaskDelayUntil(&wake, period);
    const uint32_t now = millis();

    // --- encoders -> filtered wheel speeds -----------------------------------------
    const int32_t tL = gLeftEncoder.ticks() * robot::kLeftEncSign;
    const int32_t tR = gRightEncoder.ticks() * robot::kRightEncSign;
    const float measL =
        filtL.update((tL - prevL) * metresPerTick / dt, robot::kSpeedFilterAlpha);
    const float measR =
        filtR.update((tR - prevR) * metresPerTick / dt, robot::kSpeedFilterAlpha);
    prevL = tL;
    prevR = tR;

    xSemaphoreTake(stateMutex, portMAX_DELAY);

    // --- watchdog and e-stop (both latch until E) -----------------------------------
    bool watchdogTripped = false;
    if (state.armed && !state.watchdog &&
        now - state.lastCommandMs >= kWatchdogTimeoutMs) {
      state.watchdog = true;
      watchdogTripped = true;
    }
    if (digitalRead(pins::kEstopSense) == LOW) state.estop = true;

    if (state.resetControl) {
      pidL.reset();
      pidR.reset();
      filtL.reset();
      filtR.reset();
      stallL.reset();
      stallR.reset();
      state.resetControl = false;
    }

    bool allowed = state.enabled && !state.watchdog && !state.estop && !state.encoderFault;
    if (!allowed) state.v = state.omega = 0.0f;

    // --- command -> clamped -> per-wheel setpoints -----------------------------------
    const float limV = ctl::clampf(state.v, -robot::kVMaxMps, robot::kVMaxMps);
    const float limW = ctl::clampf(state.omega, -robot::kOmegaMaxRadps, robot::kOmegaMaxRadps);
    state.clamped = allowed && (limV != state.v || limW != state.omega);
    float setL = 0.0f, setR = 0.0f;
    drive::unicycleToWheels(limV, limW, robot::kTrackWidthM, setL, setR);

    // --- velocity PID, one per wheel (FR-5) -------------------------------------------
    float countsL = 0.0f, countsR = 0.0f;
    bool stalled = false;
    if (allowed) {
      countsL = wheelCounts(pidL, setL, measL, dt, outMax);
      countsR = wheelCounts(pidR, setR, measR, dt, outMax);
      const bool sL = stallL.update(fabsf(countsL), fabsf(measL), timing::kControlPeriodMs,
                                    robot::kStallDutyCounts, robot::kStallSpeedMps,
                                    robot::kStallTimeMs);
      const bool sR = stallR.update(fabsf(countsR), fabsf(measR), timing::kControlPeriodMs,
                                    robot::kStallDutyCounts, robot::kStallSpeedMps,
                                    robot::kStallTimeMs);
      if (sL || sR) {
        stalled = true;
        state.encoderFault = true;
        state.v = state.omega = 0.0f;
        allowed = false;
        countsL = countsR = 0.0f;
      }
    } else {
      pidL.reset();
      pidR.reset();
      stallL.reset();
      stallR.reset();
    }

    // --- motors (same STBY ordering as the v2 build) -----------------------------------
    const int appliedL = (int)lroundf(countsL) * prototype::kLeftSign;
    const int appliedR = (int)lroundf(countsR) * prototype::kRightSign;
    if (!allowed) digitalWrite(pins::kStandby, LOW);
    leftMotor.write(appliedL);
    rightMotor.write(appliedR);
    if (allowed) digitalWrite(pins::kStandby, HIGH);

    // --- telemetry snapshot -------------------------------------------------------------
    Snapshot snap;
    snap.seq = seq;
    snap.tMs = now;
    snap.ticksL = tL;
    snap.ticksR = tR;
    snap.v = 0.5f * (measL + measR);
    snap.omega = (measR - measL) / robot::kTrackWidthM;
    snap.flags = flagsFrom(state);
    lastTelemetry = snap;
    xSemaphoreGive(stateMutex);

    if (watchdogTripped) logMessage('E', "watchdog: no command for 300ms, stopping");
    if (stalled) logMessage('E', "encoder fault: wheel driven but not turning; E to retry");

    // T every tick, unconditionally (mirrors the Pi's own "never skip a V" rule).
    writeLine(v1::formatTelemetry(snap.seq, snap.tMs, snap.ticksL, snap.ticksR, snap.v,
                                  snap.omega, snap.flags));
    seq++;

#ifdef PID_TUNING
    if ((seq & 1) == 0) {  // 25 Hz, as an `L D` line so the Pi ignores it
      char b[112];
      snprintf(b, sizeof(b), "tune sl=%.3f ml=%.3f cl=%.0f sr=%.3f mr=%.3f cr=%.0f el=%lu er=%lu",
               setL, measL, countsL, setR, measR, countsR,
               (unsigned long)gLeftEncoder.errors(), (unsigned long)gRightEncoder.errors());
      logMessage('D', b);
    }
#endif
  }
}

}  // namespace

void setup() {
  pinMode(pins::kStandby, OUTPUT);
  digitalWrite(pins::kStandby, LOW);  // driver stays in standby until motion is allowed
  pinMode(pins::kEstopSense, INPUT_PULLUP);
  leftMotor.begin(pins::kLeftPwm, pins::kLeftIn1, pins::kLeftIn2, 0);
  rightMotor.begin(pins::kRightPwm, pins::kRightIn1, pins::kRightIn2, 1);
  encodersBegin(encpins::kLeftA, encpins::kLeftB, encpins::kRightA, encpins::kRightB);
  Serial.begin(115200);
  stateMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  logMessage('I', "v1 firmware: encoders + closed-loop wheel speed");
  if (!robot::kConstantsMeasured) {
    logMessage('W', "robot constants unmeasured: handshake reports zeros, nav will refuse");
  }
  xTaskCreatePinnedToCore(commTask, "comm", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(controlTask, "control", 6144, nullptr, 3, nullptr, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
