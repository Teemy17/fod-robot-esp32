// fod-robot-esp32 -- motor + sensor firmware, ESP32 side of docs/protocol.md.
//
// Two FreeRTOS tasks, deliberately not a bare Arduino loop() (CLAUDE.md
// section 5's rationale, restated for this repo): the velocity PID, encoder
// handling, serial comms and drum motor all run at different rates and one
// of them (encoder ISRs) must be able to pre-empt everything else, which a
// super-loop cannot express.
//
//   CommTask     (core 0) -- reads Serial, parses commands, updates shared
//                state, feeds the watchdog timestamp, answers H/? inline.
//   ControlTask  (core 1) -- fixed 50 Hz: reads encoders, runs the watchdog
//                check, runs per-wheel velocity PID, drives the drum,
//                reads the e-stop pin, sends T telemetry.
//
// Encoder ISRs (encoders.cpp) run outside both tasks, on whichever core the
// interrupt lands on, and are never blocked by either.

#include <Arduino.h>
#include "config.h"
#include "protocol.h"
#include "encoders.h"
#include "motor.h"
#include "pid.h"

static const char kFwVersion[] = "esp32-fod-0.1.0";

// ---------------------------------------------------------------------
// Shared state. Small and simple: a mutex around a POD struct rather than
// atomics per field, because the control loop touches several fields
// together and torn reads across fields would be worse than a short lock.
// ---------------------------------------------------------------------
struct CommandState {
  float v = 0.0f;
  float omega = 0.0f;
  bool enabled = false;
  bool magnetOn = false;
  uint32_t lastCommandMs = 0;   // millis() at last watchdog-feeding line
  bool everCommanded = false;   // §6: the watchdog arms on the FIRST command,
                                 // not at power-on
};

struct TelemetrySnapshot {
  uint16_t seq = 0;
  uint32_t tMs = 0;
  int32_t ticksL = 0, ticksR = 0;
  float vMeas = 0.0f, omegaMeas = 0.0f;
  uint8_t flags = 0;
};

static SemaphoreHandle_t gStateMutex;
static CommandState gCommand;
static TelemetrySnapshot gTelemetry;  // last value sent, for '?' requests
static volatile bool gWatchdogFired = false;
static volatile bool gDriverFault = false;   // TODO: wire to TB6612 fault pin if used
static volatile bool gObstacle = false;      // TODO: set from tof.cpp once wired

static MotorChannel gLeftMotor, gRightMotor, gDrumMotor;
static Pid gLeftPid{pid::kWheelKp, pid::kWheelKi, pid::kWheelKd, pid::kOutMax};
static Pid gRightPid{pid::kWheelKp, pid::kWheelKi, pid::kWheelKd, pid::kOutMax};

// ---------------------------------------------------------------------
// Serial line reader -- bounded, so a line over kMaxLineBytes is dropped
// whole and the stream resynchronises on the next '\n' rather than being
// truncated into something that parses as a different, wrong command.
// ---------------------------------------------------------------------
namespace {

void handleLine(const String& line) {
  ParsedCommand cmd = parseCommand(line);
  uint32_t now = millis();

  if (cmd.type == CmdType::kMalformed) {
    sendLog(Serial, 'W', "malformed: " + line);
    return;
  }
  if (cmd.type == CmdType::kNone) {
    return;  // unrecognised tag: ignore, per protocol.md section 2
  }

  if (feedsWatchdog(cmd.type)) {
    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    gCommand.lastCommandMs = now;
    gCommand.everCommanded = true;
    xSemaphoreGive(gStateMutex);
  }

  switch (cmd.type) {
    case CmdType::kVelocity:
      xSemaphoreTake(gStateMutex, portMAX_DELAY);
      gCommand.v = cmd.v;
      gCommand.omega = cmd.omega;
      xSemaphoreGive(gStateMutex);
      break;
    case CmdType::kStop:
      xSemaphoreTake(gStateMutex, portMAX_DELAY);
      gCommand.v = 0.0f;
      gCommand.omega = 0.0f;
      xSemaphoreGive(gStateMutex);
      break;
    case CmdType::kEnable:
      xSemaphoreTake(gStateMutex, portMAX_DELAY);
      gCommand.enabled = true;
      xSemaphoreGive(gStateMutex);
      gWatchdogFired = false;  // explicit recovery, protocol.md section 6
      break;
    case CmdType::kDisable:
      xSemaphoreTake(gStateMutex, portMAX_DELAY);
      gCommand.enabled = false;
      gCommand.v = 0.0f;
      gCommand.omega = 0.0f;
      xSemaphoreGive(gStateMutex);
      break;
    case CmdType::kMagnet:
      xSemaphoreTake(gStateMutex, portMAX_DELAY);
      gCommand.magnetOn = cmd.magnet_on;
      xSemaphoreGive(gStateMutex);
      break;
    case CmdType::kTelemetryReq: {
      xSemaphoreTake(gStateMutex, portMAX_DELAY);
      TelemetrySnapshot t = gTelemetry;
      xSemaphoreGive(gStateMutex);
      sendTelemetry(Serial, t.seq, t.tMs, t.ticksL, t.ticksR, t.vMeas,
                    t.omegaMeas, t.flags);
      break;
    }
    case CmdType::kHandshake:
      sendInfo(Serial, robot::kTicksPerRev, robot::kWheelRadiusM,
               robot::kTrackWidthM, kFwVersion);
      break;
    default:
      break;
  }
}

void commTask(void*) {
  static char buf[kMaxLineBytes + 1];
  static size_t len = 0;

  for (;;) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n') {
        buf[len] = '\0';
        handleLine(String(buf));
        len = 0;
      } else if (c != '\r') {
        if (len < kMaxLineBytes) {
          buf[len++] = c;
        } else {
          // Over-length: drop the whole line, resync on the next '\n'.
          len = 0;
          while (Serial.available() && Serial.peek() != '\n') Serial.read();
        }
      }
    }
    vTaskDelay(1);  // yield; Serial has its own buffer, no data is lost
  }
}

// unicycle (v, omega) -> per-wheel linear speed setpoints, m/s. Mirrors
// fodnav.control.unicycle_to_wheels on the Pi so the two sides agree about
// what "V 0.25 -0.4" means at the wheel.
void unicycleToWheels(float v, float omega, float trackWidthM,
                       float& outLeft, float& outRight) {
  float half = 0.5f * trackWidthM;
  outLeft = v - omega * half;
  outRight = v + omega * half;
}

void controlTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(timing::kControlPeriodMs);
  TickType_t lastWake = xTaskGetTickCount();

  const float metresPerTick =
      2.0f * PI * robot::kWheelRadiusM / robot::kTicksPerRev;

  int32_t prevTicksL = gLeftEncoder.ticks();
  int32_t prevTicksR = gRightEncoder.ticks();
  uint16_t seq = 0;

  pinMode(pins::kEstopSense, INPUT_PULLUP);  // wire e-stop to pull this low

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    uint32_t now = millis();
    float dt = timing::kControlPeriodMs / 1000.0f;

    // --- snapshot the command state -----------------------------------
    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    CommandState cmd = gCommand;
    xSemaphoreGive(gStateMutex);

    // --- watchdog, protocol.md section 6 -------------------------------
    // Arms on the first command received, not at power-on: before that
    // there is nothing to protect against and the motors are disabled.
    bool watchdogTrip = false;
    if (cmd.everCommanded && !gWatchdogFired &&
        (now - cmd.lastCommandMs) > kWatchdogTimeoutMs) {
      gWatchdogFired = true;
      watchdogTrip = true;
      sendLog(Serial, 'E', "watchdog: no command for 300ms, braking");
    }

    // --- e-stop ---------------------------------------------------------
    // Active LOW as wired above; the physical e-stop should ALSO cut motor
    // power directly in hardware (NFR-3) -- this is the firmware noticing
    // and reporting, not the safety mechanism itself.
    bool estopTripped = (digitalRead(pins::kEstopSense) == LOW);

    bool motionAllowed = cmd.enabled && !gWatchdogFired && !estopTripped &&
                          !gDriverFault;

    // --- encoders --------------------------------------------------------
    int32_t ticksL = gLeftEncoder.ticks();
    int32_t ticksR = gRightEncoder.ticks();
    int32_t dTicksL = ticksL - prevTicksL;
    int32_t dTicksR = ticksR - prevTicksR;
    prevTicksL = ticksL;
    prevTicksR = ticksR;

    float measLeft = dTicksL * metresPerTick / dt;
    float measRight = dTicksR * metresPerTick / dt;

    // --- velocity PID, one instance per wheel (FR-5) ---------------------
    float setLeft = 0.0f, setRight = 0.0f;
    bool clamped = false;
    if (motionAllowed) {
      float reqV = constrain(cmd.v, -robot::kVMaxMps, robot::kVMaxMps);
      float reqOmega =
          constrain(cmd.omega, -robot::kOmegaMaxRadps, robot::kOmegaMaxRadps);
      clamped = (reqV != cmd.v) || (reqOmega != cmd.omega);
      unicycleToWheels(reqV, reqOmega, robot::kTrackWidthM, setLeft, setRight);

      int32_t outL = (int32_t)gLeftPid.update(setLeft, measLeft, dt);
      int32_t outR = (int32_t)gRightPid.update(setRight, measRight, dt);
      gLeftMotor.write(outL);
      gRightMotor.write(outR);
    } else {
      gLeftPid.reset();
      gRightPid.reset();
      gLeftMotor.brake();
      gRightMotor.brake();
    }

    // --- drum: fixed speed, never slaved to ground speed (PRD §9 / FR-12).
    // Runs whenever commanded on and motion is otherwise allowed; stopping
    // it on a watchdog/e-stop trip is a judgement call for your build --
    // PRD FR-7 only requires the drive motors cut, but leaving a magnet
    // drum spinning on a tripped e-stop is hard to justify.
    if (cmd.magnetOn && motionAllowed) {
      gDrumMotor.write(pid::kOutMax / 2);  // TODO: replace with a measured
                                            // fixed-RPM setpoint once the
                                            // drum is built (PRD §9 O-4,
                                            // M-2). Open-loop until the
                                            // drum gets its own encoder.
    } else {
      gDrumMotor.stop();
    }

    // --- flags -------------------------------------------------------
    uint8_t flags = 0;
    if (cmd.enabled) flags |= kFlagMotorsEnabled;
    if (gWatchdogFired) flags |= kFlagWatchdogFired;
    if (gDriverFault) flags |= kFlagDriverFault;
    if (gObstacle) flags |= kFlagObstacle;
    if (clamped) flags |= kFlagClamped;
    if (estopTripped) flags |= kFlagFaultState;
    // kFlagEncoderFault, kFlagBatteryLow: TODO once those sensors exist.

    // --- telemetry, unconditionally, every tick (mirrors the Pi's own
    // "never skip a send" rule for V -- symmetric obligation here for T).
    float vMeas = 0.5f * (measLeft + measRight);
    float omegaMeas = (measRight - measLeft) / robot::kTrackWidthM;

    TelemetrySnapshot snap{seq, now, ticksL, ticksR, vMeas, omegaMeas, flags};
    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    gTelemetry = snap;
    xSemaphoreGive(gStateMutex);

    sendTelemetry(Serial, seq, now, ticksL, ticksR, vMeas, omegaMeas, flags);
    seq++;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  gStateMutex = xSemaphoreCreateMutex();

  encodersBegin();
  gLeftMotor.begin(pins::kLeftPwm, pins::kLeftIn1, pins::kLeftIn2, /*ledc*/ 0);
  gRightMotor.begin(pins::kRightPwm, pins::kRightIn1, pins::kRightIn2, 1);
  gDrumMotor.begin(pins::kDrumPwm, pins::kDrumIn1, pins::kDrumIn2, 2);

  // TODO: tof.cpp -- VL53L0X x2-4 bring-up over I2C with XSHUT address
  // assignment (PRD FR-10), publishing into gObstacle. Stubbed out for now
  // so the motor/watchdog/protocol core is testable before the sensors are
  // wired.

  xTaskCreatePinnedToCore(commTask, "comm", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 3, nullptr, 1);
}

void loop() {
  // Everything happens in the FreeRTOS tasks above.
  vTaskDelay(portMAX_DELAY);
}
