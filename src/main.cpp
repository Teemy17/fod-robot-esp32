// Encoderless yellow TT bench prototype. See README for protocol differences.
#include <Arduino.h>
#include "config.h"
#include "protocol.h"
#include "motor.h"

namespace {
MotorChannel leftMotor, rightMotor;
SemaphoreHandle_t stateMutex, serialMutex;
struct State {
  float left = 0, right = 0;
  bool enabled = false, armed = false, watchdog = false, estop = false;
  bool clamped = false;
  uint32_t lastCommandMs = 0;
  int appliedLeft = 0, appliedRight = 0;
} state;

void logMessage(char level, const char* message) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  sendLog(Serial, level, message);
  xSemaphoreGive(serialMutex);
}
void status() {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  State s = state;
  xSemaphoreGive(stateMutex);
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  // Applied PWM, never presented as measured wheel motion.
  Serial.printf("PT %lu %d %d %u %u %u %u\n", (unsigned long)millis(),
                s.appliedLeft, s.appliedRight, s.enabled, s.watchdog,
                s.estop, s.clamped);
  xSemaphoreGive(serialMutex);
}
void handleLine(const String& line) {
  const ParsedCommand cmd = parseCommand(line);
  if (cmd.type == CmdType::kNone) return;
  if (cmd.type == CmdType::kMalformed) {
    logMessage('W', "malformed command ignored");
    return;
  }
  if (cmd.type == CmdType::kVelocity) {
    logMessage('W', "V unavailable without encoders; use P <left> <right>");
    return;
  }
  if (cmd.type == CmdType::kHandshake) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    // Version mismatch deliberately prevents v1 navigation using false odometry.
    sendInfo(Serial, 0, 0, 0, "tt-prototype-0.2.0");
    xSemaphoreGive(serialMutex);
    return;
  }
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  if (feedsWatchdog(cmd.type)) {
    state.armed = true;
    state.lastCommandMs = millis();
  }
  switch (cmd.type) {
    case CmdType::kPwm:
      // Commands received while disabled/faulted cannot cause a later restart.
      if (state.enabled && !state.watchdog && !state.estop) {
        state.left = constrain(cmd.v, -prototype::kMaxDuty, prototype::kMaxDuty);
        state.right = constrain(cmd.omega, -prototype::kMaxDuty, prototype::kMaxDuty);
        state.clamped = state.left != cmd.v || state.right != cmd.omega;
      }
      break;
    case CmdType::kEnable:
      if (digitalRead(pins::kEstopSense) != LOW) {
        state.enabled = true;
        state.watchdog = false;
        state.estop = false;
        state.left = state.right = 0;
        state.clamped = false;
      }
      break;
    case CmdType::kStop:
    case CmdType::kDisable:
      state.left = state.right = 0;
      state.clamped = false;
      if (cmd.type == CmdType::kDisable) state.enabled = false;
      break;
    default: break;
  }
  xSemaphoreGive(stateMutex);
  if (cmd.type == CmdType::kTelemetryReq) status();
}
void commTask(void*) {
  char buf[kMaxLineBytes];
  size_t len = 0;
  bool dropping = false;
  for (;;) {
    // Bound each batch so continuous input cannot monopolize this task.
    for (int n = 0; n < 256 && Serial.available(); ++n) {
      char c = (char)Serial.read();
      if (c == '\n') {
        if (!dropping) { buf[len] = '\0'; handleLine(String(buf)); }
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
void controlTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(timing::kControlPeriodMs));
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    if (state.armed && millis() - state.lastCommandMs >= kWatchdogTimeoutMs) {
      state.watchdog = true;
      state.enabled = false;
    }
    if (digitalRead(pins::kEstopSense) == LOW) {
      state.estop = true;  // latched until explicit E after switch released
      state.enabled = false;
    }
    const bool allowed = state.enabled && !state.watchdog && !state.estop;
    if (!allowed) state.left = state.right = 0;
    constexpr int maxPwm = (1 << pins::kPwmResolutionBits) - 1;
    state.appliedLeft = lroundf(state.left * maxPwm) * prototype::kLeftSign;
    state.appliedRight = lroundf(state.right * maxPwm) * prototype::kRightSign;
    if (!allowed) digitalWrite(pins::kStandby, LOW);
    leftMotor.write(state.appliedLeft);
    rightMotor.write(state.appliedRight);
    if (allowed) digitalWrite(pins::kStandby, HIGH);
    xSemaphoreGive(stateMutex);
    status();
  }
}
}
void setup() {
  pinMode(pins::kStandby, OUTPUT);
  digitalWrite(pins::kStandby, LOW);
  pinMode(pins::kEstopSense, INPUT_PULLUP);
  leftMotor.begin(pins::kLeftPwm, pins::kLeftIn1, pins::kLeftIn2, 0);
  rightMotor.begin(pins::kRightPwm, pins::kRightIn1, pins::kRightIn2, 1);
  Serial.begin(115200);
  stateMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  logMessage('I', "TT prototype: open-loop PWM; no encoders, drum or sensor polling");
  xTaskCreatePinnedToCore(commTask, "comm", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 3, nullptr, 1);
}
void loop() { vTaskDelay(portMAX_DELAY); }
