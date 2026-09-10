// Standalone bench firmware: select PlatformIO environment motor_test.
#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "motor_test_state.h"

namespace {
MotorChannel leftMotor, rightMotor;
MotorTestState state;
SemaphoreHandle_t mutex;

void help() {
  Serial.println("\nTT motor test: each move lasts 1 second; no automatic sequence.");
  Serial.println("F forward | B backward | L/R spin left/right");
  Serial.println("A left side only | C right side only | S stop");
  Serial.println("1..5 select 10..50% duty while stopped (config cap also applies)");
  Serial.println("E clear released e-stop | H help. Default duty: 20%.");
  Serial.println("Send one letter, optionally followed by Enter. Raise wheels first.");
}

void controlTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    state.tick(millis(), digitalRead(pins::kEstopSense) == LOW);
    constexpr int maxPwm = (1 << pins::kPwmResolutionBits) - 1;
    float left = constrain(state.left / 100.0f, -prototype::kMaxDuty,
                           prototype::kMaxDuty);
    float right = constrain(state.right / 100.0f, -prototype::kMaxDuty,
                            prototype::kMaxDuty);
    // Disable before changing outputs; enable only for an active timed test.
    digitalWrite(pins::kStandby, LOW);
    leftMotor.write(lroundf(left * maxPwm) * prototype::kLeftSign);
    rightMotor.write(lroundf(right * maxPwm) * prototype::kRightSign);
    if (state.moving() && !state.fault) digitalWrite(pins::kStandby, HIGH);
    xSemaphoreGive(mutex);
    // Safety timing does not depend on serial output or a connected host.
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
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
  mutex = xSemaphoreCreateMutex();
  if (!mutex) { Serial.println("ERROR: mutex allocation failed; disabled"); return; }
  if (xTaskCreatePinnedToCore(controlTask, "motor-test", 2048, nullptr, 3,
                              nullptr, 1) != pdPASS) {
    mutex = nullptr;  // loop refuses all commands; STBY remains low
    Serial.println("ERROR: control task failed; disabled");
    return;
  }
  help();
}

void loop() {
  if (!mutex) { delay(100); return; }
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= 'a' && c <= 'z') c -= 'a' - 'A';
    if (c == 'H' || c == '?') help();
    else if (c != '\n' && c != '\r' && c != ' ' && c != '\t') {
      xSemaphoreTake(mutex, portMAX_DELAY);
      bool accepted = state.command(c, millis(), digitalRead(pins::kEstopSense) == LOW);
      MotorTestState snapshot = state;
      xSemaphoreGive(mutex);
      Serial.printf("%c: %s | requested duty %d%% | e-stop %s\n", c,
                    accepted ? "OK" : "ignored (busy, fault or unknown command)",
                    snapshot.percent, snapshot.fault ? "LATCHED" : "clear");
    }
  }
  delay(1);
}
