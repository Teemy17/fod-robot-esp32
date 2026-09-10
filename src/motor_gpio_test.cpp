// Diagnostic only: motors receive full VM for one second. Raise wheels first.
// Select motor_gpio_test explicitly; the normal firmware remains the default.
#include <Arduino.h>
#include "config.h"

namespace {
bool running = false;
bool estopLatched = false;
uint32_t started = 0;
constexpr uint32_t kPulseMs = 1000;

// Do not let a host that is not reading serial prevent timed motor shutdown.
void report(const char* message) {
  const size_t n = strlen(message);
  if (Serial.availableForWrite() >= int(n + 2)) Serial.println(message);
}

void stopMotors() {
  digitalWrite(pins::kStandby, LOW);
  digitalWrite(pins::kLeftPwm, LOW);
  digitalWrite(pins::kRightPwm, LOW);
  digitalWrite(pins::kLeftIn1, LOW);
  digitalWrite(pins::kLeftIn2, LOW);
  digitalWrite(pins::kRightIn1, LOW);
  digitalWrite(pins::kRightIn2, LOW);
  running = false;
}

void setSide(int pwm, int in1, int in2, int direction) {
  digitalWrite(in1, direction > 0 ? HIGH : LOW);
  digitalWrite(in2, direction < 0 ? HIGH : LOW);
  digitalWrite(pwm, direction == 0 ? LOW : HIGH); // constant HIGH = full duty
}

void command(char c) {
  if (c >= 'a' && c <= 'z') c -= 'a' - 'A';
  if (c == '\r' || c == '\n' || c == ' ' || c == '\t') return;
  if (c == 'S') { stopMotors(); report("STOP: STBY=0"); return; }
  if (c == 'H' || c == '?') {
    report("GPIO TEST: A left, C right, F both, B reverse");
    report("S stop; E clear e-stop; full duty for 1 second");
    return;
  }
  if (c == 'E') {
    stopMotors();
    estopLatched = digitalRead(pins::kEstopSense) == LOW;
    report(estopLatched ? "BLOCKED: GPIO19 LOW" : "READY: e-stop cleared");
    return;
  }
  if (c != 'A' && c != 'C' && c != 'F' && c != 'B') {
    report("UNKNOWN: send A, C, F, B, S, E or H");
    return;
  }
  if (digitalRead(pins::kEstopSense) == LOW) estopLatched = true;
  if (estopLatched) { report("BLOCKED: release GPIO19, then send E"); return; }
  if (running) { report("BUSY: wait for STOP or send S"); return; }
  stopMotors();
  const int left = c == 'C' ? 0 : (c == 'B' ? -1 : 1);
  const int right = c == 'A' ? 0 : (c == 'B' ? -1 : 1);
  setSide(pins::kLeftPwm, pins::kLeftIn1, pins::kLeftIn2,
          left * prototype::kLeftSign);
  setSide(pins::kRightPwm, pins::kRightIn1, pins::kRightIn2,
          right * prototype::kRightSign);
  started = millis();
  running = true;
  digitalWrite(pins::kStandby, HIGH);
  report(c == 'A' ? "RUN A: left, full duty, 1 second" :
         c == 'C' ? "RUN C: right, full duty, 1 second" :
         c == 'F' ? "RUN F: both forward, full duty, 1 second" :
                    "RUN B: both reverse, full duty, 1 second");
}
}

void setup() {
  pinMode(pins::kStandby, OUTPUT);
  digitalWrite(pins::kStandby, LOW);
  for (int pin : {pins::kLeftPwm, pins::kLeftIn1, pins::kLeftIn2,
                  pins::kRightPwm, pins::kRightIn1, pins::kRightIn2}) {
    pinMode(pin, OUTPUT);
  }
  stopMotors();
  pinMode(pins::kEstopSense, INPUT_PULLUP);
  Serial.begin(115200);
  report("GPIO TEST READY: send H for help, A to test left");
}

void loop() {
  if (digitalRead(pins::kEstopSense) == LOW) {
    stopMotors();
    if (!estopLatched) report("E-STOP: GPIO19 LOW; motors disabled");
    estopLatched = true;
  }
  if (running && uint32_t(millis() - started) >= kPulseMs) {
    stopMotors();
    report("AUTO STOP: STBY=0");
  }
  if (Serial.available()) command(char(Serial.read()));
  delay(1);
}
