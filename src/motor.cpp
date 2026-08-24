#include "motor.h"
#include "config.h"

void MotorChannel::begin(int pwmPin, int in1Pin, int in2Pin, int ledcChannel) {
  pwmPin_ = pwmPin;
  in1_ = in1Pin;
  in2_ = in2Pin;
  ledcChannel_ = ledcChannel;
  pinMode(in1_, OUTPUT);
  pinMode(in2_, OUTPUT);
  ledcSetup(ledcChannel_, pins::kPwmFreqHz, pins::kPwmResolutionBits);
  ledcAttachPin(pwmPin_, ledcChannel_);
  stop();
}

void MotorChannel::write(int32_t signedDuty) {
  constexpr int32_t kMax = (1 << pins::kPwmResolutionBits) - 1;
  signedDuty = constrain(signedDuty, -kMax, kMax);
  if (signedDuty >= 0) {
    digitalWrite(in1_, HIGH);
    digitalWrite(in2_, LOW);
  } else {
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, HIGH);
    signedDuty = -signedDuty;
  }
  ledcWrite(ledcChannel_, signedDuty);
}

void MotorChannel::brake() {
  digitalWrite(in1_, LOW);
  digitalWrite(in2_, LOW);
  ledcWrite(ledcChannel_, 0);
}
