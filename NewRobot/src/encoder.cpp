#include "encoder.h"

// Internal state
static volatile long leftPulseCount_ = 0;
static volatile long rightPulseCount_ = 0;
static bool leftEnabled_ = false;
static bool rightEnabled_ = false;

void encoderInit() {
  // Check for pin conflicts with motor pins
  if (leftENCA != IN1 && leftENCA != IN2 && leftENCA != ENA &&
      leftENCA != IN3 && leftENCA != IN4 && leftENCA != ENB) {
    pinMode(leftENCA, INPUT_PULLUP);
    pinMode(leftENCB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(leftENCA), onLeftA, RISING);
    leftEnabled_ = true;
  } else {
    Serial.println("[Encoder] WARNING: leftENCA pin conflicts with motor pins. Left encoder disabled.");
    leftEnabled_ = false;
  }

  if (rightENCA != IN1 && rightENCA != IN2 && rightENCA != ENA &&
      rightENCA != IN3 && rightENCA != IN4 && rightENCA != ENB) {
    pinMode(rightENCA, INPUT_PULLUP);
    pinMode(rightENCB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rightENCA), onRightA, RISING);
    rightEnabled_ = true;
  } else {
    Serial.println("[Encoder] WARNING: rightENCA pin conflicts with motor pins. Right encoder disabled.");
    rightEnabled_ = false;
  }
}

void IRAM_ATTR onLeftA() {
  bool b = digitalRead(leftENCB);
  leftPulseCount_ += b ? -1 : 1;
}

void IRAM_ATTR onRightA() {
  bool b = digitalRead(rightENCB);
  rightPulseCount_ += b ? 1 : -1;  // Flipped the logic to fix backwards counting
}

long getLeftPulseCount() {
  noInterrupts();
  long v = leftPulseCount_;
  interrupts();
  return v;
}

long getRightPulseCount() {
  noInterrupts();
  long v = rightPulseCount_;
  interrupts();
  return v;
}

void resetEncoderCounts() {
  noInterrupts();
  leftPulseCount_ = 0;
  rightPulseCount_ = 0;
  interrupts();
}
