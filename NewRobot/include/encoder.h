#pragma once
#include <Arduino.h>
#include "config.h"

// Initialize encoder pins and attach interrupts. Handles pin conflicts.
void encoderInit();

// ISR prototypes (attached internally)
void IRAM_ATTR onLeftA();
void IRAM_ATTR onRightA();

// Pulse counter API
long getLeftPulseCount();
long getRightPulseCount();
void resetEncoderCounts();
