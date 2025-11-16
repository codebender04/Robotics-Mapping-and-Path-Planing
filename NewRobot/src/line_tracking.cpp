#include "line_tracking.h"
#include "motor.h"
#include <Arduino.h>

// Local state for smoothing & recovery
static unsigned long lastLineTime = 0;
static bool lineLost = false;

void followLineSmooth(bool s1, bool s2, bool s3, bool s4, bool s5) {
  // All sensors on line - STOP (non-blocking)
  if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1) {
    stop_car();
    lastLineTime = millis();
    lineLost = false;
    return;
  }

  // Perfect alignment - center sensor on line
  if (s3 == 1 && s2 == 0 && s4 == 0) {
    go_Advance(SPEED_FAST);
    lastLineTime = millis();
    lineLost = false;
  }
  // Slight left correction needed
  else if (s4 == 1 && s3 == 0) {
    go_GentleRight();
    lastLineTime = millis();
    lineLost = false;
  }
  // Slight right correction needed
  else if (s2 == 1 && s3 == 0) {
    go_GentleLeft();
    lastLineTime = millis();
    lineLost = false;
  }
  // Moderate left turn needed
  else if (s5 == 1) {
    go_Right();
    lastLineTime = millis();
    lineLost = false;
  }
  // Moderate right turn needed
  else if (s1 == 1) {
    go_Left();
    lastLineTime = millis();
    lineLost = false;
  }
  // Sharp left turn (right sensors on line)
  else if (s4 == 1 && s5 == 1) {
    go_Right();
    lastLineTime = millis();
    lineLost = false;
  }
  // Sharp right turn (left sensors on line)
  else if (s1 == 1 && s2 == 1) {
    go_Left();
    lastLineTime = millis();
    lineLost = false;
  }
  // Line lost - use recovery
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
    if (!lineLost) {
      lineLost = true;
      lastLineTime = millis();
    }
    recoverLine();
  }
  // Default case - keep moving forward
  else {
    go_Advance(SPEED_NORMAL);
  }
}

void recoverLine() {
  unsigned long lostTime = millis() - lastLineTime;
  
  if (lostTime < 800) {
    // Continue straight for a bit - might be a gap in line
    go_Advance(SPEED_NORMAL);
  }
  else if (lostTime < 2000) {
    // Gentle search pattern
    go_GentleRight();
  }
  else {
    // Aggressive search
    go_Right();
  }
}