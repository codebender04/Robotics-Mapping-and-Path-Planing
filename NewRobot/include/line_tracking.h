#pragma once

#include <Arduino.h>
#include "motor.h"
#include "config.h"

// Run one non-blocking step of the line-following controller
void followLineSmooth(bool s1, bool s2, bool s3, bool s4, bool s5);

// Recovery behavior when line is lost (invoked by followLineSmooth)
void recoverLine();
