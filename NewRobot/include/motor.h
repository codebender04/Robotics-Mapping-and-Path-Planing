#pragma once
#include <Arduino.h>
#include "config.h"

// Motor control functions
void go_Advance(int speed);
void go_GentleLeft();
void go_GentleRight();
void go_Left();
void go_Right();
void go_SharpLeft();
void go_SharpRight();
void stop_car();
