#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Battery monitoring constants
extern const int LED_PIN;
extern const int VOLTAGE_PIN;
extern const float R1;
extern const float R2;
extern const float ADC_MAX_VALUE;
extern const float ADC_REF_VOLTAGE;
extern const float LOW_BATT_THRESHOLD;
extern const int BLINK_INTERVAL;

// Function declarations
void batteryInit();
float readBatteryVoltage();
void BatteryTask(void* parameter);

// Thread-safe serial functions (declared in main.cpp)
extern void safePrint(const String& message);
extern void safePrintln(const String& message);

#endif // BATTERY_H