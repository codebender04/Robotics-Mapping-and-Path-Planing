#include "battery.h"

// Battery monitoring constants
const int LED_PIN = 2; 
const int VOLTAGE_PIN = 34;
const float R1 = 16000.0; 
const float R2 = 10000.0; 
const float ADC_MAX_VALUE = 4095.0; 
const float ADC_REF_VOLTAGE = 3.3; 
const float LOW_BATT_THRESHOLD = 7.5; 
const int BLINK_INTERVAL = 250;

void batteryInit() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

float readBatteryVoltage() {
  int adcRawValue = analogRead(VOLTAGE_PIN);
  float adcVoltage = (adcRawValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
  float batteryVoltage = adcVoltage * (R1 + R2) / R2;
  return batteryVoltage;
}

void BatteryTask(void* parameter) {
  const TickType_t period = pdMS_TO_TICKS(2000); // Check battery every 2 seconds
  TickType_t lastBlink = 0;
  bool ledState = false;
  
  for (;;) {
    TickType_t now = xTaskGetTickCount();
    
    // Read battery voltage
    int adcRawValue = analogRead(VOLTAGE_PIN);
    float adcVoltage = (adcRawValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float batteryVoltage = adcVoltage * (R1 + R2) / R2;
    
    // Send battery info via serial (with BATT prefix for easy parsing)
    String battMessage = "BATT," + String(adcRawValue) + "," + String(batteryVoltage, 2);
    safePrintln(battMessage);
    
    // Handle low battery warning LED
    if (batteryVoltage < LOW_BATT_THRESHOLD) {
      // Blink LED every 250ms when battery is low
      if (now - lastBlink >= pdMS_TO_TICKS(BLINK_INTERVAL)) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        lastBlink = now;
      }
    } else {
      // Turn off LED when battery is OK
      digitalWrite(LED_PIN, LOW);
      ledState = false;
    }
    
    vTaskDelay(period);
  }
}