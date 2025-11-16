#include <Wire.h>
#include <PCF8574.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "config.h"
#include "motor.h"
#include "line_tracking.h"
#include "encoder.h"
#include "serial_proto.h"
#include "battery.h"
#include "odometry.h"

// Pins, sensors and speed constants are in config.h

// Global variables
PCF8574 pcf(PCF_ADDR);

// Line-follow smoothing state is handled inside line_tracking.cpp

// Latest sensor snapshot for telemetry
volatile bool lastS1 = false, lastS2 = false, lastS3 = false, lastS4 = false, lastS5 = false;

// Serial telemetry/command handling
const unsigned long telemetryPeriodMs = 50; // 20 Hz
static char serialBuf[128];
static size_t serialPos = 0;

// FreeRTOS task handles
TaskHandle_t hControlTask = nullptr;
TaskHandle_t hSerialTask = nullptr;
TaskHandle_t hTelemetryTask = nullptr;
TaskHandle_t hBatteryTask = nullptr;

// Serial mutex for thread-safe printing
SemaphoreHandle_t serialMutex = nullptr;

// Function declarations (local module functions declared below)

// Tasks
void ControlTask(void*);
void SerialTask(void*);
void TelemetryTask(void*);

// Thread-safe serial functions
void safePrint(const String& message);
void safePrintln(const String& message);

void setup() {
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize PCF8574
  if (!pcf.begin()) {
    Serial.println("PCF8574 not found!");
    while(1);
  }
  
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  Serial.begin(115200);
  
  // Create serial mutex before any tasks start
  serialMutex = xSemaphoreCreateMutex();
  if (serialMutex == NULL) {
    Serial.println("FATAL: Failed to create serial mutex!");
    while(1);
  }
  
  // Initialize encoders
  encoderInit();
  
  // Initialize battery monitoring
  batteryInit();
  
  stop_car();
  safePrintln("=== SMOOTH LINE FOLLOWER (RTOS) ===");

  // Spawn RTOS tasks (use core 1 for control, 0 for IO)
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, nullptr, 2, &hControlTask, 1);
  xTaskCreatePinnedToCore(SerialTask,  "SerialTask",  4096, nullptr, 2, &hSerialTask,  0);
  xTaskCreatePinnedToCore(TelemetryTask,"TelemetryTask",3072, nullptr, 1, &hTelemetryTask,0);
  xTaskCreatePinnedToCore(BatteryTask, "BatteryTask", 2048, nullptr, 1, &hBatteryTask, 0);
}

void loop() {
  vTaskDelay(1);
}

// (all logic except tasks moved to modules)

// ==========================
// FreeRTOS Tasks
// ==========================

void ControlTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz control
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    // Read all line sensors
    bool s1 = pcf.read(S1_SENSOR_BIT);
    bool s2 = pcf.read(S2_SENSOR_BIT);
    bool s3 = pcf.read(S3_SENSOR_BIT);
    bool s4 = pcf.read(S4_SENSOR_BIT);
    bool s5 = pcf.read(S5_SENSOR_BIT);

    // Update snapshot (for telemetry)
    lastS1 = s1; lastS2 = s2; lastS3 = s3; lastS4 = s4; lastS5 = s5;

    // Run line follower step (non-blocking)
    // followLineSmooth(s1, s2, s3, s4, s5);  // DISABLED for rectangle navigation

    vTaskDelayUntil(&last, period);
  }
}

void SerialTask(void*) {
  for (;;) {
    handleSerial();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void TelemetryTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(telemetryPeriodMs);
  for (;;) {
    // Send telemetry using last snapshot collected in ControlTask
    sendTelemetry(lastS1, lastS2, lastS3, lastS4, lastS5);
    vTaskDelay(period);
  }
}

// ==========================
// Thread-Safe Serial Functions
// ==========================

void safePrint(const String& message) {
  if (serialMutex != nullptr && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.print(message);
    xSemaphoreGive(serialMutex);
  }
}

void safePrintln(const String& message) {
  if (serialMutex != nullptr && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println(message);
    xSemaphoreGive(serialMutex);
  }
}