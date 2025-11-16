// Central configuration for pins, sensors, speeds and I2C
#pragma once

// I2C pins for PCF8574
#define SDA_PIN 21
#define SCL_PIN 22
#define PCF_ADDR 0x20

// Sensor bit positions on PCF8574
#define S1_SENSOR_BIT 4 // Far left
#define S2_SENSOR_BIT 3 // Left
#define S3_SENSOR_BIT 2 // Center
#define S4_SENSOR_BIT 1 // Right
#define S5_SENSOR_BIT 0 // Far right

// Motor driver pins
#define IN1 16
#define IN2 25
#define ENA 18
#define IN3 33
#define IN4 32
#define ENB 4

// Quadrature encoder pins
#define leftENCA 26
#define leftENCB 27
#define rightENCA 14
#define rightENCB 13

// Speed constants
#define SPEED_FAST 220    // Straight line speed
#define SPEED_NORMAL 200  // Normal speed
#define SPEED_TURN 180    // Gentle turns
#define SPEED_SHARP 160   // Sharp turns
