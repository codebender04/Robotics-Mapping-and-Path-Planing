#pragma once
#include <Arduino.h>

// Process any pending serial input (line-buffered) and react to commands.
void handleSerial();

// Compose and send one telemetry frame with the provided sensor snapshot.
void sendTelemetry(bool s1, bool s2, bool s3, bool s4, bool s5);

// Thread-safe serial functions (declared in main.cpp)
extern void safePrint(const String& message);
extern void safePrintln(const String& message);
