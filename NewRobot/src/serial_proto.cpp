#include "serial_proto.h"
#include "encoder.h"
#include "motor.h"
#include "config.h"
#include <Arduino.h>

static char serialBuf_[128];
static size_t serialPos_ = 0;

static void processSerialLine_(const char* line) {
  if (strcmp(line, "R") == 0) {
    resetEncoderCounts();
    safePrintln("A,RESET_OK");
  } else if (strcmp(line, "PING") == 0) {
    safePrintln("PONG");
  } else if (strcmp(line, "GET") == 0) {
    // will be sent by TelemetryTask periodically; GET requests an immediate frame with last-known sensors
    // Note: caller should call sendTelemetry with current snapshot; since we don't have it here, just echo ack
    safePrintln("A,GET_OK");
  } else if (strcmp(line, "STOP") == 0) {
    stop_car();
    safePrintln("A,STOP_OK");
  } else if (strncmp(line, "VEL,", 4) == 0) {
    // VEL,left_pwm,right_pwm,duration_ms
    int left_pwm, right_pwm, duration_ms;
    if (sscanf(line, "VEL,%d,%d,%d", &left_pwm, &right_pwm, &duration_ms) == 3) {
      // Clamp PWM values to safe range
      left_pwm = constrain(left_pwm, -255, 255);
      right_pwm = constrain(right_pwm, -255, 255);
      duration_ms = constrain(duration_ms, 0, 10000); // Max 10 seconds
      
      // Set motor directions and speeds
      if (left_pwm >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, left_pwm);
      } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, -left_pwm);
      }
      
      if (right_pwm >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, right_pwm);
      } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, -right_pwm);
      }
      
      // Stop after duration (this is a simple approach)
      delay(duration_ms);
      stop_car();
      
      safePrintln("A,VEL_OK");
    } else {
      safePrintln("A,VEL_ERROR,FORMAT");
    }
  } else if (strncmp(line, "FORWARD,", 8) == 0) {
    // FORWARD,speed,duration_ms
    int speed, duration_ms;
    if (sscanf(line, "FORWARD,%d,%d", &speed, &duration_ms) == 2) {
      speed = constrain(speed, 0, 255);
      duration_ms = constrain(duration_ms, 0, 10000);
      
      go_Advance(speed);
      delay(duration_ms);
      stop_car();
      
      safePrintln("A,FORWARD_OK");
    } else {
      safePrintln("A,FORWARD_ERROR,FORMAT");
    }
  } else if (strncmp(line, "TURN_LEFT,", 10) == 0) {
    // TURN_LEFT,duration_ms
    int duration_ms;
    if (sscanf(line, "TURN_LEFT,%d", &duration_ms) == 1) {
      duration_ms = constrain(duration_ms, 0, 5000); // Max 5 seconds
      
      go_Left();
      delay(duration_ms);
      stop_car();
      
      safePrintln("A,TURN_LEFT_OK");
    } else {
      safePrintln("A,TURN_LEFT_ERROR,FORMAT");
    }
  } else if (strncmp(line, "TURN_RIGHT,", 11) == 0) {
    // TURN_RIGHT,duration_ms
    int duration_ms;
    if (sscanf(line, "TURN_RIGHT,%d", &duration_ms) == 1) {
      duration_ms = constrain(duration_ms, 0, 5000); // Max 5 seconds
      
      go_Right();
      delay(duration_ms);
      stop_car();
      
      safePrintln("A,TURN_RIGHT_OK");
    } else {
      safePrintln("A,TURN_RIGHT_ERROR,FORMAT");
    }
  } else {
    String unknownMsg = "A,UNKNOWN_CMD," + String(line);
    safePrintln(unknownMsg);
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      serialBuf_[serialPos_] = '\0';
      if (serialPos_ > 0) {
        processSerialLine_(serialBuf_);
      }
      serialPos_ = 0;
    } else {
      if (serialPos_ < sizeof(serialBuf_) - 1) {
        serialBuf_[serialPos_++] = (char)c;
      }
    }
  }
}

void sendTelemetry(bool s1, bool s2, bool s3, bool s4, bool s5) {
  long lc = getLeftPulseCount();
  long rc = getRightPulseCount();
  unsigned long t = millis();
  
  String telemetryMsg = "T," + String(t) + "," + String(lc) + "," + String(rc) + "," + 
                       String((int)s1) + "," + String((int)s2) + "," + String((int)s3) + "," + 
                       String((int)s4) + "," + String((int)s5);
  safePrintln(telemetryMsg);
}
