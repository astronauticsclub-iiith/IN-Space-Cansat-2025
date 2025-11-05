#include <Arduino.h>
#include <DShotRMT.h>

const int ESC_PIN = 25;  // GPIO pin connected to ESC signal
DShotRMT dshot((gpio_num_t)ESC_PIN, DSHOT300);

void setup() {
  Serial.begin(115200);
  dshot.begin();

  // --- ESC Calibration / Arming ---
  dshot.sendThrottle(0);   // send 0 throttle
  delay(5000);             // wait 2 sec for ESC to calibrate/arm

  Serial.println("ESC armed. Motor will spin for 10 seconds...");
}

void loop() {
  int throttleValue = 1500;  // low RPM throttle (range: 0–2047)

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {  // run for 10 seconds
    dshot.sendThrottle(throttleValue);
    delay(20);  // refresh ESC signal
  }

  // --- Stop motor ---
  dshot.sendThrottle(0);
  Serial.println("Motor stopped.");

  while (true) {
    // Do nothing, keep motor stopped
  }
}
