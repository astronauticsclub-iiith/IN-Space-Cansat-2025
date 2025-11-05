#include "MQ135.h"

// Define the analog pin the sensor is connected to.
// Use a GPIO number for an ADC pin on the ESP32 (e.g., 34).
const int SENSOR_PIN = 34; 

// Create an MQ135 sensor object
MQ135 mq135_sensor = MQ135(SENSOR_PIN);

void setup() {
  // Initialize Serial communication at 9600 bits per second
  Serial.begin(9600);
  Serial.println("MQ-135 PPM Test on ESP32");
  Serial.println("Allowing sensor to warm up for 20 seconds...");
  delay(20000); // Wait for the sensor to heat up
  Serial.println("Warm-up complete. Starting readings.");
}

void loop() {
  // Get the corrected PPM value. The library handles the calculation.
  // This value is calibrated for CO2 by default.
  float ppm = mq135_sensor.getPPM();

  // Print the PPM value to the serial monitor
  Serial.print("CO2 PPM (approx): ");
  Serial.println(ppm);

  // Wait for two seconds before taking the next reading
  delay(2000);
}
