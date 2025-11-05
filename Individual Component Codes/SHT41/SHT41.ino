#include <Wire.h>
#include "Adafruit_SHT4x.h"

// Create an instance of the SHT4x sensor
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  Serial.println("Adafruit SHT41 Test");

  // Initialize the sensor
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT41 sensor! Check your wiring.");
    while (1) delay(1); // Halt execution if sensor not found
  }

  Serial.println("Found SHT41 sensor");
  
  // You can set the precision and heater settings here if needed
  // sht4.setPrecision(SHT4X_HIGH_PRECISION);
  // sht4.setHeater(SHT4X_NO_HEATER);
}

void loop() {
  // Create sensor event variables to store the readings
  sensors_event_t humidity, temp;

  // Get the latest sensor events
  sht4.getEvent(&humidity, &temp);

  // Print the temperature
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  // Print the humidity
  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" % RH");

  // Add a newline for readability
  Serial.println("");

  // Wait 2 seconds before the next reading
  delay(2000);
}
