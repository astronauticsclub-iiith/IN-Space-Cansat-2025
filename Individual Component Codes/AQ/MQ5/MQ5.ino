// TESTED

const int SENSOR_ANALOG_PIN = 34; 

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  Serial.println("MQ-5 Sensor Test on ESP32 - Analog");
  Serial.println("Sensor warming up for 20 seconds...");
  
  // The sensor's heater needs time to stabilize
  delay(20000); 
  
  Serial.println("Warm-up complete. Starting readings.");
}

void loop() {
  // Read the raw analog value from the sensor (0-4095 on ESP32)
  int sensorValue = analogRead(SENSOR_ANALOG_PIN);

  // Print the value to the serial monitor
  Serial.print("Raw Sensor Value: ");
  Serial.println(sensorValue);

  delay(1000);
}
