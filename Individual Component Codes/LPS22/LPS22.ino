//TESTED 
#include <Wire.h>
#include "Adafruit_LPS2X.h"

Adafruit_LPS22 lps;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LPS22 Test");

  if (!lps.begin_I2C()) {
    Serial.println("Failed to find LPS22 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("LPS22 Found!");

  lps.setDataRate(LPS22_RATE_10_HZ);
}

void loop() {
  sensors_event_t pressure, temp;
  lps.getEvent(&pressure, &temp);

  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" C");
  Serial.print("Pressure: "); Serial.print(pressure.pressure); Serial.println(" hPa");
  
  Serial.println("");
  delay(1000);
}
