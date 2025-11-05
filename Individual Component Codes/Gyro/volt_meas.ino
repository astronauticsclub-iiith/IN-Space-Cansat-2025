#include <Arduino.h>
#include "esp_adc_cal.h" // Include the ADC calibration library

const int batteryPin = 36; // ADC1_CH0

// --- CALIBRATION ---
// We measured 12.28V with a multimeter when the ESP32 Vout was 3.021V.
// The true divider ratio is 12.28 / 3.021 = 4.0648
// Use this constant instead of R1 and R2 for better accuracy.
const float VOLTAGE_DIVIDER_RATIO = 4.0648;

/*
// Original (and incorrect) resistor values
const float R2 = 27000.0; // Ohms, battery+ to ADC pin
const float R1 = 100000.0; // Ohms, ADC pin to GND
*/

// Define the attenuation for the ESP-IDF functions
static const adc_atten_t atten = ADC_ATTEN_DB_11;
// Define the attenuation for the Arduino API functions
static const adc_attenuation_t arduino_atten = ADC_11db; // Fixed

// Global variable to store ADC calibration characteristics
static esp_adc_cal_characteristics_t *adc_chars;

void setup() {
  Serial.begin(115200);

  // --- ADC Calibration Setup ---
  analogSetPinAttenuation(batteryPin, arduino_atten);
  analogSetWidth(12);
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, 1100, adc_chars);
  // -----------------------------
}

void loop() {
  // Get the raw ADC reading
  int adcValue = analogRead(batteryPin);

  // --- Convert raw ADC to calibrated voltage ---
  uint32_t vout_mv = esp_adc_cal_raw_to_voltage(adcValue, adc_chars);

  // Convert from millivolts (mV) to volts (V)
  float vout = (float)vout_mv / 1000.0;
  // ---------------------------------------------

  // Calculate the original battery voltage by reversing the voltage divider
  // float vbat = vout * (R1 + R2) / R2; // <-- OLD FORMULA
  float vbat = vout * VOLTAGE_DIVIDER_RATIO; // <-- NEW, ACCURATE FORMULA

  Serial.print("ADC Raw: "); Serial.print(adcValue);
  Serial.print(" | Vout (calibrated): "); Serial.print(vout, 3); Serial.print(" V");
  Serial.print(" ("); Serial.print(vout_mv); Serial.print(" mV)");
  Serial.print(" | Vbat (calculated): "); Serial.print(vbat, 3); Serial.println(" V");

  delay(1000);
}
