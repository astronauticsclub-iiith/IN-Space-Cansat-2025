#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <LoRa.h>
#include <Preferences.h>

// --- Team Configuration ---
#define TEAM_ID "2024ASI-CANSAT-044"
#define SEALEVELPRESSURE_HPA 1013.25

// --- Pin Definitions ---
#define LORA_SS_PIN 5
#define LORA_RST_PIN 14
#define LORA_DIO0_PIN 2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define SERVO_PIN 13
#define VOLTAGE_PIN 34
#define SD_CS_PIN 15

// --- I2C Addresses ---
#define LPS22_ADDR 0x5D
#define BME680_ADDR 0x76
#define ICM_ADDR 0x69
#define OLED_ADDR 0x3C

// --- Gas Sensor Pins ---
#define MQ135_PIN 35       // CO2/Air quality sensor
#define MQ5_PIN 39         // LPG/Natural gas sensor

// --- Servo Configuration ---
#define SERVO_NEUTRAL_ANGLE 90
#define SERVO_DEPLOY_ANGLE 0
#define SERVO_RELEASE_ANGLE 180

// --- Battery Configuration ---
#define VOLTAGE_DIVIDER_RATIO 2.0
#define BATTERY_MAX_VOLTAGE 4.2
#define BATTERY_MIN_VOLTAGE 3.0

// --- Flight Parameters ---
#define LAUNCH_ACCEL_THRESHOLD 2.5  // g's
#define APOGEE_DESCENT_THRESHOLD -5.0 // m/s
#define IMPACT_ALTITUDE_THRESHOLD 5.0 // meters AGL
#define IMPACT_VELOCITY_THRESHOLD 2.0 // m/s
#define DEPLOY_ALTITUDE 400.0 // meters AGL
#define AEROBRAKE_RELEASE_ALTITUDE 50.0 // meters AGL

// --- Timing Configuration ---
#define TELEMETRY_INTERVAL 1000 // ms (1 Hz)
#define SENSOR_READ_INTERVAL 100 // ms (10 Hz)
#define SD_FLUSH_INTERVAL 5000 // ms

// --- Flight State Enum ---
enum FlightState : uint8_t {
    BOOT = 0,
    TEST_MODE = 1,
    LAUNCH_PAD = 2,
    ASCENT = 3,
    ROCKET_DEPLOY = 4,
    DESCENT = 5,
    AEROBRAKE_RELEASE = 6,
    IMPACT = 7
};

// --- GPS Serial ---
#define GPS_SERIAL Serial2

// --- Logging Function Declaration ---
void logEvent(const String& message);

#endif
