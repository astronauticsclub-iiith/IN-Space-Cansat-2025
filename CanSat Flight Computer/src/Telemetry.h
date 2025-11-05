#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "config.h"

// --- Telemetry Field Name Constants (Competition Specification) ---
// Required 16 telemetry fields per competition rules
#define TELEMETRY_FIELD_TEAM_ID "TEAM_ID"
#define TELEMETRY_FIELD_TIME_STAMPING "TIME_STAMPING"
#define TELEMETRY_FIELD_PACKET_COUNT "PACKET_COUNT"
#define TELEMETRY_FIELD_ALTITUDE "ALTITUDE"
#define TELEMETRY_FIELD_PRESSURE "PRESSURE"
#define TELEMETRY_FIELD_TEMP "TEMP"
#define TELEMETRY_FIELD_VOLTAGE "VOLTAGE"
#define TELEMETRY_FIELD_GNSS_TIME "GNSS_TIME"
#define TELEMETRY_FIELD_GNSS_LATITUDE "GNSS_LATITUDE"
#define TELEMETRY_FIELD_GNSS_LONGITUDE "GNSS_LONGITUDE"
#define TELEMETRY_FIELD_GNSS_ALTITUDE "GNSS_ALTITUDE"
#define TELEMETRY_FIELD_GNSS_SATS "GNSS_SATS"
#define TELEMETRY_FIELD_ACCELEROMETER_DATA "ACCELEROMETER_DATA"
#define TELEMETRY_FIELD_GYRO_SPIN_RATE "GYRO_SPIN_RATE"
#define TELEMETRY_FIELD_FLIGHT_SOFTWARE_STATE "FLIGHT_SOFTWARE_STATE"
#define TELEMETRY_FIELD_OPTIONAL_DATA "OPTIONAL_DATA"

// --- Telemetry Data Structure ---
struct TelemetryData {
    unsigned long packetCount = 0;
    float timeStamp = 0.0f;
    float altitude = 0.0f;
    float pressure = 0.0f;
    float temperature = 0.0f;
    float voltage = 0.0f;
    float batteryPercent = 0.0f;
    float humidity = 0.0f;
    float gasResistance = 0.0f;
    char gnssTime[10] = "00:00:00";
    double gnssLatitude = 0.0;
    double gnssLongitude = 0.0;
    float gnssAltitude = 0.0f;
    int gnssSatellites = 0;
    float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
    float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
    FlightState flightState = BOOT;
    float verticalSpeed = 0.0f;
    bool gpsFix = false;
};

// --- External References ---
extern TelemetryData telemetryData;
extern float maxAltitude;
extern bool aerobrakeDeployed;
extern SemaphoreHandle_t spiMutex;
extern SemaphoreHandle_t servoMutex;
extern bool lora_ok;
extern volatile FlightState currentFlightState;
extern bool isInTestMode;

// Forward declarations from sensors.h
#ifdef ARDUINO
#include <ESP32Servo.h>
extern Servo parachuteServo;
#endif

// Forward declarations
const char* getStateName(FlightState state);
void transitionToState(FlightState newState);
void calibrateGroundAltitude();
void calibrateIMU();
void testServo();
void deployParachute();

/**
 * Get CSV header for telemetry
 */
const char* getTelemetryHeader() {
    return "TEAM_ID,TIME_STAMPING,PACKET_COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,"
           "GNSS_TIME,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE,GNSS_SATS,"
           "ACCEL_DATA,GYRO_DATA,FLIGHT_STATE,OPTIONAL_DATA";
}

/**
 * Format telemetry data as CSV string
 * Follows competition format specification
 */
String formatTelemetryCSV() {
    char buffer[400];
    char accelBuffer[40];
    char gyroBuffer[40];
    char optionalData[50];
    
    // Format accelerometer data (convert to m/s²)
    snprintf(accelBuffer, sizeof(accelBuffer), "%.2f;%.2f;%.2f", 
             telemetryData.accelX * 9.81f, 
             telemetryData.accelY * 9.81f, 
             telemetryData.accelZ * 9.81f);
    
    // Format gyroscope data (deg/s)
    snprintf(gyroBuffer, sizeof(gyroBuffer), "%.2f;%.2f;%.2f", 
             telemetryData.gyroX, 
             telemetryData.gyroY, 
             telemetryData.gyroZ);
    
    // Format optional data
    snprintf(optionalData, sizeof(optionalData), "BATT:%.0f%%;VS:%.1f;HUM:%.0f;GAS:%.1f", 
             telemetryData.batteryPercent, 
             telemetryData.verticalSpeed,
             telemetryData.humidity,
             telemetryData.gasResistance);
    
    // Format complete telemetry packet
    // Resolution specifications per competition requirements:
    // - %.1f altitude (0.1m resolution)
    // - %.0f pressure (1 Pa resolution)
    // - %.1f temperature (0.1°C resolution)
    // - %.2f voltage (0.01V resolution)
    // - %.4f gnssLatitude, gnssLongitude (0.0001° resolution)
    snprintf(buffer, sizeof(buffer),
             "%s,%.1f,%lu,%.1f,%.0f,%.1f,%.2f,%s,%.4f,%.4f,%.1f,%d,%s,%s,%d,%s",
             TEAM_ID,
             telemetryData.timeStamp,
             telemetryData.packetCount,
             telemetryData.altitude,         // %.1f - 0.1m resolution
             telemetryData.pressure,         // %.0f - 1 Pa resolution
             telemetryData.temperature,      // %.1f - 0.1°C resolution
             telemetryData.voltage,          // %.2f - 0.01V resolution
             telemetryData.gnssTime,
             telemetryData.gnssLatitude,     // %.4f - 0.0001° resolution
             telemetryData.gnssLongitude,    // %.4f - 0.0001° resolution
             telemetryData.gnssAltitude,
             telemetryData.gnssSatellites,
             accelBuffer,
             gyroBuffer,
             (int)telemetryData.flightState,
             optionalData
    );
    
    return String(buffer);
}

/**
 * Send telemetry packet via LoRa (thread-safe)
 */
void sendLoRaTelemetry() {
    if (!lora_ok) return;
    
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        String telemetryString = formatTelemetryCSV();
        
        LoRa.beginPacket();
        LoRa.print(telemetryString);
        LoRa.endPacket();
        
        xSemaphoreGive(spiMutex);
    }
}

/**
 * Receive and process LoRa commands (thread-safe)
 * Supports multiple command types for testing and control
 */
void receiveLoRaCommands() {
    if (!lora_ok) return;
    
    int packetSize = 0;
    
    // Check for packet (thread-safe)
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        packetSize = LoRa.parsePacket();
        xSemaphoreGive(spiMutex);
    }
    
    if (packetSize == 0) return;
    
    // Read command
    String cmd = "";
    while (LoRa.available()) {
        cmd += (char)LoRa.read();
    }
    cmd.trim();
    
    logEvent("CMD_RX: " + cmd);
    
    // Process commands
    if (cmd.equalsIgnoreCase("CMD,CAL_ALT")) {
        calibrateGroundAltitude();
    }
    else if (cmd.equalsIgnoreCase("CMD,CAL_IMU")) {
        calibrateIMU();
    }
    else if (cmd.equalsIgnoreCase("CMD,RESET_STATE")) {
        transitionToState(LAUNCH_PAD);
    }
    else if (cmd.equalsIgnoreCase("CMD,STATUS")) {
        String status = "STATUS: State=" + String(getStateName(currentFlightState)) +
                       ", Alt=" + String(telemetryData.altitude, 1) + "m" +
                       ", VS=" + String(telemetryData.verticalSpeed, 1) + "m/s" +
                       ", Batt=" + String(telemetryData.batteryPercent, 0) + "%";
        logEvent(status);
    }
    else if (cmd.equalsIgnoreCase("CMD,SET_MODE,TEST")) {
        isInTestMode = true;
        transitionToState(TEST_MODE);
    }
    else if (cmd.equalsIgnoreCase("CMD,SET_MODE,FLIGHT")) {
        isInTestMode = false;
        transitionToState(LAUNCH_PAD);
    }
    else if (cmd.equalsIgnoreCase("CMD,TEST_SERVO")) {
        if (currentFlightState == LAUNCH_PAD || currentFlightState == TEST_MODE) {
            testServo();
        } else {
            logEvent("ERROR: Servo test only allowed on ground");
        }
    }
    else if (cmd.equalsIgnoreCase("CMD,DEPLOY_TEST")) {
        if (currentFlightState == LAUNCH_PAD || currentFlightState == TEST_MODE) {
            logEvent("Manual deployment test");
            deployParachute();
            delay(2000);
            if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                parachuteServo.write(SERVO_NEUTRAL_ANGLE);
                xSemaphoreGive(servoMutex);
            }
        } else {
            logEvent("ERROR: Deploy test only allowed on ground");
        }
    }
    else if (cmd.equalsIgnoreCase("CMD,START_TX")) {
        lora_ok = true;
        logEvent("Telemetry transmission ENABLED");
    }
    else if (cmd.equalsIgnoreCase("CMD,STOP_TX")) {
        lora_ok = false;
        logEvent("Telemetry transmission DISABLED");
    }
    else {
        logEvent("ERROR: Unknown command: " + cmd);
    }
}

/**
 * Send acknowledgment for received command
 */
void sendCommandAck(const String& cmd, bool success) {
    if (!lora_ok) return;
    
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        LoRa.beginPacket();
        LoRa.print("ACK," + cmd + "," + (success ? "OK" : "FAIL"));
        LoRa.endPacket();
        xSemaphoreGive(spiMutex);
    }
}

#endif
