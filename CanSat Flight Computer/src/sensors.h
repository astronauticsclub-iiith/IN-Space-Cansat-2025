#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "config.h"
#include "Telemetry.h"  // Include before using TelemetryData
#include <Adafruit_LPS2X.h>
#include "ICM_20948.h"
#include <TinyGPSPlus.h>
#include "KalmanFilter.h"
#include <ESP32Servo.h>
#include <SPI.h>

// Sensor objects (defined in main)
extern Adafruit_LPS22 lps;
extern Adafruit_BME680 bme;
extern ICM_20948_I2C myICM;
extern TinyGPSPlus gps;
extern Servo parachuteServo;
extern KalmanFilter altitudeKF;
extern SensorFusion sensorFusion;

// Calibration data
struct SensorCalibration {
    float gyro_offset_x = 0.0f;
    float gyro_offset_y = 0.0f;
    float gyro_offset_z = 0.0f;
    float accel_offset_x = 0.0f;
    float accel_offset_y = 0.0f;
    float accel_offset_z = 0.0f;
    bool calibrated = false;
};
extern SensorCalibration sensorCal;

// Sensor status flags
extern bool lps_ok, bme_ok, icm_ok, gps_ok, servo_ok, lora_ok, oled_ok, sdCardOK;

// External data
extern struct TelemetryData telemetryData;
extern float groundAltitude;
extern SemaphoreHandle_t servoMutex;

// ============================================
// Gas Sensor Definitions (merged from GasSensors.h)
// ============================================

// Pin definitions for gas sensors
#define MQ135_PIN 35       // CO2/Air quality sensor
#define MQ5_PIN 39         // LPG/Natural gas sensor

// Calibration constants for MQ135 (CO2 measurement)
#define MQ135_RZERO 76.63f  // Calibrated resistance in clean air
#define MQ135_PARA 116.6020682f
#define MQ135_PARB 2.769034857f
#define MQ135_RL 10.0f     // Load resistance in kOhms

// MQ5 thresholds (raw analog values)
#define MQ5_CLEAN_AIR_VALUE 300    // Typical value in clean air
#define MQ5_GAS_THRESHOLD 600      // Threshold for gas detection

// Gas sensor data structure
struct GasSensorData {
    float co2_ppm = 0.0f;          // CO2 concentration in PPM
    float air_quality = 0.0f;      // Air quality index (0-100)
    int lpg_level = 0;             // LPG level (0-4095 raw, or processed)
    bool gas_detected = false;     // LPG/gas detection flag
    bool sensors_ready = false;    // Warmup complete
    uint32_t warmup_start_time = 0;
};

extern GasSensorData gasSensorData;
extern bool mq135_ok;
extern bool mq5_ok;

// Gas sensor function prototypes
void initializeGasSensors();
float calculateCO2PPM(int analogValue);
float calculateAirQuality(float co2_ppm);
void processMQ5Reading(int analogValue);
void readGasSensors();
String getGasSensorStatus();
void checkGasSafetyAlerts();

/**
 * Initialize all sensors with error handling
 */
void initializeSensors() {
    logEvent("=== Sensor Initialization ===");
    
    // LPS22 Barometric Altimeter
    if (lps.begin_I2C(LPS22_ADDR)) {
        lps.setDataRate(LPS22_RATE_75_HZ);
        lps_ok = true;
        logEvent("✓ LPS22 Altimeter OK");
    } else {
        logEvent("✗ LPS22 FAILED");
    }
    
    // ICM-20948 9-DOF IMU
    myICM.begin(Wire, ICM_ADDR);
    if (myICM.status == ICM_20948_Stat_Ok) {
        // Configure IMU
        ICM_20948_smplrt_t sampleRate;
        sampleRate.a = 9;  // 1.125 kHz / (1 + 9) = 112.5 Hz
        sampleRate.g = 9;
        myICM.setSampleRate(0, sampleRate);
        myICM.setSampleRate(1, sampleRate);
        
        icm_ok = true;
        logEvent("✓ ICM-20948 IMU OK");
    } else {
        logEvent("✗ ICM-20948 FAILED (Status: " + String(myICM.status) + ")");
    }
    
    // LoRa Radio
    LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (LoRa.begin(433E6)) {
        LoRa.setSpreadingFactor(7);
        LoRa.setSignalBandwidth(125E3);
        LoRa.setCodingRate4(5);
        LoRa.enableCrc();
        lora_ok = true;
        logEvent("✓ LoRa Radio OK (433 MHz)");
    } else {
        logEvent("✗ LoRa FAILED");
    }
    
    // GPS Module
    GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    gps_ok = true;
    logEvent("✓ GPS Serial Started");
    
    // Parachute Servo
    ESP32PWM::allocateTimer(0);
    parachuteServo.setPeriodHertz(50);
    parachuteServo.attach(SERVO_PIN, 500, 2500);
    parachuteServo.write(SERVO_NEUTRAL_ANGLE);
    servo_ok = true;
    logEvent("✓ Servo OK (Neutral: " + String(SERVO_NEUTRAL_ANGLE) + "°)");

    // Gas Sensors

    initializeGasSensors();
    logEvent("=== Initialization Complete ===");
}

/**
 * Calibrate ground altitude using averaging
 */
void calibrateGroundAltitude() {
    if (!lps_ok) {
        logEvent("Cannot calibrate - LPS22 not available");
        return;
    }
    
    logEvent("Calibrating ground altitude...");
    
    const int samples = 100;
    float total_pressure = 0.0f;
    int valid_samples = 0;
    
    for (int i = 0; i < samples; i++) {
        sensors_event_t p_evt, t_evt;
        if (lps.getEvent(&p_evt, &t_evt)) {
            total_pressure += p_evt.pressure;
            valid_samples++;
        }
        delay(10);
    }
    
    if (valid_samples > 0) {
        float avg_pressure = total_pressure / valid_samples;
        groundAltitude = 44330.0f * (1.0f - pow(avg_pressure / SEALEVELPRESSURE_HPA, 0.1903f));
        altitudeKF.reset();
        logEvent("Ground altitude: " + String(groundAltitude, 2) + " m (from " + 
                 String(valid_samples) + " samples)");
    } else {
        logEvent("ERROR: No valid pressure readings");
    }
}

/**
 * Calibrate IMU gyroscope and accelerometer offsets
 */
void calibrateIMU() {
    if (!icm_ok) {
        logEvent("Cannot calibrate - ICM-20948 not available");
        return;
    }
    
    logEvent("Calibrating IMU - Keep still for 3 seconds...");
    
    const int samples = 300;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    int valid_samples = 0;
    
    for (int i = 0; i < samples; i++) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            gx_sum += myICM.gyrX();
            gy_sum += myICM.gyrY();
            gz_sum += myICM.gyrZ();
            ax_sum += myICM.accX();
            ay_sum += myICM.accY();
            az_sum += myICM.accZ();
            valid_samples++;
        }
        delay(10);
    }
    
    if (valid_samples > 0) {
        sensorCal.gyro_offset_x = gx_sum / valid_samples;
        sensorCal.gyro_offset_y = gy_sum / valid_samples;
        sensorCal.gyro_offset_z = gz_sum / valid_samples;
        sensorCal.accel_offset_x = ax_sum / valid_samples;
        sensorCal.accel_offset_y = ay_sum / valid_samples;
        sensorCal.accel_offset_z = (az_sum / valid_samples) - 1.0f; // Remove gravity
        sensorCal.calibrated = true;
        
        logEvent("IMU calibrated (" + String(valid_samples) + " samples)");
        logEvent("Gyro offsets: X=" + String(sensorCal.gyro_offset_x, 3) + 
                 " Y=" + String(sensorCal.gyro_offset_y, 3) + 
                 " Z=" + String(sensorCal.gyro_offset_z, 3));
    } else {
        logEvent("ERROR: No valid IMU readings");
    }
}

/**
 * Read all sensors and update telemetry data
 * Non-blocking GPS parsing
 */
void readAllSensors() {
    static uint32_t lastReadTime = 0;
    uint32_t currentTime = millis();
    float dt = (currentTime - lastReadTime) / 1000.0f;
    lastReadTime = currentTime;
    
    telemetryData.timeStamp = currentTime / 1000.0f;
    telemetryData.packetCount++;
    
    // Read both barometric sensors
    float lps_pressure = 0, bme_pressure = 0;
    float lps_temp = 0, bme_temp = 0;
    bool lps_valid = false, bme_valid = false;
    
    if (lps_ok) {
        sensors_event_t p_evt, t_evt;
        if (lps.getEvent(&p_evt, &t_evt)) {
            lps_pressure = p_evt.pressure * 100.0f; // hPa to Pa
            lps_temp = t_evt.temperature;
            lps_valid = true;
        }
    }
    
    if (bme_ok) {
        if (bme.performReading()) {
            bme_pressure = bme.pressure; // Pa
            bme_temp = bme.temperature;
            bme_valid = true;
            
            // Store BME680 unique data
            telemetryData.humidity = bme.humidity;
            telemetryData.gasResistance = bme.gas_resistance / 1000.0f; // kOhms
        }
    }
    
    // Fuse sensor readings
    telemetryData.pressure = sensorFusion.fusePressure(lps_pressure, bme_pressure, lps_valid, bme_valid);
    telemetryData.temperature = sensorFusion.fuseTemperature(lps_temp, bme_temp, lps_valid, bme_valid);
    
    // Calculate altitude from fused pressure
    float raw_alt = 44330.0f * (1.0f - pow(telemetryData.pressure / (SEALEVELPRESSURE_HPA * 100.0f), 0.1903f));
    raw_alt -= groundAltitude;
    
    // Apply Kalman filter
    telemetryData.altitude = altitudeKF.updateEstimate(raw_alt, dt);
    telemetryData.verticalSpeed = altitudeKF.getVelocity();
    
    // IMU (Accelerometer + Gyroscope)
    if (icm_ok && myICM.dataReady()) {
        myICM.getAGMT();
        telemetryData.accelX = myICM.accX() - sensorCal.accel_offset_x;
        telemetryData.accelY = myICM.accY() - sensorCal.accel_offset_y;
        telemetryData.accelZ = myICM.accZ() - sensorCal.accel_offset_z;
        telemetryData.gyroX = myICM.gyrX() - sensorCal.gyro_offset_x;
        telemetryData.gyroY = myICM.gyrY() - sensorCal.gyro_offset_y;
        telemetryData.gyroZ = myICM.gyrZ() - sensorCal.gyro_offset_z;
    }
    
    // GPS (Non-blocking)
    int gpsCharsProcessed = 0;
    while (GPS_SERIAL.available() > 0 && gpsCharsProcessed < 100) {
        gps.encode(GPS_SERIAL.read());
        gpsCharsProcessed++;
    }
    
    if (gps.location.isUpdated()) {
        telemetryData.gpsFix = gps.location.isValid();
        telemetryData.gnssLatitude = gps.location.lat();
        telemetryData.gnssLongitude = gps.location.lng();
        telemetryData.gnssAltitude = gps.altitude.meters();
        telemetryData.gnssSatellites = gps.satellites.value();
        
        if (gps.time.isValid()) {
            snprintf(telemetryData.gnssTime, sizeof(telemetryData.gnssTime), 
                    "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
        }
        
        // Cross-validate barometric altitude with GPS
        telemetryData.altitude = sensorFusion.fuseAltitude(telemetryData.altitude, 
                                                          telemetryData.gnssAltitude, 
                                                          telemetryData.gpsFix);
    }
    
    // Battery Voltage & Percentage
    float rawVoltage = analogRead(VOLTAGE_PIN) / 4095.0f * 3.3f * VOLTAGE_DIVIDER_RATIO;
    telemetryData.voltage = telemetryData.voltage * 0.9f + rawVoltage * 0.1f;
    telemetryData.batteryPercent = ((telemetryData.voltage - BATTERY_MIN_VOLTAGE) / 
                                   (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0f;
    telemetryData.batteryPercent = constrain(telemetryData.batteryPercent, 0.0f, 100.0f);
}

/**
 * Get human-readable state name
 */
const char* getStateName(FlightState state) {
    switch (state) {
        case BOOT: return "BOOT";
        case TEST_MODE: return "TEST_MODE";
        case LAUNCH_PAD: return "LAUNCH_PAD";
        case ASCENT: return "ASCENT";
        case ROCKET_DEPLOY: return "ROCKET_DEPLOY";
        case DESCENT: return "DESCENT";
        case AEROBRAKE_RELEASE: return "AEROBRAKE_RELEASE";
        case IMPACT: return "IMPACT";
        default: return "UNKNOWN";
    }
}

/**
 * Deploy parachute (thread-safe)
 */
void deployParachute() {
    if (!servo_ok) {
        logEvent("ERROR: Cannot deploy - servo not initialized");
        return;
    }
    
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        logEvent("DEPLOYING PARACHUTE!");
        parachuteServo.write(SERVO_DEPLOY_ANGLE);
        xSemaphoreGive(servoMutex);
    } else {
        logEvent("ERROR: Could not acquire servo mutex");
    }
}

/**
 * Release aerobrake (thread-safe)
 */
void releaseAerobrake() {
    if (!servo_ok) {
        logEvent("ERROR: Cannot release - servo not initialized");
        return;
    }
    
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        logEvent("RELEASING AEROBRAKE!");
        parachuteServo.write(SERVO_RELEASE_ANGLE);
        xSemaphoreGive(servoMutex);
    } else {
        logEvent("ERROR: Could not acquire servo mutex");
    }
}

/**
 * Test servo movement (for ground testing)
 */
void testServo() {
    if (!servo_ok) {
        logEvent("ERROR: Servo not initialized");
        return;
    }
    
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        logEvent("Testing servo sequence...");
        
        parachuteServo.write(SERVO_NEUTRAL_ANGLE);
        delay(500);
        
        parachuteServo.write(SERVO_DEPLOY_ANGLE);
        delay(1000);
        
        parachuteServo.write(SERVO_NEUTRAL_ANGLE);
        delay(500);
        
        parachuteServo.write(SERVO_RELEASE_ANGLE);
        delay(1000);
        
        parachuteServo.write(SERVO_NEUTRAL_ANGLE);
        
        logEvent("Servo test complete");
        xSemaphoreGive(servoMutex);
    }
}

// ============================================
// Gas Sensor Implementation (merged from GasSensors.h)
// ============================================

/**
 * Initialize gas sensors
 */
void initializeGasSensors() {
    logEvent("=== Gas Sensor Initialization ===");
    
    // Set analog pins as input (default, but explicit)
    pinMode(MQ135_PIN, INPUT);
    pinMode(MQ5_PIN, INPUT);
    
    mq135_ok = true;
    mq5_ok = true;
    
    gasSensorData.warmup_start_time = millis();
    gasSensorData.sensors_ready = false;
    
    logEvent("✓ MQ135 (CO2) on pin " + String(MQ135_PIN));
    logEvent("✓ MQ5 (LPG) on pin " + String(MQ5_PIN));
    logEvent("Gas sensors warming up (20 seconds)...");
}

/**
 * Calculate CO2 PPM from MQ135 sensor
 */
float calculateCO2PPM(int analogValue) {
    if (analogValue <= 0) return 0.0f;
    
    // Convert analog reading to resistance
    float voltage = (analogValue * 3.3f) / 4095.0f;  // ESP32 ADC
    if (voltage <= 0.1f) return 0.0f;  // Avoid division by zero
    
    float rs = ((3.3f - voltage) / voltage) * MQ135_RL;
    
    // Calculate ratio Rs/R0
    float ratio = rs / MQ135_RZERO;
    
    // Convert to PPM using the calibrated formula
    float ppm = MQ135_PARA * pow(ratio, -MQ135_PARB);
    
    // Reasonable bounds for CO2 (outdoor: ~400ppm, indoor: 400-1000ppm)
    return constrain(ppm, 350.0f, 5000.0f);
}

/**
 * Calculate air quality index from CO2 levels
 */
float calculateAirQuality(float co2_ppm) {
    // Air quality scale based on CO2 levels
    if (co2_ppm < 400) return 100.0f;      // Excellent
    else if (co2_ppm < 600) return 90.0f;  // Good
    else if (co2_ppm < 800) return 70.0f;  // Fair
    else if (co2_ppm < 1000) return 50.0f; // Moderate
    else if (co2_ppm < 1500) return 30.0f; // Poor
    else return 10.0f;                      // Very Poor
}

/**
 * Process MQ5 gas sensor reading
 */
void processMQ5Reading(int analogValue) {
    gasSensorData.lpg_level = analogValue;
    
    // Detect gas presence
    if (analogValue > MQ5_GAS_THRESHOLD) {
        if (!gasSensorData.gas_detected) {
            logEvent("WARNING: Gas detected! Level: " + String(analogValue));
        }
        gasSensorData.gas_detected = true;
    } else {
        gasSensorData.gas_detected = false;
    }
}

/**
 * Read all gas sensors
 */
void readGasSensors() {
    // Check if warmup period is complete (20 seconds)
    if (!gasSensorData.sensors_ready) {
        if (millis() - gasSensorData.warmup_start_time >= 20000) {
            gasSensorData.sensors_ready = true;
            logEvent("Gas sensors warmup complete - ready for readings");
        } else {
            // Still warming up
            return;
        }
    }
    
    // Read MQ135 (CO2/Air Quality)
    if (mq135_ok) {
        int mq135_raw = analogRead(MQ135_PIN);
        gasSensorData.co2_ppm = calculateCO2PPM(mq135_raw);
        gasSensorData.air_quality = calculateAirQuality(gasSensorData.co2_ppm);
    }
    
    // Read MQ5 (LPG/Gas Detection)
    if (mq5_ok) {
        int mq5_raw = analogRead(MQ5_PIN);
        processMQ5Reading(mq5_raw);
    }
}

/**
 * Get gas sensor status string for telemetry
 */
String getGasSensorStatus() {
    if (!gasSensorData.sensors_ready) {
        return "WARMUP:" + String((20000 - (millis() - gasSensorData.warmup_start_time)) / 1000);
    }
    
    String status = "CO2:" + String(gasSensorData.co2_ppm, 0) + "ppm;";
    status += "AQ:" + String(gasSensorData.air_quality, 0) + "%;";
    status += "LPG:" + String(gasSensorData.lpg_level);
    if (gasSensorData.gas_detected) status += ";GAS_ALERT";
    
    return status;
}

/**
 * Check for gas safety alerts
 */
void checkGasSafetyAlerts() {
    // High CO2 warning (above 1500 ppm is dangerous)
    if (gasSensorData.co2_ppm > 1500.0f) {
        logEvent("ALERT: High CO2 detected: " + String(gasSensorData.co2_ppm, 0) + " ppm");
    }
    
    // Gas leak detection
    if (gasSensorData.gas_detected) {
        logEvent("ALERT: Combustible gas detected! Level: " + String(gasSensorData.lpg_level));
    }
    
    // Very poor air quality
    if (gasSensorData.air_quality < 20.0f) {
        logEvent("WARNING: Poor air quality: " + String(gasSensorData.air_quality, 0) + "%");
    }
}

#endif
