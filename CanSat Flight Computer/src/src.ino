#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LoRa.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_BME680.h>
#include "ICM_20948.h"
#include <TinyGPSPlus.h>
#include <ESP32Servo.h>
#include <Preferences.h>

#include "config.h"
#include "Telemetry.h"      // Must come before BlackBox and sensors
#include "KalmanFilter.h"
#include "DataBuffer.h"
#include "BlackBox.h"
#include "SensorFusion.h"
#include "sensors.h"
#include "OLEDDisplay.h"

// --- Global Object Instances ---
Adafruit_LPS22 lps;
Adafruit_BME680 bme;
ICM_20948_I2C myICM;
TinyGPSPlus gps;
Servo parachuteServo;
KalmanFilter altitudeKF;
SensorFusion sensorFusion;
Preferences preferences;
BlackBox blackBox;

#ifdef ARDUINO
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

// --- Global Variables ---
TelemetryData telemetryData;
SensorCalibration sensorCal;
DataBuffer telemetryBuffer;
DataBuffer eventBuffer;
GasSensorData gasSensorData;
DisplayData displayData;

// Status flags
bool lps_ok = false;
bool bme_ok = false;
bool icm_ok = false;
bool gps_ok = false;
bool servo_ok = false;
bool lora_ok = false;
bool oled_initialized = false;
bool mq135_ok = false;
bool mq5_ok = false;
bool sdCardOK = false;

// Flight state
volatile FlightState currentFlightState = BOOT;
bool isInTestMode = false;
float maxAltitude = 0.0f;
float groundAltitude = 0.0f;
bool aerobrakeDeployed = false;

// Timing
unsigned long lastTelemetryTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastSDFlushTime = 0;
unsigned long lastStateCheckTime = 0;

// Thread synchronization
SemaphoreHandle_t spiMutex = NULL;
SemaphoreHandle_t servoMutex = NULL;
SemaphoreHandle_t sdMutex = NULL;

// File handles
File dataFile;
File eventFile;
String dataFileName;
String eventFileName;

// --- Function Prototypes ---
void setupSDCard();
void logToSD(const String& message, bool isEvent = false);
void flushSDBuffers();
void stateMachine();
void telemetryTask(void* parameter);
void sensorTask(void* parameter);
void commandTask(void* parameter);

// New initialization prototypes
bool initializeSD();
void initializeSensors();
bool initializeLoRa();

// --- Logging Function ---
void logEvent(const String& message) {
    Serial.println("[" + String(millis()) + "] " + message);
    eventBuffer.add(String(millis()) + "," + message);
}

// --- SD Card Setup ---
void setupSDCard() {
    if (!SD.begin(SD_CS_PIN)) {
        logEvent("✗ SD Card FAILED");
        return;
    }
    
    sdCardOK = true;
    logEvent("✓ SD Card OK");
    
    // Create unique filenames with timestamp
    int fileIndex = 0;
    do {
        dataFileName = "/Flight_" + String(TEAM_ID) + "_" + String(fileIndex) + ".csv";
        eventFileName = "/Events_" + String(TEAM_ID) + "_" + String(fileIndex) + ".csv";
        fileIndex++;
    } while (SD.exists(dataFileName));
    
    // Create data file with header
    dataFile = SD.open(dataFileName, FILE_WRITE);
    if (dataFile) {
        dataFile.println(getTelemetryHeader());
        dataFile.close();
        logEvent("Created: " + dataFileName);
    }
    
    // Create event file
    eventFile = SD.open(eventFileName, FILE_WRITE);
    if (eventFile) {
        eventFile.println("TIMESTAMP,EVENT");
        eventFile.close();
        logEvent("Created: " + eventFileName);
    }
}

// --- SD Logging ---
void logToSD(const String& message, bool isEvent) {
    if (!sdCardOK) return;
    
    if (isEvent) {
        eventBuffer.add(message);
    } else {
        telemetryBuffer.add(message);
    }
}

// --- Flush SD Buffers ---
void flushSDBuffers() {
    if (!sdCardOK) return;
    
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
    
    // Flush telemetry data
    if (!telemetryBuffer.isEmpty()) {
        dataFile = SD.open(dataFileName, FILE_APPEND);
        if (dataFile) {
            while (!telemetryBuffer.isEmpty()) {
                dataFile.println(telemetryBuffer.get());
            }
            dataFile.close();
        }
    }
    
    // Flush event data
    if (!eventBuffer.isEmpty()) {
        eventFile = SD.open(eventFileName, FILE_APPEND);
        if (eventFile) {
            while (!eventBuffer.isEmpty()) {
                eventFile.println(eventBuffer.get());
            }
            eventFile.close();
        }
    }
    
    xSemaphoreGive(sdMutex);
}

// --- State Transition ---
void transitionToState(FlightState newState) {
    if (currentFlightState == newState) return;
    
    FlightState oldState = currentFlightState;
    currentFlightState = newState;
    telemetryData.flightState = newState;
    
    logEvent("STATE: " + String(getStateName(oldState)) + " -> " + 
             String(getStateName(newState)));
    
    blackBox.logCriticalEvent(EVENT_STATE_CHANGE, oldState, newState);
    blackBox.saveState();
}

// --- State Machine Logic ---
void stateMachine() {
    static unsigned long stateEntryTime = 0;
    unsigned long timeInState = millis() - stateEntryTime;
    
    // Update state entry time on state change
    static FlightState lastState = BOOT;
    if (currentFlightState != lastState) {
        stateEntryTime = millis();
        timeInState = 0;
        lastState = currentFlightState;
    }
    
    switch (currentFlightState) {
        case BOOT:
            // Wait for all systems to initialize
            if (lps_ok && icm_ok && lora_ok && servo_ok) {
                transitionToState(LAUNCH_PAD);
            }
            break;
            
        case TEST_MODE:
            // Manual testing mode - no automatic transitions
            // Controlled via ground commands
            break;
            
        case LAUNCH_PAD:
            if (isInTestMode) break;
            {
                // Detect launch by acceleration
                float totalAccel = sqrt(telemetryData.accelX * telemetryData.accelX +
                                      telemetryData.accelY * telemetryData.accelY +
                                      telemetryData.accelZ * telemetryData.accelZ);
                
                if (totalAccel > LAUNCH_ACCEL_THRESHOLD && timeInState > 2000) {
                    logEvent("LAUNCH DETECTED! Accel=" + String(totalAccel, 2) + "g");
                    blackBox.logCriticalEvent(EVENT_STATE_CHANGE, totalAccel, telemetryData.altitude);
                    transitionToState(ASCENT);
                }
            }
            break;
            
        case ASCENT:
            // Track maximum altitude
            if (telemetryData.altitude > maxAltitude) {
                maxAltitude = telemetryData.altitude;
            }
            
            // Detect apogee by vertical velocity
            if (telemetryData.verticalSpeed < APOGEE_DESCENT_THRESHOLD && 
                timeInState > 3000) {
                logEvent("APOGEE! MaxAlt=" + String(maxAltitude, 1) + "m");
                blackBox.logCriticalEvent(EVENT_APOGEE, maxAltitude, telemetryData.verticalSpeed);
                transitionToState(ROCKET_DEPLOY);
            }
            break;
            
        case ROCKET_DEPLOY:
            // Deploy parachute immediately at apogee
            if (!aerobrakeDeployed) {
                deployParachute();
                aerobrakeDeployed = true;
                blackBox.logCriticalEvent(EVENT_DEPLOY, telemetryData.altitude, telemetryData.verticalSpeed);
                blackBox.saveState();
            }
            
            // Transition to descent after deployment
            if (timeInState > 2000) {
                transitionToState(DESCENT);
            }
            break;
            
        case DESCENT:
            // Release aerobrake at low altitude
            if (telemetryData.altitude < AEROBRAKE_RELEASE_ALTITUDE && 
                timeInState > 5000) {
                transitionToState(AEROBRAKE_RELEASE);
            }
            
            // Detect impact
            if (telemetryData.altitude < IMPACT_ALTITUDE_THRESHOLD &&
                abs(telemetryData.verticalSpeed) < IMPACT_VELOCITY_THRESHOLD &&
                timeInState > 10000) {
                transitionToState(IMPACT);
            }
            break;
            
        case AEROBRAKE_RELEASE:
            // Release aerobrake
            if (timeInState < 100) {
                releaseAerobrake();
            }
            
            // Continue to impact detection
            if (telemetryData.altitude < IMPACT_ALTITUDE_THRESHOLD &&
                abs(telemetryData.verticalSpeed) < IMPACT_VELOCITY_THRESHOLD) {
                transitionToState(IMPACT);
            }
            break;
            
        case IMPACT:
            // Final state - continue logging for recovery
            // Reduce telemetry rate to conserve power
            if (timeInState % 10000 == 0) {
                logEvent("POST-IMPACT: Alt=" + String(telemetryData.altitude, 1) + "m, " +
                        "GPS=" + String(telemetryData.gnssLatitude, 4) + "," +
                        String(telemetryData.gnssLongitude, 4));
            }
            break;
    }
}

// --- RTOS Tasks ---

void sensorTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SENSOR_READ_INTERVAL);
    
    while (true) {
        readAllSensors();
        readGasSensors();
        checkGasSafetyAlerts();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void telemetryTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_INTERVAL);
    
    while (true) {
        // Update telemetry state
        telemetryData.flightState = currentFlightState;
        
        // Send via LoRa
        sendLoRaTelemetry();
        
        // Log to SD card
        String csvData = formatTelemetryCSV();
        logToSD(csvData, false);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void commandTask(void* parameter) {
    while (true) {
        receiveLoRaCommands();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void stateTask(void* parameter) {
    while (true) {
        stateMachine();
        
        // Periodic state save
        if (millis() - lastStateCheckTime > 5000) {
            blackBox.saveState();
            lastStateCheckTime = millis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void sdTask(void* parameter) {
    while (true) {
        flushSDBuffers();
        vTaskDelay(pdMS_TO_TICKS(SD_FLUSH_INTERVAL));
    }
}

void displayTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500);  // Update display every 500ms
    
    while (true) {
        updateDisplay();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n==============================");
    Serial.println("   CANSAT FLIGHT COMPUTER");
    Serial.println("   Team: " + String(TEAM_ID));
    Serial.println("==============================\n");
    
    // Create mutexes
    spiMutex = xSemaphoreCreateMutex();
    servoMutex = xSemaphoreCreateMutex();
    sdMutex = xSemaphoreCreateMutex();
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);
    
    // Initialize SPI
    SPI.begin();
    
    // Initialize SD card
    initializeSD();

    // Initialize preferences for blackbox
    preferences.begin("cansat", false);

    // Try to recover previous state
    FlightState recoveredState = BOOT;
    currentFlightState = blackBox.recoverFlightState(currentFlightState, recoveredState);
    logEvent("Recovered state: " + String(getStateName(currentFlightState)));
    
    // Initialize sensors
    initializeSensors();
    
    // Initialize OLED display
    if (initializeOLED()) {
        Serial.println("✅ OLED initialized");
    }
    
    // Initialize LoRa
    initializeLoRa();
    
    // Create RTOS tasks on dual cores
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(telemetryTask, "TelemetryTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(commandTask, "CommandTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(stateTask, "StateTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(sdTask, "SDTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, NULL, 1);
    
    Serial.println("✅ Setup complete - RTOS tasks started");
}

// --- Loop (unused with RTOS) ---
void loop() {
    // Watchdog monitoring
    vTaskDelay(pdMS_TO_TICKS(1000));
}

bool initializeSD() {
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("⚠️  SD Card initialization failed");
        return false;
    } else {
        Serial.println("✅ SD Card initialized");
        sdCardOK = true;
        return true;
    }
}


bool initializeLoRa() {
    LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (LoRa.begin(433E6)) {  // 433 MHz
        LoRa.setSpreadingFactor(7);
        LoRa.setSignalBandwidth(125E3);
        LoRa.setCodingRate4(5);
        LoRa.enableCrc();
        lora_ok = true;
        Serial.println("✅ LoRa initialized");
        return true;
    } else {
        lora_ok = false;
        Serial.println("⚠️  LoRa initialization failed");
        return false;
    }
}
