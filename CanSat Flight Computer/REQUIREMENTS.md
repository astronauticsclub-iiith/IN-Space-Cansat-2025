# CanSat Requirements Specification

## Overview
This document outlines the complete requirements for the CanSat flight software system, covering all functional, performance, safety, and interface requirements.

## Flight State Machine Requirements

### REQ-FSM-001: Flight States Definition
**Description:** The system shall implement exactly 8 flight states
**States:**
- BOOT = 0 (System initialization)
- TEST_MODE = 1 (Ground testing mode)  
- LAUNCH_PAD = 2 (Ready for launch)
- ASCENT = 3 (Rocket powered flight)
- ROCKET_DEPLOY = 4 (CanSat deployment from rocket)
- DESCENT = 5 (Parachute descent)
- AEROBRAKE_RELEASE = 6 (Aerobrake deployment)
- IMPACT = 7 (Ground contact and recovery)

## Telemetry Requirements

### REQ-TM-001: Transmission Rate
**Description:** Telemetry shall be transmitted at 1 Hz (once per second)
**Verification:** Check TELEMETRY_INTERVAL = 1000ms

### REQ-TM-002: CSV Format
**Description:** All telemetry shall use competition-specified CSV format
**Format:** TEAM_ID,TIME,PACKET_COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,GNSS_TIME,GNSS_LAT,GNSS_LON,GNSS_ALT,GNSS_SATS,ACCEL_DATA,GYRO_DATA,STATE,OPTIONAL_DATA

### REQ-TM-003: Required Fields
**Description:** All 16 telemetry fields must be present in every packet
1. TEAM_ID - Team identifier string
2. TIME_STAMPING - Mission elapsed time
3. PACKET_COUNT - Sequential packet number  
4. ALTITUDE - Pressure altitude (m)
5. PRESSURE - Atmospheric pressure (Pa)
6. TEMP - Temperature (°C)
7. VOLTAGE - Battery voltage (V)
8. GNSS_TIME - GPS time string
9. GNSS_LATITUDE - GPS latitude (degrees)
10. GNSS_LONGITUDE - GPS longitude (degrees) 
11. GNSS_ALTITUDE - GPS altitude (m)
12. GNSS_SATS - Number of GPS satellites
13. ACCELEROMETER_DATA - 3-axis acceleration
14. GYRO_SPIN_RATE - 3-axis gyro data
15. FLIGHT_SOFTWARE_STATE - Current flight state
16. OPTIONAL_DATA - Additional sensor data

### REQ-TM-008 to REQ-TM-013: Data Resolution
- **REQ-TM-008:** Altitude resolution: 0.1 meters
- **REQ-TM-009:** Pressure resolution: 1 Pa  
- **REQ-TM-010:** Temperature resolution: 0.1°C
- **REQ-TM-011:** Voltage resolution: 0.01V
- **REQ-TM-012:** GPS coordinate resolution: 0.0001 degrees
- **REQ-TM-013:** Time resolution: 0.1 seconds

## Sensor Requirements

### REQ-SENS-001: Pressure Sensors
**Description:** System shall use dual pressure sensors for redundancy
**Sensors:** LPS22HB primary, BME680 secondary

### REQ-SENS-002: IMU Requirements  
**Description:** 9-DOF IMU for attitude and acceleration
**Sensor:** ICM-20948 (3-axis accel, gyro, magnetometer)

### REQ-SENS-003: GPS Requirements
**Description:** GPS receiver for position and time
**Requirements:** Minimum 4 satellites for valid fix

### REQ-SENS-004: Environmental Sensors
**Description:** Temperature, humidity, gas resistance monitoring
**Sensor:** BME680 for environmental data

### REQ-SENS-005: Sensor Fusion
**Description:** Multiple sensors fused for improved accuracy
**Implementation:** Kalman filtering and sensor health monitoring

### REQ-SENS-006: Sensor Health Monitoring
**Description:** Continuous monitoring of sensor status
**Actions:** Automatic fallback to backup sensors

## Flight Detection Requirements

### REQ-LAUNCH-001: Launch Detection
**Description:** Detect rocket launch via acceleration threshold
**Threshold:** 2.5g sustained acceleration
**Duration:** Minimum 100ms above threshold

### REQ-DEPLOY-002: Apogee Detection  
**Description:** Detect CanSat deployment from rocket
**Method:** Vertical velocity < -5 m/s (descent initiated)
**Backup:** Altitude decrease over 3 consecutive readings

### REQ-LAND-001: Impact Detection - Altitude
**Description:** Detect ground impact via altitude threshold
**Threshold:** Below 5 meters AGL (Above Ground Level)

### REQ-LAND-002: Impact Detection - Velocity
**Description:** Confirm landing via low vertical velocity
**Threshold:** Vertical velocity < 2 m/s for 5 seconds

## Command Interface Requirements

### REQ-CMD-001 to REQ-CMD-010: Ground Commands
**Description:** System shall respond to ground station commands
**Commands:**
1. **CMD,CAL_ALT** - Calibrate ground altitude reference
2. **CMD,CAL_IMU** - Calibrate IMU sensors
3. **CMD,RESET_STATE** - Reset flight state to LAUNCH_PAD
4. **CMD,STATUS** - Report current system status
5. **CMD,SET_MODE,TEST** - Enter test mode
6. **CMD,SET_MODE,FLIGHT** - Enter flight mode  
7. **CMD,TEST_SERVO** - Test servo operation
8. **CMD,DEPLOY_TEST** - Test deployment mechanism
9. **CMD,START_TX** - Enable telemetry transmission
10. **CMD,STOP_TX** - Disable telemetry transmission

## Performance Requirements

### REQ-PERF-TIME-001: Telemetry Timing
**Description:** Telemetry transmitted every 1000ms ±50ms
**Jitter:** Maximum 5% timing deviation

### REQ-PERF-TIME-002: Sensor Reading Rate  
**Description:** Sensors read at 10 Hz (100ms intervals)
**Critical:** No missed sensor readings during flight

### REQ-PERF-MEM-001: Memory Usage
**Description:** Total firmware size < 80% of available flash
**Flash:** ESP32 4MB flash, firmware < 3.2MB
**RAM:** Runtime RAM usage < 70% of available

### REQ-PERF-POWER-001: Power Management
**Description:** System operational for minimum 3 hours
**Battery:** Monitor and report battery voltage
**Sleep:** Low-power modes when appropriate

## Safety Requirements

### REQ-SAFE-001: Dual Sensors
**Description:** Critical measurements use redundant sensors
**Implementation:** Primary/backup sensor pairs

### REQ-SAFE-002: Watchdog Timer
**Description:** Hardware watchdog prevents system lockup
**Timeout:** 5-second watchdog timer

### REQ-SAFE-003: Mutex Protection
**Description:** Shared resources protected by mutexes
**Resources:** SPI bus, I2C bus, SD card, servo control

### REQ-SAFE-004: Error Recovery
**Description:** System recovers from sensor failures
**Actions:** Automatic sensor switching, error logging

### REQ-SAFE-005: Deployment Safety
**Description:** Parachute deployment only in valid states
**States:** Only deploy in DESCENT or AEROBRAKE_RELEASE

## Recovery Requirements

### REQ-RECOVERY-001: BlackBox Logging
**Description:** All critical events logged to non-volatile storage
**Storage:** SD card with backup to EEPROM
**Data:** Flight state changes, sensor failures, commands

### REQ-RECOVERY-002: State Persistence
**Description:** Flight state preserved across power cycles
**Implementation:** State saved to EEPROM every transition
**Recovery:** Automatic state recovery on boot

### REQ-RECOVERY-003: Data Integrity
**Description:** Logged data protected against corruption
**Methods:** Checksums, magic numbers, redundant storage

## Architecture Requirements

### REQ-ARCH-MOD: Modular Design
**Description:** Software organized in functional modules
**Modules:** Config, Sensors, Telemetry, BlackBox, KalmanFilter, SensorFusion, DataBuffer

### REQ-ARCH-TASK: RTOS Task Structure
**Description:** FreeRTOS tasks for concurrent operation
**Tasks:**
- sensorTask (Core 0) - Sensor data collection
- telemetryTask (Core 1) - Data transmission  
- commandTask (Core 1) - Command processing
- stateTask (Core 0) - State machine management
- sdTask (Core 0) - Data logging

### REQ-ARCH-CORE: Dual Core Usage
**Description:** Utilize both ESP32 cores efficiently  
**Core 0:** Time-critical sensor and control tasks
**Core 1:** Communication and data processing

## Hardware Interface Requirements

### REQ-HW-001 to REQ-HW-005: Pin Assignments
**Description:** Standardized pin assignments for hardware interfaces
**Pins:**
- LORA_SS_PIN = 5 (LoRa SPI select)
- GPS_RX_PIN = 16, GPS_TX_PIN = 17 (GPS UART)
- SERVO_PIN = 13 (Parachute servo control)
- VOLTAGE_PIN = 34 (Battery voltage ADC)
- SD_CS_PIN = 15 (SD card SPI select)

### REQ-HW-I2C: I2C Bus Configuration
**Description:** I2C sensors on shared bus with proper addressing
**Addresses:** LPS22=0x5D, BME680=0x76, ICM=0x69, OLED=0x3C

### REQ-HW-SPI: SPI Bus Configuration  
**Description:** SPI devices with proper chip select management
**Devices:** LoRa radio, SD card (separate CS pins)

## Verification Methods
- **Static Analysis:** Code inspection and automated checks
- **Unit Testing:** Individual module testing with mocks
- **Integration Testing:** Module interaction validation  
- **Hardware-in-Loop:** Testing with actual sensors
- **Flight Simulation:** Complete mission scenario testing
- **Ground Testing:** Pre-flight validation procedures

## Compliance Matrix
All requirements are traceable to:
- Competition rules and regulations
- Safety standards and best practices  
- Hardware capabilities and constraints
- Mission success criteria

---
**Document Version:** 1.0
**Last Updated:** October 2025
**Approved By:** Team Brahmand