# Backup-Parachute-System

A quad-redundant parachute deployment system for CanSat missions, designed to run on the Seeeduino XIAO (SAMD21).

## Overview

This system provides four independent deployment triggers to ensure reliable parachute deployment under various failure scenarios. Uses onboard clock and BMP280 pressure sensor for fault-tolerant operation.


## Hardware Requirements

- **Microcontroller:** Seeeduino XIAO SAMD21
- **Sensor:** Adafruit BMP280 (I2C address: 0x76)
- **Actuator:** Standard servo motor (connected to pin A1)
- **Power:** Appropriate power supply for XIAO and servo

## Wiring

| Component | Pin |
|-----------|-----|
| Servo Signal | A1 |
| BMP280 SDA | SDA (I2C) |
| BMP280 SCL | SCL (I2C) 
![IMG_5732](https://github.com/user-attachments/assets/53043b47-2ec4-4073-ab4d-8cd626a9f092)


## Deployment Logic

The system uses a **quad-redundant trigger mechanism**. The first condition met triggers the servo to deploy the parachute.

### Deployment Triggers

1. **Primary: Altitude-Based Descent 🏔️**
   - **Condition:** After passing **500m**, deploys when altitude drops below **450m** for **5 consecutive readings**
   - **Purpose:** Primary deployment mechanism based on altitude thresholds
   - **Debounce:** 5 readings (1 second at 200ms sampling rate)

2. **Secondary: Post-Apogee Timer ⏱️**
   - **Condition:** Deploys **1 minute** after apogee (peak altitude) is detected
   - **Purpose:** Backup if altitude sensor fails during descent
   - **Detection:** Apogee detected when altitude drops 2m below maximum

3. **Tertiary: Master System Timer 🚨**
   - **Condition:** Deploys **30 seconds** after power-on (configurable to 10 minutes for flight)
   - **Purpose:** Ultimate failsafe for catastrophic failures
   - **Note:** Currently set to 30s for testing; change to `10 * 60 * 1000UL` for actual flight

4. **Quaternary: Emergency Low-Altitude Descent 🆘**
   - **Condition:** Descending for **>10 consecutive readings** while below **100m**
   - **Purpose:** Emergency deployment if stuck in tumbling descent at low altitude
   - **Reset:** Counter resets if altitude increases

## Features

- **Servo Self-Test:** On startup, servo sweeps full range (0° → 90° → 0°) to verify mechanical operation
- **Ground-Level Calibration:** Automatically calibrates pressure at startup for accurate altitude readings
- **Serial Debugging:** Real-time altitude, arming status, and timer information via Serial Monitor (9600 baud)
- **Single Deployment:** Prevents multiple deployments once triggered

## Installation

1. Install required libraries in Arduino IDE:
   ```
   - Adafruit BMP280 Library
   - Adafruit Unified Sensor
   - Servo (built-in)
   ```

2. Connect hardware according to wiring diagram

3. Upload code to Seeeduino XIAO

4. Open Serial Monitor (9600 baud) to view system status

## Configuration

### Key Parameters (adjust in code as needed)

```cpp
const float MIN_APOGEE_ALTITUDE = 500.0;        // Arming altitude (meters)
const float DEPLOYMENT_ALTITUDE = 450.0;        // Primary deployment altitude (meters)
const float EMERGENCY_LOW_ALTITUDE = 100.0;     // Emergency trigger altitude (meters)
const int DESCENT_READINGS_REQUIRED = 5;        // Debounce for primary trigger
const int EMERGENCY_DESCENT_READINGS = 10;      // Debounce for emergency trigger
const unsigned long APOGEE_TIMER_MS = 60000;    // Post-apogee timeout (1 minute)
const unsigned long MASTER_TIMER_MS = 30000;    // Master failsafe (30 sec for testing)
```

### Servo Angles

```cpp
const int SERVO_CLOSED_ANGLE = 0;   // Parachute locked
const int SERVO_OPEN_ANGLE = 90;    // Parachute deployed
```

## Testing

### Ground Test Mode
Current configuration deploys after 30 seconds for quick testing. Before flight:

**⚠️ CRITICAL: Change master timer back to 10 minutes:**
```cpp
const unsigned long MASTER_TIMER_MS = 10 * 60 * 1000UL;
```

### Expected Serial Output (Ground)
```
Alt: 0.1m, Max: 0.2m, Armed: 0, Master Time: 5s
Alt: 0.1m, Max: 0.2m, Armed: 0, Master Time: 6s
...
```

### Expected Serial Output (Flight)
```
Alt: 523.4m, Max: 523.4m, Armed: 1, Master Time: 45s
!!! APOGEE DETECTED - Starting 1-minute backup timer. !!!
Alt: 518.2m, Max: 523.4m, Armed: 1, Apogee Timer: 3s, Master Time: 48s
...
**************************************
DEPLOYING PARACHUTE! Reason: Altitude Trigger (Confirmed Descent)
**************************************
```

## Safety Notes

- Always test servo operation before flight
- Verify BMP280 sensor readings are stable
- Ensure backup battery is fully charged
- Test deployment mechanism separately before integration
- **Remember to adjust timer back to 10 minutes for actual flight**

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Servo doesn't move | Check power supply, verify pin A1 connection |
| BMP280 not found | Verify I2C address (0x76 vs 0x77), check wiring |
| Deploys immediately | Check master timer value, verify startTime initialization |
| Altitude readings unstable | Sensor may need settling time, increase filter settings |


