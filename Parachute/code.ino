//TESTED 

/**************************************************************************
  CanSat Backup Parachute Deployment Code (Quad-Redundant)
  For Seeeduino XIAO SAMD21

  Deploys under one of four conditions:
  1. Altitude Trigger: Rises >500m, then falls <450m.
  2. Apogee Timer: 1 minute has passed since apogee was detected.
  3. Master Failsafe: 10 minutes have passed since power-on.
  4. Emergency Low-Altitude: Descending for >10 readings below 100m.
***************************************************************************/

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// --- Pin Definitions ---
#define SERVO_PIN A1

// --- Servo Configuration ---
const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_OPEN_ANGLE = 90;

// --- Deployment Logic Parameters ---
const float MIN_APOGEE_ALTITUDE = 500.0;     // meters: Min height to arm the system
const float DEPLOYMENT_ALTITUDE = 450.0;     // meters: Deploy when falling below this height
const int DESCENT_READINGS_REQUIRED = 5;       // Debounce counter for altitude trigger
const unsigned long APOGEE_TIMER_MS = 1 * 60 * 1000UL; // 1 minute post-apogee timer
const unsigned long MASTER_TIMER_MS = 10 * 60 * 1000UL; // 10 mins mission timer
const float EMERGENCY_LOW_ALTITUDE = 100.0;    // meters: Emergency deployment threshold
const int EMERGENCY_DESCENT_READINGS = 10;     // Consecutive descent readings for emergency trigger

// --- Global Objects & State Variables ---
Servo parachuteServo;
Adafruit_BMP280 bmp;

float groundLevelPressure;
float maxAltitude = 0.0;
float lastAltitude = 0.0; // Used to detect the moment of apogee

bool isArmed = false;            // True once MIN_APOGEE_ALTITUDE is passed
bool apogeeTimeRecorded = false; // True once the apogee timer has started
bool parachuteDeployed = false;  // Prevents multiple deployments

unsigned long startTime = 0;       // Timestamp for master timer
unsigned long apogeeTime = 0;      // Timestamp for post-apogee timer
int descentReadingCounter = 0; // Debounce counter
int emergencyDescentCounter = 0; // Counter for emergency low-altitude trigger

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CanSat Quad-Redundant Parachute System Initializing...");

  parachuteServo.attach(SERVO_PIN);
  
  // Servo self-test: sweep full range
  Serial.println("Performing servo self-test...");
  parachuteServo.write(SERVO_CLOSED_ANGLE);
  delay(500);
  for (int angle = SERVO_CLOSED_ANGLE; angle <= SERVO_OPEN_ANGLE; angle++) {
    parachuteServo.write(angle);
    delay(10);
  }
  delay(500);
  for (int angle = SERVO_OPEN_ANGLE; angle >= SERVO_CLOSED_ANGLE; angle--) {
    parachuteServo.write(angle);
    delay(10);
  }
  parachuteServo.write(SERVO_CLOSED_ANGLE);
  Serial.println("Servo test complete. Servo locked.");
  delay(500);

  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  Serial.println("BMP280 Sensor found.");

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  Serial.println("Calibrating ground level pressure...");
  delay(1000);
  groundLevelPressure = bmp.readPressure() / 100.0F;
  Serial.print("Ground Level Pressure: "); Serial.print(groundLevelPressure); Serial.println(" hPa");

  startTime = millis();
  Serial.println("Initialization complete. System armed and waiting for launch.");
}

// =========================================================================
// LOOP
// =========================================================================
void loop() {
  if (parachuteDeployed) {
    return; // Stop all logic after deployment
  }

  float currentAltitude = bmp.readAltitude(groundLevelPressure);

  // --- State Detection Logic ---
  if (currentAltitude > maxAltitude) {
    maxAltitude = currentAltitude;
  }

  // 1. Arm the system once it passes the minimum apogee altitude
  if (!isArmed && maxAltitude > MIN_APOGEE_ALTITUDE) {
    isArmed = true;
    Serial.println("!!! System Armed: Minimum apogee altitude reached. !!!");
  }

  // 2. Detect the moment of apogee to start the 1-minute timer
  // This happens ONCE when the system is armed and altitude first starts to decrease.
  if (isArmed && !apogeeTimeRecorded && (currentAltitude < maxAltitude - 2.0)) { // -2.0m buffer for noise
    apogeeTime = millis();
    apogeeTimeRecorded = true;
    Serial.println("!!! APOGEE DETECTED - Starting 1-minute backup timer. !!!");
  }

  // --- Deployment Trigger Checks ---

  // TRIGGER 1: Post-Apogee Timer (1 minute)
  if (apogeeTimeRecorded && (millis() - apogeeTime > APOGEE_TIMER_MS)) {
    deployParachute("Apogee Timer Trigger (1 min)");
  }

  // TRIGGER 2: Altitude-based Descent Trigger
  if (isArmed) {
    if (currentAltitude < DEPLOYMENT_ALTITUDE) {
      descentReadingCounter++; // Increment the counter
    } else {
      descentReadingCounter = 0; // Reset if we go back up
    }
    if (descentReadingCounter >= DESCENT_READINGS_REQUIRED) {
      deployParachute("Altitude Trigger (Confirmed Descent)");
    }
  }

  // TRIGGER 3: Master Failsafe Timer (10 minutes)
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;
  if (elapsedTime > MASTER_TIMER_MS) {
    Serial.print("DEBUG: currentTime="); Serial.print(currentTime);
    Serial.print(", startTime="); Serial.print(startTime);
    Serial.print(", elapsed="); Serial.print(elapsedTime);
    Serial.print(", threshold="); Serial.println(MASTER_TIMER_MS);
    deployParachute("Master Failsafe Timer");
  }

  // TRIGGER 4: Emergency Low-Altitude Descent
  if (currentAltitude < EMERGENCY_LOW_ALTITUDE && currentAltitude < lastAltitude) {
    emergencyDescentCounter++;
  } else if (currentAltitude >= lastAltitude) {
    emergencyDescentCounter = 0; // Reset if ascending
  }
  if (emergencyDescentCounter >= EMERGENCY_DESCENT_READINGS) {
    deployParachute("Emergency Low-Altitude Descent (<100m, 10+ descents)");
  }

  lastAltitude = currentAltitude;

  // --- Serial Output for Debugging ---
  Serial.print("Alt: "); Serial.print(currentAltitude, 1);
  Serial.print("m, Max: "); Serial.print(maxAltitude, 1);
  Serial.print("m, Armed: "); Serial.print(isArmed);
  if (apogeeTimeRecorded) {
    Serial.print(", Apogee Timer: "); Serial.print((millis() - apogeeTime) / 1000); Serial.print("s");
  }
  Serial.print(", Master Time: "); Serial.print((millis() - startTime) / 1000); Serial.println("s");

  delay(200);
}

// =========================================================================
// DEPLOY_PARACHUTE
// =========================================================================
void deployParachute(String reason) {
  if (parachuteDeployed) {
    return;
  }
  parachuteDeployed = true;

  Serial.println("\n**************************************");
  Serial.print("DEPLOYING PARACHUTE! Reason: ");
  Serial.println(reason);
  Serial.println("**************************************");

  parachuteServo.write(SERVO_OPEN_ANGLE);
}
