#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <SPI.h>
#include <SD.h>

// Define the Chip Select pin for the SD card module
#define SD_CS 5

// Create a file object
File dataFile;

// Global variable to hold the unique filename for the current session
String currentFilename;

// Create an Adafruit BNO08x sensor object
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// Variables to hold the latest sensor data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
unsigned long timestamp_ms;

// Flags to track if we have received new data from each sensor
bool hasNewAccel = false;
bool hasNewGyro = false;

// Function to find the next available filename
String getNextFilename() {
  String filename = "";
  int fileNum = 0;
  // Loop to find a unique filename
  while (true) {
    fileNum++;
    // Create a padded filename like datalog01.csv, datalog02.csv
    char temp[16];
    sprintf(temp, "/datalog%02d.csv", fileNum);
    filename = String(temp);
    
    if (!SD.exists(filename)) {
      break;
    }
  }
  return filename;
}

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("BNO08x & SD Card Data Logger - Synchronized Output");
  Serial.println("");

  //---- SENSOR INITIALIZATION ----//
  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("Failed to find BNO08x chip! Check wiring or I2C address.");
    while (1) delay(10);
  }
  Serial.println("BNO08x Found!");

  // --- Enable the reports you want to receive ---
  Serial.println("Setting up reports...");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  Serial.println("Reports enabled.");
  delay(500);

  //---- SD CARD INITIALIZATION ----//
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("initialization failed!");
    while (1) delay(10);
  }
  Serial.println("initialization done.");

  // Get a unique filename and store it in the global variable
  currentFilename = getNextFilename();

  // Open the new file and write the header
  dataFile = SD.open(currentFilename, FILE_WRITE);
  if (dataFile) {
    Serial.print("Writing CSV header to new file: ");
    Serial.println(currentFilename);
    dataFile.println("Timestamp (ms),AccelX (m/s^2),AccelY (m/s^2),AccelZ (m/s^2),GyroX (rad/s),GyroY (rad/s),GyroZ (rad/s)");
    dataFile.close();
  } else {
    Serial.println("Error creating new datalog file.");
  }

  Serial.println("\nSetup complete. Logging data to SD Card and Serial Monitor...");
  Serial.println("Timestamp (ms),AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
}

void loop() {
  // Check for new sensor data
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        accelX = sensorValue.un.accelerometer.x;
        accelY = sensorValue.un.accelerometer.y;
        accelZ = sensorValue.un.accelerometer.z;
        timestamp_ms = sensorValue.timestamp / 1000;
        hasNewAccel = true;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        gyroX = sensorValue.un.gyroscope.x;
        gyroY = sensorValue.un.gyroscope.y;
        gyroZ = sensorValue.un.gyroscope.z;
        hasNewGyro = true;
        break;
    }
  }

  // If we have a fresh reading from BOTH sensors, log the combined data
  if (hasNewAccel && hasNewGyro) {
    // 1. Combine data into a single CSV formatted string
    String dataString = String(timestamp_ms) + "," + String(accelX) + "," + String(accelY) + "," + String(accelZ) + "," + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ);

    // 2. Print the complete line to the Serial Monitor for real-time debugging
    Serial.println(dataString);

    // 3. Open the file using the saved filename and append the data
    dataFile = SD.open(currentFilename, FILE_APPEND);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    } else {
      Serial.println("Error writing to datalog file.");
    }

    // 4. Reset flags to wait for the next complete set of data
    hasNewAccel = false;
    hasNewGyro = false;
  }
}
