/****************************************************************
 * CanSat Ground Station - LoRa Communication
 *
 * DESCRIPTION:
 * - Receives telemetry from CanSat and forwards to Python parser
 * - Sends commands to CanSat for mode switching
 * - Handles signal quality monitoring and error detection
 *
 * EXPECTED CANSAT TELEMETRY FORMAT:
 * TEAM_ID,TimeStamp,PacketCount,FlightState,Altitude,Pressure,
 * Temperature,Voltage,Latitude,Longitude,AccelX,GyroY,Satellites
 *
 * SUPPORTED COMMANDS:
 * - "test" or "CMD,SET_MODE,TEST" -> Switch to test mode
 * - "flight" or "CMD,SET_MODE,FLIGHT" -> Switch to flight mode
 ****************************************************************/

#include <SPI.h>
#include <LoRa.h>

// --- CONFIGURATION ---
// This device is permanently configured as the Ground Station
#define GROUND_STATION_MODE

// --- PIN DEFINITIONS ---
// DEFINE THE PINS USED BY THE TRANSCEIVER MODULE
#define ss   15  // CORRECTED: Pin 15 to match CanSat's config (LORA_SS_PIN)
#define rst  14
#define dio0 2

void setup() {
  // INITIALIZE SERIAL MONITOR
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  Serial.println("LoRa Duplex Transceiver");
  Serial.println("-------------------------");
  Serial.println("Mode: Ground Station 📡");

  // SETUP LORA TRANSCEIVER MODULE
  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
  
  // Configure LoRa parameters to match CanSat exactly
  LoRa.setSignalBandwidth(125E3);    // 125 kHz bandwidth (default)
  LoRa.setSpreadingFactor(7);        // Spreading factor 7 (default)
  LoRa.setCodingRate4(5);            // Coding rate 4/5 (default)
  LoRa.setPreambleLength(8);         // Preamble length (default)
  LoRa.setSyncWord(0xF3);            // MUST match the CanSat's sync word
  LoRa.enableCrc();                  // Enable CRC checking
  LoRa.setTxPower(20);               // Set transmission power to maximum
  
  Serial.println("LoRa Initializing OK!");
  Serial.println("LoRa Configuration:");
  Serial.println("  Frequency: 433 MHz");
  Serial.println("  Bandwidth: 125 kHz");
  Serial.println("  Spreading Factor: 7");
  Serial.println("  Coding Rate: 4/5");
  Serial.println("  Sync Word: 0xF3");
  Serial.println("  TX Power: 20 dBm");
  Serial.println();
  Serial.println("📡 CanSat Ground Station Ready!");
  Serial.println("Commands:");
  Serial.println("  'test'   -> Switch CanSat to TEST mode");
  Serial.println("  'flight' -> Switch CanSat to FLIGHT mode");
  Serial.println("  Or type full command: CMD,SET_MODE,TEST");
  Serial.println();
  Serial.println("Waiting for telemetry...");
}

void loop() {
  // Listen for an incoming LoRa packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    onReceive(packetSize);
  }

  // Check if the user has typed a command into the Serial Monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      // Handle predefined commands or raw input
      if (input.equalsIgnoreCase("test")) {
        sendMessage("CMD,SET_MODE,TEST");
        Serial.println("[GCS] Sent: Switch to TEST mode");
      } else if (input.equalsIgnoreCase("flight")) {
        sendMessage("CMD,SET_MODE,FLIGHT");
        Serial.println("[GCS] Sent: Switch to FLIGHT mode");
      } else if (input.startsWith("CMD,")) {
        // User typed a full command format
        sendMessage(input);
        Serial.println("[GCS] Sent: " + input);
      } else {
        // Assume it's a simple command and wrap it
        sendMessage("CMD," + input);
        Serial.println("[GCS] Sent: CMD," + input);
      }
    }
  }
}

/**
 * @brief Sends a message via LoRa and prints it to the Serial Monitor.
 * @param outgoing The String message to be sent.
 */
void sendMessage(String outgoing) {
  LoRa.beginPacket();
  LoRa.print(outgoing);
  LoRa.endPacket();
  
  Serial.print("Sending -> ");
  Serial.println(outgoing);
}

/**
 * @brief Handles received LoRa packets from the CanSat.
 * Expected format: TEAM_ID,TimeStamp,PacketCount,FlightState,Altitude,Pressure,Temperature,Voltage,Latitude,Longitude,AccelX,GyroY,Satellites
 * @param packetSize The size of the incoming packet.
 */
void onReceive(int packetSize) {
  String receivedText = "";
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  
  // Read the packet
  while (LoRa.available()) {
    char c = (char)LoRa.read();
    receivedText += c;
  }
  
  // Check if the data contains printable characters
  bool isPrintable = true;
  int commaCount = 0;
  
  for (int i = 0; i < receivedText.length(); i++) {
    char c = receivedText.charAt(i);
    if (c == ',') commaCount++;
    if (c < 32 || c > 126) { // Non-printable ASCII characters
      isPrintable = false;
    }
  }
  
  if (isPrintable && commaCount >= 10) {
    // This looks like valid CanSat telemetry data (should have ~12 fields)
    // Forward to Python parser in expected format
    Serial.println("PING:" + receivedText);
    
    // Parse and display key telemetry values for monitoring
    String fields[15];
    int fieldIndex = 0;
    int lastComma = -1;
    
    for (int i = 0; i <= receivedText.length(); i++) {
      if (i == receivedText.length() || receivedText.charAt(i) == ',') {
        if (fieldIndex < 15) {
          fields[fieldIndex] = receivedText.substring(lastComma + 1, i);
          fieldIndex++;
        }
        lastComma = i;
      }
    }
    
    // Display parsed telemetry (CanSat format: TEAM_ID,TimeStamp,PacketCount,FlightState,Altitude,Pressure,Temperature,Voltage,Latitude,Longitude,AccelX,GyroY,Satellites)
    Serial.print("[TELEMETRY] ");
    if (fieldIndex >= 13) {
      String flightStates[] = {"BOOT", "TEST", "PAD", "ASCENT", "DEPLOY", "DESCENT", "BRAKE", "IMPACT"};
      int stateNum = fields[3].toInt();
      String stateName = (stateNum >= 0 && stateNum < 8) ? flightStates[stateNum] : "UNK";
      
      Serial.print("Team: " + fields[0]);
      Serial.print(", T: " + fields[1] + "s");
      Serial.print(", Pkt: " + fields[2]);
      Serial.print(", State: " + stateName + "(" + fields[3] + ")");
      Serial.print(", Alt: " + fields[4] + "m");
      Serial.print(", P: " + fields[5] + "Pa");
      Serial.print(", Temp: " + fields[6] + "°C");
      Serial.print(", V: " + fields[7] + "V");
      Serial.print(", GPS: " + fields[8] + "," + fields[9]);
      Serial.print(", Acc: " + fields[10]);
      Serial.print(", Gyro: " + fields[11]);
      Serial.print(", Sats: " + fields[12]);
    }
    Serial.print(" | RSSI: " + String(rssi));
    Serial.println(" | SNR: " + String(snr, 1));
    
  } else if (isPrintable && commaCount > 0) {
    // Some other CSV format, forward anyway
    Serial.println("PING:" + receivedText);
    Serial.println("[INFO] CSV data forwarded to parser");
    
  } else if (isPrintable) {
    // Printable but not CSV format (maybe a command response)
    Serial.println("[MSG] " + receivedText + " (RSSI: " + String(rssi) + ")");
    
  } else {
    // Corrupted or non-printable data
    Serial.print("[ERROR] Corrupted data (");
    Serial.print(receivedText.length());
    Serial.print(" bytes), RSSI: ");
    Serial.print(rssi);
    Serial.print(", SNR: ");
    Serial.println(snr);
    
    // Show signal quality assessment
    if (rssi < -100) {
      Serial.println("[SIGNAL] Very weak signal - move closer or check antennas");
    } else if (rssi < -90) {
      Serial.println("[SIGNAL] Weak signal - data corruption likely");
    } else {
      Serial.println("[SIGNAL] Signal strength OK - check LoRa configuration");
    }
    
    // Print hex representation for debugging
    Serial.print("[HEX] ");
    int maxBytes = receivedText.length() < 20 ? receivedText.length() : 20;
    for (int i = 0; i < maxBytes; i++) {
      if ((unsigned char)receivedText.charAt(i) < 16) Serial.print("0");
      Serial.print((unsigned char)receivedText.charAt(i), HEX);
      Serial.print(" ");
    }
    if (receivedText.length() > 20) Serial.print("...");
    Serial.println();
  }
}
