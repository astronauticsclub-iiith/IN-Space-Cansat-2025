/*********
 * Combined LoRa Transceiver with Mode Selection
 * - Use the #define flags in the CONFIGURATION section to set the device's role.
 * - CANSAT_MODE: Automatically sends telemetry pings and listens for commands.
 * - GROUND_STATION_MODE: Listens for pings and sends commands via the Serial Monitor.
*********/

#include <SPI.h>
#include <LoRa.h>

// --- CONFIGURATION ---
// Uncomment the mode you want to use for this device
#define CANSAT_MODE
//#define GROUND_STATION_MODE
// --------------------

// DEFINE THE PINS USED BY THE TRANSCEIVER MODULE
#define ss 5
#define rst 14
#define dio0 2

// VARIABLES FOR NON-BLOCKING TIMER (used in CanSat mode)
unsigned long lastSendTime = 0;
const long sendInterval = 1000; // interval at which to send pings

int counter = 0; // packet counter

void setup() {
  // INITIALIZE SERIAL MONITOR
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Duplex Transceiver");
  Serial.println("-------------------------");

  // SETUP LORA TRANSCEIVER MODULE
  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xF3);

  // Print the current operating mode to the Serial Monitor
  #if defined(CANSAT_MODE)
    Serial.println("Mode: CanSat 🚀");
  #elif defined(GROUND_STATION_MODE)
    Serial.println("Mode: Ground Station 📡");
  #else
    Serial.println("ERROR: No mode selected! Please uncomment a mode in the CONFIGURATION section.");
    while(1); // Halt if no mode is selected
  #endif
  
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  // Both modes need to be able to receive packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    onReceive(packetSize);
  }

  // --- Mode-Specific Tasks ---

  #if defined(CANSAT_MODE)
    // CanSat Mode: Send a telemetry ping periodically
    if (millis() - lastSendTime > sendInterval) {
      lastSendTime = millis();
      String pingMessage = generateTelemetryData();
      sendMessage(pingMessage);
      counter++;
    }
  #endif

  #if defined(GROUND_STATION_MODE)
    // Ground Station Mode: Check for commands from the Serial Monitor to send
    if (Serial.available() > 0) {
      String commandToSend = Serial.readString();
      commandToSend.trim();
      sendMessage("CMD:" + commandToSend);
    }
  #endif
}

/**
 * @brief Generates telemetry data in the required CSV format
 */
String generateTelemetryData() {
  // Generate random sensor data for testing
  String telemetry = "PING:";
  telemetry += "Packet:" + String(counter);
  telemetry += ",TS:" + String(millis());
  telemetry += ",State:" + String(random(0, 5)); // Flight states 0-4
  telemetry += ",Altitude:" + String(random(0, 3000) + random(0, 100) * 0.01, 2); // 0-3000m with decimals
  telemetry += ",Pressure:" + String(random(800, 1200) + random(0, 100) * 0.01, 2); // 800-1200 hPa
  telemetry += ",Temp:" + String(random(-20, 50) + random(0, 100) * 0.01, 2); // -20 to 50°C
  telemetry += ",Voltage:" + String(random(300, 420) * 0.01, 2); // 3.0-4.2V battery
  telemetry += ",Lat:" + String(28.5 + random(-100, 100) * 0.001, 6); // Around Delhi coordinates
  telemetry += ",Lon:" + String(77.2 + random(-100, 100) * 0.001, 6);
  telemetry += ",Sats:" + String(random(4, 12)); // GPS satellites
  telemetry += ",BNO_qi:" + String(random(-100, 100) * 0.01, 3); // Quaternion data
  telemetry += ",BNO_qj:" + String(random(-100, 100) * 0.01, 3);
  telemetry += ",BNO_qk:" + String(random(-100, 100) * 0.01, 3);
  telemetry += ",BNO_qr:" + String(random(-100, 100) * 0.01, 3);
  telemetry += ",SHT_Temp:" + String(random(15, 35) + random(0, 100) * 0.01, 2); // SHT sensor temp
  telemetry += ",SHT_Hum:" + String(random(30, 80) + random(0, 100) * 0.01, 2); // Humidity %
  telemetry += ",BME_Press:" + String(random(950, 1050) + random(0, 100) * 0.01, 2); // BME pressure
  telemetry += ",BME_Temp:" + String(random(18, 32) + random(0, 100) * 0.01, 2); // BME temp
  telemetry += ",BME_Hum:" + String(random(35, 75) + random(0, 100) * 0.01, 2); // BME humidity
  telemetry += ",BME_Gas:" + String(random(10000, 50000)); // Gas resistance
  telemetry += ",BME_IAQ:" + String(random(25, 200)); // Indoor Air Quality index
  telemetry += ",BMP_Press:" + String(random(940, 1060) + random(0, 100) * 0.01, 2); // BMP pressure
  telemetry += ",BMP_Temp:" + String(random(16, 34) + random(0, 100) * 0.01, 2); // BMP temp
  telemetry += ",ADS_V:" + String(random(280, 350) * 0.01, 2); // ADS voltage reading
  
  return telemetry;
}

/**
 * @brief Sends a message via LoRa and prints it to the Serial Monitor.
 */
void sendMessage(String outgoing) {
  LoRa.beginPacket();
  LoRa.print(outgoing);
  LoRa.endPacket();
  
  Serial.print("Sending -> ");
  Serial.println(outgoing);
}

/**
 * @brief Handles received LoRa packets.
 */
void onReceive(int packetSize) {
  Serial.print("Received <- '");
  String receivedText = "";
  while (LoRa.available()) {
    receivedText += (char)LoRa.read();
  }
  Serial.print(receivedText);
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());
}
