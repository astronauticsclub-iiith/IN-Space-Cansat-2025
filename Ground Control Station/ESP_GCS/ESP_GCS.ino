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
      
      // Generate mock telemetry data in the format expected by the parser
      String pingMessage = "PING:Packet:" + String(counter) + 
                          ",TS:" + String(millis()) +
                          ",Alt:" + String(100.0 + random(-50, 50)) +  // Mock altitude
                          ",Temp:" + String(25.0 + random(-5, 5)) +    // Mock temperature
                          ",Pressure:" + String(1013.0 + random(-10, 10)) + // Mock pressure
                          ",Lat:" + String(28.6139 + (random(-100, 100) * 0.0001)) + // Mock GPS
                          ",Lon:" + String(77.209 + (random(-100, 100) * 0.0001)) +
                          ",Voltage:" + String(3.3 + random(-0.5, 0.5)) + // Mock voltage
                          ",State:" + String(counter % 4); // Mock system state
      
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