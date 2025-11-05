// TESTED on one xbee connected with esp32 and other with XCTU

/*
 * ==========================================================
 * ESP32 to XBee Communication (Using Custom Pins)
 * ==========================================================
 * * This code uses Serial2 on custom pins GPIO 25 (RX) and 
 * GPIO 26 (TX) because the default pins (16/17) were faulty.
 * * Hardware Setup:
 * - ESP32 GND     -> XBee GND (Pin 10)
 * - ESP32 3V3     -> XBee VCC (Pin 1)  (Or external 3.3V)
 * - ESP32 GPIO 25 -> XBee DOUT (Pin 2)
 * - ESP32 GPIO 26 -> XBee DIN (Pin 3)
 */

// We will use UART2 for the XBee
HardwareSerial XBeeSerial(2); 

// --- DEFINE OUR WORKING PINS ---
#define XBee_RX_PIN  25  // (ESP32's RX, connects to XBee's DOUT)
#define XBee_TX_PIN  26  // (ESP32's TX, connects to XBee's DIN)

void setup() {
  // Start the Serial Monitor (for debugging)
  Serial.begin(115200);
  Serial.println("--- ESP32 XBee Test on Pins 25/26 ---");
  
  // Start the XBee Serial Port (UART2)
  // We MUST specify the pins, otherwise it will default to 16/17
  // begin(baud_rate, config, rxPin, txPin);
  XBeeSerial.begin(9600, SERIAL_8N1, XBee_RX_PIN, XBee_TX_PIN);

  Serial.println("XBee port is open. Ready to send/receive.");
}

void loop() {
  // 1. Send data TO THE XBEE (via GPIO 26)
  XBeeSerial.println("Hello from ESP32!");

  // 2. Send a debug message TO THE MONITOR
  Serial.println("Just sent 'Hello' to XBee.");
  
  // 3. Check for any data coming FROM the XBee (via GPIO 25)
  if (XBeeSerial.available()) {
    
    Serial.print("Received from XBee: ");
    
    // Read all available bytes and print them to the monitor
    while (XBeeSerial.available()) {
      char inChar = XBeeSerial.read();
      Serial.print(inChar);
    }
    Serial.println(); // Add a new line
  }

  delay(5000); // Wait 5 seconds before sending again
}
