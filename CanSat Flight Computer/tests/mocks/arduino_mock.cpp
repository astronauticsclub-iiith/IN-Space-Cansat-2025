#ifdef TESTING_MODE

#include "arduino_mock.h"
#include <chrono>
#include <thread>

// Global mock state
std::map<int, int> pin_modes;
std::map<int, int> pin_states;
std::map<int, int> analog_values;

// Global instances
SerialMock Serial;
WireMock Wire;
SPIMock SPI;
ESPMock ESP;
PreferencesMock Preferences;

// SPIFFS mock class implementation
class SPIFFSMock {
public:
    bool begin(bool formatOnFail = false) { return true; }
    void end() {}
    bool format() { return true; }
    size_t totalBytes() { return 1000000; }
    size_t usedBytes() { return 500000; }
};

SPIFFSMock SPIFFS;

// LoRa mock class implementation
class LoRaMock {
public:
    bool begin(long frequency) { return true; }
    void setPins(int ss = -1, int reset = -1, int dio0 = -1) {}
    void setSpreadingFactor(int sf) {}
    void setSignalBandwidth(long sbw) {}
    void setCodingRate4(int denominator) {}
    void setPreambleLength(long length) {}
    void setSyncWord(int sw) {}
    void enableCrc() {}
    void disableCrc() {}
    
    int beginPacket(int implicitHeader = false) { return 1; }
    int endPacket(bool async = false) { return 1; }
    int parsePacket(int size = 0) { return 0; }
    int packetRssi() { return -50; }
    float packetSnr() { return 10.0; }
    long packetFrequencyError() { return 0; }
    
    size_t write(uint8_t byte) { return 1; }
    size_t write(const uint8_t *buffer, size_t size) { return size; }
    int available() { return 0; }
    int read() { return 0; }
    int peek() { return 0; }
    void flush() {}
    
    void onReceive(void(*callback)(int)) {}
    void onTxDone(void(*callback)()) {}
    
    void receive(int size = 0) {}
    void idle() {}
    void sleep() {}
    
    void setTxPower(int level, int outputPin = 1) {}
    void setFrequency(long frequency) {}
    void setOCP(uint8_t mA) {}
    void setGain(uint8_t gain) {}
    
    uint8_t random() { return 42; }
    void enableInvertIQ() {}
    void disableInvertIQ() {}
    
    void onCadDone(void(*callback)(boolean)) {}
    int rssi() { return -50; }
    
    void setLdoFlag() {}
    
    size_t print(const String& s) { return s.length(); }
    size_t print(const char* str) { return strlen(str); }
    size_t print(int n) { return 1; }
    size_t print(float f) { return 1; }
    size_t println(const String& s) { return s.length() + 1; }
    size_t println(const char* str) { return strlen(str) + 1; }
    size_t println(int n) { return 2; }
    size_t println(float f) { return 2; }
    size_t println() { return 1; }
};

LoRaMock LoRa;

// Mock implementation of timing functions
unsigned long millis() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

unsigned long micros() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
}

void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void delayMicroseconds(unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

// Mock implementation of digital I/O functions
void pinMode(uint8_t pin, uint8_t mode) {
    pin_modes[pin] = mode;
}

void digitalWrite(uint8_t pin, uint8_t val) {
    pin_states[pin] = val;
}

int digitalRead(uint8_t pin) {
    return pin_states.count(pin) ? pin_states[pin] : LOW;
}

int analogRead(uint8_t pin) {
    return analog_values.count(pin) ? analog_values[pin] : 0;
}

void analogWrite(uint8_t pin, int val) {
    pin_states[pin] = val;
}

// Mock control methods
void ArduinoMock::reset() {
    pin_modes.clear();
    pin_states.clear();
    analog_values.clear();
    mock_pins.clear();
    mock_analog.clear();
}

void ArduinoMock::setPinMode(int pin, int mode) {
    pin_modes[pin] = mode;
}

void ArduinoMock::setDigitalPin(int pin, int value) {
    pin_states[pin] = value;
}

void ArduinoMock::setAnalogPin(int pin, int value) {
    analog_values[pin] = value;
}

int ArduinoMock::mockAnalogRead(int pin) {
    return analog_values.count(pin) ? analog_values[pin] : 0;
}

void ArduinoMock::mockDelay(unsigned long ms) {
    // Mock delay without actually delaying in tests
}

#endif // TESTING_MODE