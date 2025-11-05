#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#ifdef TESTING_MODE

#include <cstdint>
#include <cmath>
#include <string>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include <functional>

// Arduino data types
typedef uint8_t byte;
typedef std::string String;

// Pin modes and states
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define HIGH 0x1
#define LOW 0x0

// Serial configuration
#define SERIAL_8N1 0x06

// Mock pin states
extern std::map<int, int> pin_modes;
extern std::map<int, int> pin_states;
extern std::map<int, int> analog_values;

// Timing functions
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

// Digital I/O
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);

// Math functions
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define pow(base, exp) std::pow(base, exp)
#define sqrt(x) std::sqrt(x)

// String manipulation
class String {
private:
    std::string data;

public:
    String() {}
    String(const char* str) : data(str) {}
    String(const std::string& str) : data(str) {}
    String(int value) : data(std::to_string(value)) {}
    String(unsigned int value) : data(std::to_string(value)) {}
    String(long value) : data(std::to_string(value)) {}
    String(unsigned long value) : data(std::to_string(value)) {}
    String(float value, int decimals = 2) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
        data = buffer;
    }
    String(double value, int decimals = 2) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
        data = buffer;
    }

    // Operators
    String operator+(const String& other) const {
        return String(data + other.data);
    }
    String operator+(const char* str) const {
        return String(data + str);
    }
    String& operator+=(const String& other) {
        data += other.data;
        return *this;
    }
    String& operator+=(const char* str) {
        data += str;
        return *this;
    }
    bool operator==(const String& other) const {
        return data == other.data;
    }
    
    // Methods
    unsigned int length() const { return data.length(); }
    const char* c_str() const { return data.c_str(); }
    String substring(int start, int end = -1) const {
        if (end == -1) end = data.length();
        return String(data.substr(start, end - start));
    }
    void trim() {
        data.erase(0, data.find_first_not_of(" \t\n\r"));
        data.erase(data.find_last_not_of(" \t\n\r") + 1);
    }
    bool equalsIgnoreCase(const String& other) const {
        std::string a = data, b = other.data;
        std::transform(a.begin(), a.end(), a.begin(), ::tolower);
        std::transform(b.begin(), b.end(), b.begin(), ::tolower);
        return a == b;
    }
};

// Serial communication mock
class SerialMock {
public:
    void begin(int baud, int config = SERIAL_8N1, int rx = -1, int tx = -1) {
        (void)baud; (void)config; (void)rx; (void)tx;
        // Mock implementation
    }
    
    void println(const String& str = "") {
        std::cout << str.c_str() << std::endl;
    }
    
    void print(const String& str) {
        std::cout << str.c_str();
    }
    
    int available() { return 0; }
    int read() { return -1; }
    void flush() {}
};

extern SerialMock Serial;
extern SerialMock Serial2;

// Wire (I2C) mock
class WireMock {
public:
    void begin() {}
    void setClock(int frequency) { (void)frequency; }
    void beginTransmission(int address) { (void)address; }
    void endTransmission() {}
    int requestFrom(int address, int length) { (void)address; (void)length; return 0; }
    int write(uint8_t data) { (void)data; return 1; }
    int read() { return 0; }
    int available() { return 0; }
};

extern WireMock Wire;

// SPI mock
class SPIMock {
public:
    void begin() {}
    void end() {}
    void setFrequency(int freq) { (void)freq; }
    uint8_t transfer(uint8_t data) { return data; }
};

extern SPIMock SPI;

// Preferences mock
class PreferencesMock {
private:
    std::map<std::string, std::map<std::string, std::string>> storage;
    std::string current_namespace;
    
public:
    bool begin(const char* name, bool readOnly = false) {
        (void)readOnly;
        current_namespace = name;
        return true;
    }
    
    void end() {}
    void clear() {
        if (!current_namespace.empty()) {
            storage[current_namespace].clear();
        }
    }
    
    // Getters
    uint32_t getUInt(const char* key, uint32_t defaultValue = 0) {
        auto it = storage[current_namespace].find(key);
        return (it != storage[current_namespace].end()) ? 
               std::stoul(it->second) : defaultValue;
    }
    
    uint8_t getUChar(const char* key, uint8_t defaultValue = 0) {
        auto it = storage[current_namespace].find(key);
        return (it != storage[current_namespace].end()) ? 
               (uint8_t)std::stoul(it->second) : defaultValue;
    }
    
    unsigned long getULong(const char* key, unsigned long defaultValue = 0) {
        auto it = storage[current_namespace].find(key);
        return (it != storage[current_namespace].end()) ? 
               std::stoul(it->second) : defaultValue;
    }
    
    float getFloat(const char* key, float defaultValue = 0.0f) {
        auto it = storage[current_namespace].find(key);
        return (it != storage[current_namespace].end()) ? 
               std::stof(it->second) : defaultValue;
    }
    
    bool getBool(const char* key, bool defaultValue = false) {
        auto it = storage[current_namespace].find(key);
        return (it != storage[current_namespace].end()) ? 
               (it->second == "1") : defaultValue;
    }
    
    size_t getBytes(const char* key, void* buf, size_t maxLen) {
        auto it = storage[current_namespace].find(key);
        if (it != storage[current_namespace].end() && buf && maxLen > 0) {
            size_t copyLen = std::min(maxLen, it->second.length());
            memcpy(buf, it->second.c_str(), copyLen);
            return copyLen;
        }
        return 0;
    }
    
    // Setters
    size_t putUInt(const char* key, uint32_t value) {
        storage[current_namespace][key] = std::to_string(value);
        return sizeof(uint32_t);
    }
    
    size_t putUChar(const char* key, uint8_t value) {
        storage[current_namespace][key] = std::to_string(value);
        return sizeof(uint8_t);
    }
    
    size_t putULong(const char* key, unsigned long value) {
        storage[current_namespace][key] = std::to_string(value);
        return sizeof(unsigned long);
    }
    
    size_t putFloat(const char* key, float value) {
        storage[current_namespace][key] = std::to_string(value);
        return sizeof(float);
    }
    
    size_t putBool(const char* key, bool value) {
        storage[current_namespace][key] = value ? "1" : "0";
        return sizeof(bool);
    }
    
    size_t putBytes(const char* key, const void* value, size_t len) {
        if (value && len > 0) {
            storage[current_namespace][key] = std::string((const char*)value, len);
            return len;
        }
        return 0;
    }
};

extern PreferencesMock Preferences;

// FreeRTOS mocks
typedef void* SemaphoreHandle_t;
typedef void* TaskHandle_t;
typedef uint32_t TickType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) (ms)

SemaphoreHandle_t xSemaphoreCreateMutex();
void vSemaphoreDelete(SemaphoreHandle_t sem);
int xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout);
void xSemaphoreGive(SemaphoreHandle_t sem);

void xTaskCreatePinnedToCore(void (*task)(void*), const char* name, 
                            int stackSize, void* param, int priority, 
                            TaskHandle_t* handle, int core);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t* lastWakeTime, TickType_t frequency);
TickType_t xTaskGetTickCount();

// Mock ESP32 functions
class ESPMock {
public:
    uint32_t getFreeHeap() { return 200000; }  // 200KB free heap
};

extern ESPMock ESP;

// Mock function to simulate logging
void logEvent(const String& message);

// Test utilities
namespace TestUtils {
    void setAnalogValue(int pin, int value);
    void setDigitalValue(int pin, int value);
    void simulateTimeAdvance(unsigned long ms);
    void resetMocks();
}

#endif // TESTING_MODE
#endif // ARDUINO_MOCK_H