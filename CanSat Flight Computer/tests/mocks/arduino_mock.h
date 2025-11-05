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
#include <algorithm>
#include <sstream>
#include <iomanip>

// Arduino data types - avoid conflicts with std::string
class ArduinoString {
private:
    std::string data;

public:
    ArduinoString() = default;
    ArduinoString(const char* str) : data(str) {}
    ArduinoString(const std::string& str) : data(str) {}
    ArduinoString(int value) : data(std::to_string(value)) {}
    ArduinoString(float value) : data(std::to_string(value)) {}
    ArduinoString(float value, int decimals) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(decimals) << value;
        data = oss.str();
    }
    
    // Basic operations
    size_t length() const { return data.length(); }
    const char* c_str() const { return data.c_str(); }
    
    // Operators
    ArduinoString operator+(const ArduinoString& other) const {
        return ArduinoString(data + other.data);
    }
    ArduinoString operator+(const char* str) const {
        return ArduinoString(data + str);
    }
    ArduinoString& operator+=(const ArduinoString& other) {
        data += other.data;
        return *this;
    }
    ArduinoString& operator+=(const char* str) {
        data += str;
        return *this;
    }
    bool operator==(const ArduinoString& other) const {
        return data == other.data;
    }
    char operator[](int index) const {
        return (index >= 0 && (size_t)index < data.length()) ? data[index] : '\0';
    }
    
    // Arduino-specific methods
    ArduinoString substr(size_t start, size_t len = std::string::npos) const {
        return ArduinoString(data.substr(start, len));
    }
    
    int indexOf(const char* str) const {
        size_t pos = data.find(str);
        return (pos != std::string::npos) ? (int)pos : -1;
    }
    
    void trim() {
        data.erase(data.begin(), std::find_if(data.begin(), data.end(), [](int ch) {
            return !std::isspace(ch);
        }));
        data.erase(std::find_if(data.rbegin(), data.rend(), [](int ch) {
            return !std::isspace(ch);
        }).base(), data.end());
    }
    
    // Conversion to std::string for compatibility
    operator std::string() const { return data; }
    std::string toString() const { return data; }
    
    // Case-insensitive comparison
    bool equalsIgnoreCase(const char* other) const {
        std::string str1 = data;
        std::string str2 = other;
        std::transform(str1.begin(), str1.end(), str1.begin(), ::tolower);
        std::transform(str2.begin(), str2.end(), str2.begin(), ::tolower);
        return str1 == str2;
    }
    
    bool equalsIgnoreCase(const ArduinoString& other) const {
        return equalsIgnoreCase(other.data.c_str());
    }
    
    // Friend operators for const char* + String
    friend ArduinoString operator+(const char* lhs, const ArduinoString& rhs) {
        return ArduinoString(std::string(lhs) + rhs.data);
    }
};

typedef uint8_t byte;
typedef bool boolean;
typedef ArduinoString String;

// Pin modes and states
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define HIGH 0x1
#define LOW 0x0

// Analog pins
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5

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

// Arduino math functions - use different names to avoid std conflicts
inline int arduino_constrain(int amt, int low, int high) {
    return (amt < low) ? low : ((amt > high) ? high : amt);
}
inline float arduino_constrain(float amt, float low, float high) {
    return (amt < low) ? low : ((amt > high) ? high : amt);
}
inline int arduino_min(int a, int b) { return (a < b) ? a : b; }
inline float arduino_min(float a, float b) { return (a < b) ? a : b; }
inline int arduino_max(int a, int b) { return (a > b) ? a : b; }
inline float arduino_max(float a, float b) { return (a > b) ? a : b; }
inline int arduino_abs(int x) { return (x < 0) ? -x : x; }
inline float arduino_abs(float x) { return (x < 0.0f) ? -x : x; }
inline double arduino_abs(double x) { return (x < 0.0) ? -x : x; }

// Map these to Arduino names only when not conflicting
// Note: Do NOT redefine abs/min/max as they conflict with std library
// Arduino code should use std::abs, std::min, std::max in tests
#ifndef constrain
#define constrain arduino_constrain
#endif

// Map function
inline float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// Serial mock class
class SerialMock {
public:
    void begin(long baud) {}
    void print(const String& str) { std::cout << str.toString(); }
    void print(const char* str) { std::cout << str; }
    void print(int value) { std::cout << value; }
    void print(float value) { std::cout << value; }
    void println(const String& str) { std::cout << str.toString() << std::endl; }
    void println(const char* str) { std::cout << str << std::endl; }
    void println(int value) { std::cout << value << std::endl; }
    void println(float value) { std::cout << value << std::endl; }
    void println() { std::cout << std::endl; }
    int available() { return 0; }
    char read() { return 0; }
    void flush() {}
};

// Wire mock class for I2C
class WireMock {
public:
    void begin() {}
    void begin(int addr) {}
    void setClock(long freq) {}
    void beginTransmission(int addr) {}
    int endTransmission() { return 0; }
    void write(byte data) {}
    void write(const byte* data, int len) {}
    int requestFrom(int addr, int len) { return len; }
    int available() { return 0; }
    byte read() { return 0; }
};

// SPI mock class
class SPIMock {
public:
    void begin() {}
    void end() {}
    void setBitOrder(int order) {}
    void setDataMode(int mode) {}
    void setClockDivider(int div) {}
    byte transfer(byte data) { return data; }
    void transfer(byte* data, int size) {}
};

// Servo mock class
class ServoMock {
public:
    void attach(int pin) { _pin = pin; }
    void write(int angle) { _angle = angle; }
    int read() { return _angle; }
    void detach() {}
private:
    int _pin = -1;
    int _angle = 0;
};

// ESP32 specific mocks
class ESPMock {
public:
    uint32_t getFreeHeap() { return 200000; }
    void restart() {}
};

// FreeRTOS mocks
typedef void* TaskHandle_t;
typedef void* SemaphoreHandle_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;

#define pdPASS 1
#define pdFAIL 0
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((ms))
#define portMAX_DELAY 0xFFFFFFFF

inline BaseType_t xTaskCreatePinnedToCore(
    void (*task)(void*), const char* name, uint32_t stack, 
    void* param, UBaseType_t priority, TaskHandle_t* handle, BaseType_t core) {
    // Mock implementation - would normally create a task
    if (handle) *handle = (TaskHandle_t)0x12345678;
    return pdPASS;
}

inline void vTaskDelay(TickType_t ticks) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ticks));
}

inline SemaphoreHandle_t xSemaphoreCreateMutex() {
    return (SemaphoreHandle_t)0x87654321;
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout) {
    return pdPASS;
}

inline BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) {
    return pdPASS;
}

inline void vSemaphoreDelete(SemaphoreHandle_t sem) {
    // Mock implementation - would normally delete semaphore
}

// Preferences mock class
class PreferencesMock {
public:
    void begin(const char* name, bool readOnly = false) {}
    bool putBytes(const char* key, const void* value, size_t len) { return true; }
    size_t getBytesLength(const char* key) { return 0; }
    size_t getBytes(const char* key, void* buf, size_t maxLen) { return 0; }
    void end() {}
    bool clear() { return true; }
    bool remove(const char* key) { return true; }
};

// Global instances
extern SerialMock Serial;
extern WireMock Wire;
extern SPIMock SPI;
extern ESPMock ESP;
extern PreferencesMock Preferences;

// Forward declare LoRa mock
class LoRaMock;
extern LoRaMock LoRa;

// Global logging function mock
inline void logEvent(const String& message) {
    std::cout << "[LOG] " << message.toString() << std::endl;
}

inline void logEvent(const char* message) {
    std::cout << "[LOG] " << message << std::endl;
}

// Test utilities
class TestUtils {
public:
    static void resetMocks() {
        pin_modes.clear();
        pin_states.clear();
        analog_values.clear();
    }
};

// Mock class for test control
class ArduinoMock {
public:
    void reset();
    void setPinMode(int pin, int mode);
    void setDigitalPin(int pin, int value);
    void setAnalogPin(int pin, int value);
    int mockAnalogRead(int pin);
    void mockDelay(unsigned long ms);
    
private:
    std::map<int, int> mock_pins;
    std::map<int, int> mock_analog;
};

#endif // TESTING_MODE
#endif // ARDUINO_MOCK_H