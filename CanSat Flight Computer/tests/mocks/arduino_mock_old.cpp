#include "arduino_mock.h"

#ifdef TESTING_MODE

#include <iostream>
#include <algorithm>
#include <cctype>

// Global variables
std::map<int, int> pin_modes;
std::map<int, int> pin_states;
std::map<int, int> analog_values;

SerialMock Serial;
SerialMock Serial2;
WireMock Wire;
SPIMock SPI;
PreferencesMock Preferences;
ESPMock ESP;

// Timing simulation
static auto start_time = std::chrono::steady_clock::now();
static unsigned long time_offset = 0;

unsigned long millis() {
    auto current = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current - start_time);
    return elapsed.count() + time_offset;
}

unsigned long micros() {
    auto current = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current - start_time);
    return elapsed.count() + (time_offset * 1000);
}

void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void delayMicroseconds(unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

// Digital I/O
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

// FreeRTOS mocks
static std::map<SemaphoreHandle_t, bool> semaphores;
static int next_sem_id = 1;

SemaphoreHandle_t xSemaphoreCreateMutex() {
    SemaphoreHandle_t handle = (SemaphoreHandle_t)(intptr_t)next_sem_id++;
    semaphores[handle] = true;  // Available
    return handle;
}

void vSemaphoreDelete(SemaphoreHandle_t sem) {
    semaphores.erase(sem);
}

int xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout) {
    (void)timeout;
    if (semaphores.count(sem) && semaphores[sem]) {
        semaphores[sem] = false;  // Taken
        return pdTRUE;
    }
    return pdFALSE;
}

void xSemaphoreGive(SemaphoreHandle_t sem) {
    if (semaphores.count(sem)) {
        semaphores[sem] = true;  // Available
    }
}

// Task creation (simplified - just for compilation)
void xTaskCreatePinnedToCore(void (*task)(void*), const char* name, 
                            int stackSize, void* param, int priority, 
                            TaskHandle_t* handle, int core) {
    (void)task; (void)name; (void)stackSize; (void)param; 
    (void)priority; (void)handle; (void)core;
    // In tests, we don't actually create tasks
}

void vTaskDelay(TickType_t ticks) {
    delay(ticks);  // Simple delay
}

void vTaskDelayUntil(TickType_t* lastWakeTime, TickType_t frequency) {
    (void)lastWakeTime; (void)frequency;
    delay(frequency);
}

TickType_t xTaskGetTickCount() {
    return millis();
}

// Logging function
void logEvent(const String& message) {
    std::cout << "[" << millis() << "] " << message.c_str() << std::endl;
}

// Test utilities
namespace TestUtils {
    void setAnalogValue(int pin, int value) {
        analog_values[pin] = value;
    }
    
    void setDigitalValue(int pin, int value) {
        pin_states[pin] = value;
    }
    
    void simulateTimeAdvance(unsigned long ms) {
        time_offset += ms;
    }
    
    void resetMocks() {
        pin_modes.clear();
        pin_states.clear();
        analog_values.clear();
        semaphores.clear();
        time_offset = 0;
        start_time = std::chrono::steady_clock::now();
    }
}

#endif // TESTING_MODE