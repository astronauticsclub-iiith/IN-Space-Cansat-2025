#ifndef SENSOR_MOCK_H
#define SENSOR_MOCK_H

#ifdef TESTING_MODE

#include "arduino_mock.h"
#include <random>
#include <cmath>

/**
 * Mock sensor classes for testing CanSat flight software
 * Provides realistic sensor data simulation with controllable noise and failures
 */

// Base class for sensor simulation
class SensorSimulator {
protected:
    bool enabled = true;
    bool shouldFail = false;
    float noiseLevel = 0.0f;
    std::mt19937 rng{42};  // Fixed seed for reproducible tests
    
public:
    void setEnabled(bool state) { enabled = state; }
    void setFailure(bool fail) { shouldFail = fail; }
    void setNoiseLevel(float level) { noiseLevel = level; }
    
    float addNoise(float value) {
        if (noiseLevel > 0) {
            std::normal_distribution<float> noise(0, noiseLevel);
            return value + noise(rng);
        }
        return value;
    }
};

// Flight simulation state
struct FlightSimState {
    float time = 0.0f;
    float altitude = 0.0f;
    float velocity = 0.0f;
    float acceleration = 0.0f;
    bool launched = false;
    bool apogee_reached = false;
    bool parachute_deployed = false;
    bool landed = false;
    
    // Environmental
    float temperature = 20.0f;
    float pressure = 101325.0f;  // Pa at sea level
    float humidity = 50.0f;
    
    // GPS
    double latitude = 40.7128;   // New York
    double longitude = -74.0060;
    
    // IMU
    float accel_x = 0.0f, accel_y = 0.0f, accel_z = 1.0f;  // 1g down
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
    
    void update(float dt);
    void reset();
};

extern FlightSimState g_flight_state;

// Mock LPS22 barometer
class MockLPS22 : public SensorSimulator {
public:
    bool begin_I2C(int address = 0x5D) {
        (void)address;
        return enabled && !shouldFail;
    }
    
    void setDataRate(int rate) { (void)rate; }
    
    bool getEvent(void* pressure_event, void* temp_event) {
        if (!enabled || shouldFail) return false;
        
        // Simulate pressure based on altitude
        // P = P0 * (1 - 0.0065 * h / T0)^(g * M / (R * 0.0065))
        float h = g_flight_state.altitude;
        float pressure_hPa = 1013.25f * pow(1.0f - 0.0065f * h / 288.15f, 5.255f);
        float temperature = 15.0f - 0.0065f * h;  // Standard atmosphere
        
        // Add noise
        pressure_hPa = addNoise(pressure_hPa);
        temperature = addNoise(temperature);
        
        // Mock sensor event structure (simplified)
        float* p_data = (float*)pressure_event;
        float* t_data = (float*)temp_event;
        if (p_data) *p_data = pressure_hPa;
        if (t_data) *t_data = temperature;
        
        return true;
    }
};

// Mock BME680 environmental sensor
class MockBME680 : public SensorSimulator {
private:
    float _temperature = 20.0f;
    float _pressure = 101325.0f;
    float _humidity = 50.0f;
    float _gas_resistance = 100000.0f;
    
public:
    bool begin(int address = 0x76) {
        (void)address;
        return enabled && !shouldFail;
    }
    
    bool performReading() {
        if (!enabled || shouldFail) return false;
        
        // Update based on flight state
        float h = g_flight_state.altitude;
        _pressure = 101325.0f * pow(1.0f - 0.0065f * h / 288.15f, 5.255f);
        _temperature = 15.0f - 0.0065f * h + addNoise(0);
        _humidity = 50.0f + addNoise(0);
        _gas_resistance = 100000.0f + addNoise(0);
        
        return true;
    }
    
    // Properties
    float temperature() const { return _temperature; }
    float pressure() const { return _pressure; }
    float humidity() const { return _humidity; }
    float gas_resistance() const { return _gas_resistance; }
};

// Mock ICM-20948 IMU
enum ICM_20948_Status_e {
    ICM_20948_Stat_Ok = 0,
    ICM_20948_Stat_Err,
    ICM_20948_Stat_NotImpl,
};

struct ICM_20948_smplrt_t {
    uint8_t a;  // Accelerometer sample rate divisor
    uint8_t g;  // Gyroscope sample rate divisor
};

class MockICM20948 : public SensorSimulator {
private:
    float _accel_x = 0.0f, _accel_y = 0.0f, _accel_z = 1.0f;
    float _gyro_x = 0.0f, _gyro_y = 0.0f, _gyro_z = 0.0f;
    
public:
    ICM_20948_Status_e status = ICM_20948_Stat_Ok;
    
    ICM_20948_Status_e begin(void* wire, int address = 0x69) {
        (void)wire; (void)address;
        if (enabled && !shouldFail) {
            status = ICM_20948_Stat_Ok;
        } else {
            status = ICM_20948_Stat_Err;
        }
        return status;
    }
    
    ICM_20948_Status_e setSampleRate(int bank, ICM_20948_smplrt_t rate) {
        (void)bank; (void)rate;
        return status;
    }
    
    bool dataReady() {
        return enabled && !shouldFail;
    }
    
    void getAGMT() {
        if (!enabled || shouldFail) return;
        
        // Simulate IMU data based on flight state
        _accel_z = 1.0f + g_flight_state.acceleration / 9.81f;  // Add flight acceleration to gravity
        _accel_x = addNoise(0.0f);
        _accel_y = addNoise(0.0f);
        _accel_z = addNoise(_accel_z);
        
        // Add some rotational motion during ascent/descent
        if (g_flight_state.launched && !g_flight_state.landed) {
            _gyro_z = addNoise(5.0f);  // Some spin
        } else {
            _gyro_x = addNoise(0.0f);
            _gyro_y = addNoise(0.0f);
            _gyro_z = addNoise(0.0f);
        }
    }
    
    // Data accessors
    float accX() const { return _accel_x; }
    float accY() const { return _accel_y; }
    float accZ() const { return _accel_z; }
    float gyrX() const { return _gyro_x; }
    float gyrY() const { return _gyro_y; }
    float gyrZ() const { return _gyro_z; }
};

// Mock TinyGPSPlus
class MockLocation {
private:
    bool _isValid = false;
    double _lat = 0.0, _lng = 0.0;
    
public:
    bool isValid() const { return _isValid; }
    bool isUpdated() const { return _isValid; }
    double lat() const { return _lat; }
    double lng() const { return _lng; }
    
    void set(double latitude, double longitude, bool valid = true) {
        _lat = latitude;
        _lng = longitude;
        _isValid = valid;
    }
};

class MockAltitude {
private:
    bool _isValid = false;
    double _meters = 0.0;
    
public:
    bool isValid() const { return _isValid; }
    double meters() const { return _meters; }
    
    void set(double altitude, bool valid = true) {
        _meters = altitude;
        _isValid = valid;
    }
};

class MockTime {
private:
    bool _isValid = false;
    uint8_t _hour = 0, _minute = 0, _second = 0;
    
public:
    bool isValid() const { return _isValid; }
    uint8_t hour() const { return _hour; }
    uint8_t minute() const { return _minute; }
    uint8_t second() const { return _second; }
    
    void set(uint8_t h, uint8_t m, uint8_t s, bool valid = true) {
        _hour = h;
        _minute = m;
        _second = s;
        _isValid = valid;
    }
};

class MockSatellites {
private:
    uint32_t _value = 0;
    
public:
    uint32_t value() const { return _value; }
    void set(uint32_t count) { _value = count; }
};

class MockTinyGPSPlus : public SensorSimulator {
public:
    MockLocation location;
    MockAltitude altitude;
    MockTime time;
    MockSatellites satellites;
    
    bool encode(char c) {
        (void)c;
        if (!enabled || shouldFail) return false;
        
        // Simulate GPS fix based on flight state
        if (g_flight_state.time > 10.0f) {  // GPS takes time to acquire
            location.set(g_flight_state.latitude, g_flight_state.longitude, true);
            altitude.set(g_flight_state.altitude, true);
            time.set(12, 0, (uint8_t)g_flight_state.time, true);
            satellites.set(8);
        }
        
        return true;
    }
    
    void simulateNoFix() {
        location.set(0, 0, false);
        altitude.set(0, false);
        time.set(0, 0, 0, false);
        satellites.set(0);
    }
};

// Mock Servo
class MockServo : public SensorSimulator {
private:
    int _position = 90;
    bool _attached = false;
    
public:
    void setPeriodHertz(int frequency) { (void)frequency; }
    
    bool attach(int pin, int min_us = 544, int max_us = 2400) {
        (void)pin; (void)min_us; (void)max_us;
        _attached = enabled && !shouldFail;
        return _attached;
    }
    
    void detach() { _attached = false; }
    
    void write(int value) {
        if (_attached) {
            _position = value;
        }
    }
    
    int read() const { return _position; }
    bool attached() const { return _attached; }
};

// Mock ESP32PWM
class MockESP32PWM {
public:
    static bool allocateTimer(int timerNum) {
        (void)timerNum;
        return true;
    }
};

// LoRa mock
class MockLoRa {
private:
    bool _initialized = false;
    String _receivedData;
    
public:
    void setPins(int ss, int reset, int dio0) {
        (void)ss; (void)reset; (void)dio0;
    }
    
    bool begin(long frequency) {
        (void)frequency;
        _initialized = true;
        return true;
    }
    
    void setSpreadingFactor(int sf) { (void)sf; }
    void setSignalBandwidth(long sbw) { (void)sbw; }
    void setCodingRate4(int denominator) { (void)denominator; }
    void enableCrc() {}
    
    void beginPacket() {}
    void endPacket() {}
    void print(const String& data) {
        // In tests, we can capture transmitted data
        std::cout << "[LoRa TX]: " << data.c_str() << std::endl;
    }
    
    int parsePacket() {
        // Return length of received packet (0 if none)
        return _receivedData.length();
    }
    
    int available() {
        return _receivedData.length();
    }
    
    int read() {
        if (_receivedData.length() > 0) {
            char c = _receivedData.c_str()[0];
            _receivedData = _receivedData.substr(1);
            return c;
        }
        return -1;
    }
    
    // Test utility to simulate received command
    void simulateReceive(const String& data) {
        _receivedData = data;
    }
};

// Global mock instances
extern MockLPS22 mock_lps;
extern MockBME680 mock_bme;
extern MockICM20948 mock_icm;
extern MockTinyGPSPlus mock_gps;
extern MockServo mock_servo;
extern MockLoRa mock_lora;

// Test utilities
namespace SensorMocks {
    void initializeAll();
    void resetAll();
    void simulateFlightProfile();
    void setFlightTime(float time_seconds);
    void simulateLaunch();
    void simulateApogee();
    void simulateLanding();
    void setSensorFailure(const String& sensor, bool failed);
    void setSensorNoise(const String& sensor, float noise_level);
}

#endif // TESTING_MODE
#endif // SENSOR_MOCK_H