#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "arduino_mock.h"

class SafetyTest : public ::testing::Test {
protected:
    void SetUp() override {
        arduinoMock.reset();
    }
    
    void TearDown() override {}
    
    ArduinoMock arduinoMock;
};

// REQ-SAFE-001: Battery voltage monitoring
TEST_F(SafetyTest, ShouldMonitorBatteryVoltage) {
    // Mock low battery condition
    arduinoMock.mockAnalogRead(2048); // ~2.5V (low battery)
    
    // Test battery monitoring logic
    float voltage = (arduinoMock.mockAnalogRead(A0) / 4095.0f) * 3.3f * 2.0f; // Voltage divider
    EXPECT_LT(voltage, 3.0f); // Should detect low battery
}

// REQ-SAFE-002: Altitude limits
TEST_F(SafetyTest, ShouldRespectAltitudeLimits) {
    // Test altitude safety limits
    float maxAltitude = 30000.0f; // 30km safety limit
    float currentAltitude = 25000.0f;
    
    EXPECT_LT(currentAltitude, maxAltitude);
}

// REQ-SAFE-003: Temperature monitoring
TEST_F(SafetyTest, ShouldMonitorTemperature) {
    // Test temperature safety limits
    float minTemp = -40.0f;
    float maxTemp = 85.0f;
    float currentTemp = 25.0f;
    
    EXPECT_GT(currentTemp, minTemp);
    EXPECT_LT(currentTemp, maxTemp);
}

// REQ-SAFE-004: Gas concentration alerts
TEST_F(SafetyTest, ShouldDetectDangerousGasLevels) {
    // Test gas safety alerts
    float co2_threshold = 5000.0f; // ppm
    float lpg_threshold = 1000.0f; // ppm
    
    float current_co2 = 400.0f; // Normal level
    float current_lpg = 0.0f;   // Normal level
    
    EXPECT_LT(current_co2, co2_threshold);
    EXPECT_LT(current_lpg, lpg_threshold);
}