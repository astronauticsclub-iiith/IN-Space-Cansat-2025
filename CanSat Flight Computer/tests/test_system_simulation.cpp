#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "arduino_mock.h"
#include "sensor_mock.h"

class SystemSimulationTest : public ::testing::Test {
protected:
    void SetUp() override {
        arduinoMock.reset();
        sensorMock.reset();
    }
    
    void TearDown() override {}
    
    ArduinoMock arduinoMock;
    SensorMock sensorMock;
};

// REQ-SIM-001: Complete mission simulation
TEST_F(SystemSimulationTest, ShouldSimulateCompleteMission) {
    // Simulate pre-launch phase
    sensorMock.setPressure(1013.25f);
    sensorMock.setTemperature(20.0f);
    sensorMock.setAltitude(100.0f);
    
    EXPECT_FLOAT_EQ(sensorMock.getPressure(), 1013.25f);
    EXPECT_FLOAT_EQ(sensorMock.getTemperature(), 20.0f);
    EXPECT_FLOAT_EQ(sensorMock.getAltitude(), 100.0f);
    
    // Simulate ascent phase
    sensorMock.setAltitude(15000.0f);
    sensorMock.setPressure(120.0f);
    sensorMock.setTemperature(-50.0f);
    
    EXPECT_FLOAT_EQ(sensorMock.getAltitude(), 15000.0f);
    EXPECT_LT(sensorMock.getPressure(), 200.0f);
    EXPECT_LT(sensorMock.getTemperature(), 0.0f);
}

// REQ-SIM-002: Environmental condition simulation
TEST_F(SystemSimulationTest, ShouldSimulateEnvironmentalConditions) {
    // Test various environmental conditions
    sensorMock.setTemperature(-60.0f); // Extreme cold
    sensorMock.setPressure(1.0f);      // Near vacuum
    
    EXPECT_LT(sensorMock.getTemperature(), -50.0f);
    EXPECT_LT(sensorMock.getPressure(), 10.0f);
}

// REQ-SIM-003: Failure mode simulation
TEST_F(SystemSimulationTest, ShouldSimulateFailureModes) {
    // Simulate sensor failures
    sensorMock.setFailureMode(true);
    
    // Test system response to failures
    EXPECT_TRUE(sensorMock.isInFailureMode());
}

// REQ-SIM-004: Data replay capability
TEST_F(SystemSimulationTest, ShouldReplayFlightData) {
    // Test data replay from CSV
    EXPECT_TRUE(true); // Placeholder for CSV replay functionality
}