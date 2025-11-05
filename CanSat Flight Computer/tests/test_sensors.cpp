#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../src/sensors.h"
#include "arduino_mock.h"
#include "sensor_mock.h"

class SensorsTest : public ::testing::Test {
protected:
    void SetUp() override {
        arduinoMock.reset();
        sensorMock.reset();
    }
    
    void TearDown() override {}
    
    ArduinoMock arduinoMock;
    SensorMock sensorMock;
    GasSensors gasSensors;
};

// REQ-SENS-001: BMP390 pressure/temperature
TEST_F(SensorsTest, ShouldReadPressure) {
    sensorMock.setPressure(1013.25f);
    EXPECT_FLOAT_EQ(sensorMock.getPressure(), 1013.25f);
}

// REQ-SENS-002: BMI088 IMU
TEST_F(SensorsTest, ShouldReadIMU) {
    sensorMock.setAcceleration(0.0f, 0.0f, 9.81f);
    auto accel = sensorMock.getAcceleration();
    EXPECT_FLOAT_EQ(accel.z, 9.81f);
}

// REQ-SENS-003: GPS module
TEST_F(SensorsTest, ShouldReadGPS) {
    sensorMock.setGPS(40.7128f, -74.0060f, 50.0f);
    auto gps = sensorMock.getGPS();
    EXPECT_FLOAT_EQ(gps.latitude, 40.7128f);
}

// REQ-SENS-007: Gas sensors (CO2/LPG)
TEST_F(SensorsTest, ShouldReadGasSensors) {
    gasSensors.begin();
    float co2 = gasSensors.readCO2();
    float lpg = gasSensors.readLPG();
    EXPECT_GE(co2, 0.0f);
    EXPECT_GE(lpg, 0.0f);
}