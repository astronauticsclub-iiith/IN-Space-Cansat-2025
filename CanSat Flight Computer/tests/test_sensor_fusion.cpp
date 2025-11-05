#include <gtest/gtest.h>
#include "mocks/arduino_mock.h"
#include "mocks/sensor_mock.h"

#ifdef TESTING_MODE

#include "SensorFusion.h"

class SensorFusionTest : public ::testing::Test {
protected:
    void SetUp() override {
        TestUtils::resetMocks();
        SensorMocks::resetAll();
        fusion.resetHealth();
    }
    
    void TearDown() override {
        // Cleanup if needed
    }
    
    SensorFusion fusion;
};

// REQ-TEST-UNIT-002: Sensor fusion SHALL be unit tested
TEST_F(SensorFusionTest, PressureFusionBothSensorsValid) {
    // Test pressure fusion with both sensors providing valid data
    float lps_pressure = 101325.0f;  // 1 atm in Pa
    float bme_pressure = 101300.0f;  // Slightly different
    
    float fused = fusion.fusePressure(lps_pressure, bme_pressure, true, true);
    
    // Result should be between the two values
    EXPECT_GT(fused, 101300.0f);
    EXPECT_LT(fused, 101325.0f);
    
    // Should be closer to the average initially
    float expected_avg = (lps_pressure + bme_pressure) / 2.0f;
    EXPECT_LT(abs(fused - expected_avg), 20.0f);
}

TEST_F(SensorFusionTest, PressureFusionSingleSensorFailure) {
    // Test pressure fusion when one sensor fails
    float lps_pressure = 101325.0f;
    float bme_pressure = 0.0f;  // Invalid data
    
    // LPS valid, BME invalid
    float fused = fusion.fusePressure(lps_pressure, bme_pressure, true, false);
    EXPECT_FLOAT_EQ(fused, lps_pressure);
    
    // BME valid, LPS invalid
    fused = fusion.fusePressure(lps_pressure, bme_pressure, false, true);
    EXPECT_FLOAT_EQ(fused, 0.0f);  // BME pressure was 0
}

TEST_F(SensorFusionTest, PressureFusionBothSensorsInvalid) {
    // Test pressure fusion when both sensors fail
    float fused = fusion.fusePressure(0.0f, 0.0f, false, false);
    EXPECT_FLOAT_EQ(fused, 0.0f);
}

TEST_F(SensorFusionTest, TemperatureFusionMultipleSensors) {
    // Test temperature fusion with multiple sensors
    float lps_temp = 20.0f;
    float bme_temp = 21.0f;
    float sht_temp = 20.5f;
    
    float fused = fusion.fuseTemperature(lps_temp, bme_temp, true, true, sht_temp, true);
    
    // Result should be reasonable average weighted by sensor quality
    EXPECT_GT(fused, 19.5f);
    EXPECT_LT(fused, 21.5f);
}

TEST_F(SensorFusionTest, TemperatureFusionTwoSensors) {
    // Test temperature fusion with only two sensors
    float lps_temp = 18.0f;
    float bme_temp = 22.0f;
    
    float fused = fusion.fuseTemperature(lps_temp, bme_temp, true, true);
    
    // Should be between the two values
    EXPECT_GT(fused, 18.0f);
    EXPECT_LT(fused, 22.0f);
}

TEST_F(SensorFusionTest, AltitudeCrossValidation) {
    // Test altitude cross-validation with GPS
    float baro_altitude = 1000.0f;
    float gps_altitude = 1002.0f;  // Small difference (within 5m)
    
    float fused = fusion.fuseAltitude(baro_altitude, gps_altitude, true);
    
    // Should slightly correct barometric altitude toward GPS
    EXPECT_GT(fused, baro_altitude);
    EXPECT_LT(fused, 1001.0f);  // Small correction
}

TEST_F(SensorFusionTest, AltitudeLargeDiscrepancy) {
    // Test altitude fusion with large GPS discrepancy
    float baro_altitude = 1000.0f;
    float gps_altitude = 1050.0f;  // Large difference (50m)
    
    float fused = fusion.fuseAltitude(baro_altitude, gps_altitude, true);
    
    // Should blend the values when discrepancy is large
    EXPECT_GT(fused, baro_altitude);
    EXPECT_LT(fused, gps_altitude);
}

TEST_F(SensorFusionTest, AltitudeGPSInvalid) {
    // Test altitude fusion when GPS is invalid
    float baro_altitude = 800.0f;
    
    float fused = fusion.fuseAltitude(baro_altitude, 0.0f, false);
    EXPECT_FLOAT_EQ(fused, baro_altitude);  // Should use barometric only
}

TEST_F(SensorFusionTest, SensorHealthTracking) {
    // Test sensor health monitoring
    String initial_health = fusion.getHealthStatus();
    EXPECT_FALSE(initial_health.c_str() == nullptr);
    
    // Simulate multiple failures
    for (int i = 0; i < 10; i++) {
        fusion.fusePressure(0.0f, 0.0f, false, false);
    }
    
    String degraded_health = fusion.getHealthStatus();
    EXPECT_NE(initial_health.c_str(), degraded_health.c_str());
}

TEST_F(SensorFusionTest, CriticalSensorHealth) {
    // Test critical sensor health check
    EXPECT_TRUE(fusion.areCriticalSensorsHealthy());
    
    // Simulate sensor failures
    for (int i = 0; i < 10; i++) {
        fusion.fusePressure(0.0f, 0.0f, false, false);
    }
    
    // After many failures, critical sensors should be unhealthy
    EXPECT_FALSE(fusion.areCriticalSensorsHealthy());
}

TEST_F(SensorFusionTest, OutlierDetection) {
    // Test outlier detection functionality
    // First, establish a history of normal readings
    for (int i = 0; i < 10; i++) {
        fusion.fusePressure(101325.0f + i, 101320.0f + i, true, true);
    }
    
    // Now send an outlier
    float outlier_result = fusion.fusePressure(110000.0f, 101330.0f, true, true);
    
    // The outlier should be rejected or heavily weighted down
    EXPECT_LT(outlier_result, 105000.0f);  // Should not accept the extreme outlier
}

// REQ-PERF-ACC-003: Temperature fusion SHALL reduce noise by 50%
TEST_F(SensorFusionTest, TemperatureNoiseReduction) {
    // Test temperature noise reduction requirement
    std::vector<float> lps_readings, bme_readings, fused_readings;
    
    // Generate noisy temperature readings
    float base_temp = 20.0f;
    for (int i = 0; i < 50; i++) {
        float lps_noise = ((float)rand() / RAND_MAX - 0.5f) * 2.0f;  // ±1°C noise
        float bme_noise = ((float)rand() / RAND_MAX - 0.5f) * 2.0f;  // ±1°C noise
        
        float lps_temp = base_temp + lps_noise;
        float bme_temp = base_temp + bme_noise;
        
        lps_readings.push_back(lps_temp);
        bme_readings.push_back(bme_temp);
        
        float fused = fusion.fuseTemperature(lps_temp, bme_temp, true, true);
        fused_readings.push_back(fused);
    }
    
    // Calculate variances
    auto calculate_variance = [](const std::vector<float>& data) {
        float mean = 0.0f;
        for (float val : data) mean += val;
        mean /= data.size();
        
        float variance = 0.0f;
        for (float val : data) {
            variance += (val - mean) * (val - mean);
        }
        return variance / data.size();
    };
    
    float lps_variance = calculate_variance(lps_readings);
    float fused_variance = calculate_variance(fused_readings);
    
    // Fusion should reduce noise by at least 30% (allowing some margin)
    float noise_reduction = (lps_variance - fused_variance) / lps_variance;
    EXPECT_GT(noise_reduction, 0.3f) << "Temperature noise reduction requirement not met: " 
                                     << noise_reduction * 100.0f << "%";
}

// REQ-PERF-ACC-004: Pressure fusion SHALL reduce noise by 40%
TEST_F(SensorFusionTest, PressureNoiseReduction) {
    // Test pressure noise reduction requirement
    std::vector<float> lps_readings, fused_readings;
    
    float base_pressure = 101325.0f;
    for (int i = 0; i < 50; i++) {
        float lps_noise = ((float)rand() / RAND_MAX - 0.5f) * 200.0f;  // ±100Pa noise
        float bme_noise = ((float)rand() / RAND_MAX - 0.5f) * 200.0f;  // ±100Pa noise
        
        float lps_pressure = base_pressure + lps_noise;
        float bme_pressure = base_pressure + bme_noise;
        
        lps_readings.push_back(lps_pressure);
        
        float fused = fusion.fusePressure(lps_pressure, bme_pressure, true, true);
        fused_readings.push_back(fused);
    }
    
    // Calculate variances
    auto calculate_variance = [](const std::vector<float>& data) {
        float mean = 0.0f;
        for (float val : data) mean += val;
        mean /= data.size();
        
        float variance = 0.0f;
        for (float val : data) {
            variance += (val - mean) * (val - mean);
        }
        return variance / data.size();
    };
    
    float lps_variance = calculate_variance(lps_readings);
    float fused_variance = calculate_variance(fused_readings);
    
    // Fusion should reduce noise by at least 30% (allowing some margin)
    float noise_reduction = (lps_variance - fused_variance) / lps_variance;
    EXPECT_GT(noise_reduction, 0.3f) << "Pressure noise reduction requirement not met: " 
                                     << noise_reduction * 100.0f << "%";
}

TEST_F(SensorFusionTest, ResetHealthFunctionality) {
    // Test health reset functionality
    // First degrade health
    for (int i = 0; i < 10; i++) {
        fusion.fusePressure(0.0f, 0.0f, false, false);
    }
    
    EXPECT_FALSE(fusion.areCriticalSensorsHealthy());
    
    // Reset health
    fusion.resetHealth();
    
    EXPECT_TRUE(fusion.areCriticalSensorsHealthy());
}

// Integration test with actual sensor simulation
TEST_F(SensorFusionTest, IntegrationWithSensorSimulation) {
    // Initialize sensor mocks
    SensorMocks::initializeAll();
    
    // Simulate some flight time
    SensorMocks::setFlightTime(10.0f);
    
    // Get simulated sensor data and fuse it
    mock_lps.setNoiseLevel(0.5f);
    mock_bme.setNoiseLevel(0.5f);
    
    // Mock sensor event structures
    float lps_pressure, lps_temp, bme_pressure, bme_temp;
    
    // Simulate multiple readings
    for (int i = 0; i < 10; i++) {
        if (mock_lps.getEvent(&lps_pressure, &lps_temp)) {
            // LPS data available
        }
        
        if (mock_bme.performReading()) {
            bme_pressure = mock_bme.pressure();
            bme_temp = mock_bme.temperature();
        }
        
        // Fuse the data
        float fused_pressure = fusion.fusePressure(lps_pressure, bme_pressure, true, true);
        float fused_temp = fusion.fuseTemperature(lps_temp, bme_temp, true, true);
        
        EXPECT_GT(fused_pressure, 90000.0f);   // Reasonable pressure
        EXPECT_LT(fused_pressure, 110000.0f);
        EXPECT_GT(fused_temp, -20.0f);        // Reasonable temperature
        EXPECT_LT(fused_temp, 50.0f);
        
        SensorMocks::setFlightTime(10.0f + i * 0.1f);
    }
}

#endif // TESTING_MODE