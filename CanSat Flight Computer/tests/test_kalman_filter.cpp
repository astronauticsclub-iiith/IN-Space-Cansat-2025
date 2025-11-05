#include <gtest/gtest.h>
#include "mocks/arduino_mock.h"

#ifdef TESTING_MODE

// Include the header under test
#include "KalmanFilter.h"

class KalmanFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        TestUtils::resetMocks();
        filter.reset();
    }
    
    void TearDown() override {
        // Cleanup if needed
    }
    
    KalmanFilter filter;
};

// REQ-TEST-UNIT-001: Kalman filter SHALL be unit tested
TEST_F(KalmanFilterTest, InitializationTest) {
    // Test that filter initializes properly
    EXPECT_EQ(filter.getVelocity(), 0.0f);
}

TEST_F(KalmanFilterTest, FirstMeasurementInitialization) {
    // First measurement should initialize the filter
    float result = filter.updateEstimate(100.0f, 0.1f);
    EXPECT_FLOAT_EQ(result, 100.0f);
    EXPECT_FLOAT_EQ(filter.getVelocity(), 0.0f);
}

TEST_F(KalmanFilterTest, BasicFilteringTest) {
    // Test basic filtering behavior
    filter.updateEstimate(0.0f, 0.1f);  // Initialize at 0
    
    // Add measurements with noise
    float measurements[] = {1.0f, 2.1f, 2.9f, 4.2f, 4.8f};
    float last_estimate = 0.0f;
    
    for (int i = 0; i < 5; i++) {
        float estimate = filter.updateEstimate(measurements[i], 0.1f);
        
        // Filter should smooth the measurements
        EXPECT_GT(estimate, last_estimate);  // Should be increasing
        EXPECT_LE(abs(estimate - measurements[i]), 1.0f);  // Should be close to measurement
        
        last_estimate = estimate;
    }
}

TEST_F(KalmanFilterTest, VelocityEstimationTest) {
    // Test velocity estimation
    filter.updateEstimate(0.0f, 0.1f);
    filter.updateEstimate(1.0f, 0.1f);  // 1m in 0.1s = 10 m/s
    
    float velocity = filter.getVelocity();
    EXPECT_GT(velocity, 5.0f);  // Should detect positive velocity
    EXPECT_LT(velocity, 15.0f); // But not unreasonably high
}

TEST_F(KalmanFilterTest, NoiseFilteeringTest) {
    // Test noise filtering capability
    filter.updateEstimate(100.0f, 0.1f);  // Initialize
    
    // Add very noisy measurements around 100m
    std::vector<float> noisy_measurements = {
        98.0f, 102.0f, 99.5f, 101.2f, 100.8f, 99.2f, 100.3f
    };
    
    std::vector<float> estimates;
    for (float measurement : noisy_measurements) {
        estimates.push_back(filter.updateEstimate(measurement, 0.1f));
    }
    
    // Filtered estimates should be less noisy than measurements
    float estimate_variance = 0.0f, measurement_variance = 0.0f;
    float estimate_mean = 0.0f, measurement_mean = 0.0f;
    
    // Calculate means
    for (size_t i = 0; i < estimates.size(); i++) {
        estimate_mean += estimates[i];
        measurement_mean += noisy_measurements[i];
    }
    estimate_mean /= estimates.size();
    measurement_mean /= noisy_measurements.size();
    
    // Calculate variances
    for (size_t i = 0; i < estimates.size(); i++) {
        estimate_variance += (estimates[i] - estimate_mean) * (estimates[i] - estimate_mean);
        measurement_variance += (noisy_measurements[i] - measurement_mean) * (noisy_measurements[i] - measurement_mean);
    }
    estimate_variance /= estimates.size();
    measurement_variance /= noisy_measurements.size();
    
    // Filter should reduce noise
    EXPECT_LT(estimate_variance, measurement_variance);
}

TEST_F(KalmanFilterTest, ResetFunctionality) {
    // Test reset functionality
    filter.updateEstimate(50.0f, 0.1f);
    filter.updateEstimate(55.0f, 0.1f);
    
    EXPECT_NE(filter.getVelocity(), 0.0f);  // Should have some velocity
    
    filter.reset();
    
    float new_estimate = filter.updateEstimate(100.0f, 0.1f);
    EXPECT_FLOAT_EQ(new_estimate, 100.0f);   // Should initialize fresh
    EXPECT_FLOAT_EQ(filter.getVelocity(), 0.0f);  // Velocity should be reset
}

TEST_F(KalmanFilterTest, TimestepHandling) {
    // Test handling of different timesteps
    filter.updateEstimate(0.0f, 0.1f);
    
    // Very small timestep should not cause issues
    float estimate1 = filter.updateEstimate(1.0f, 0.001f);
    EXPECT_GT(estimate1, 0.0f);
    EXPECT_LT(estimate1, 1.0f);
    
    // Large timestep should reinitialize
    filter.reset();
    filter.updateEstimate(0.0f, 0.1f);
    float estimate2 = filter.updateEstimate(10.0f, 2.0f);  // Large dt
    EXPECT_FLOAT_EQ(estimate2, 10.0f);  // Should reinitialize
}

TEST_F(KalmanFilterTest, NoiseTuningTest) {
    // Test noise parameter tuning
    KalmanFilter filter1, filter2;
    
    // Set different noise parameters
    filter1.setProcessNoise(0.01f, 0.1f);   // Low process noise
    filter2.setProcessNoise(1.0f, 10.0f);   // High process noise
    
    filter1.setMeasurementNoise(1.0f);      // Moderate measurement noise
    filter2.setMeasurementNoise(0.1f);      // Low measurement noise
    
    // Initialize both filters
    filter1.updateEstimate(0.0f, 0.1f);
    filter2.updateEstimate(0.0f, 0.1f);
    
    // Add same measurements
    float measurements[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    
    for (int i = 0; i < 5; i++) {
        filter1.updateEstimate(measurements[i], 0.1f);
        filter2.updateEstimate(measurements[i], 0.1f);
    }
    
    // Filters with different parameters should give different results
    EXPECT_NE(filter1.getVelocity(), filter2.getVelocity());
}

// Performance test - REQ-PERF-ACC-001: Altitude accuracy SHALL be ±2m after filtering
TEST_F(KalmanFilterTest, AccuracyRequirementTest) {
    // Test altitude accuracy requirement
    float true_altitude = 500.0f;
    filter.updateEstimate(true_altitude, 0.1f);
    
    // Add measurements with realistic noise (±1m standard deviation)
    std::vector<float> measurements;
    for (int i = 0; i < 50; i++) {
        float noise = ((float)rand() / RAND_MAX - 0.5f) * 4.0f;  // ±2m noise
        measurements.push_back(true_altitude + noise);
    }
    
    // Process measurements
    float final_estimate = 0.0f;
    for (float measurement : measurements) {
        final_estimate = filter.updateEstimate(measurement, 0.1f);
    }
    
    // Final estimate should be within ±2m of true value
    float error = abs(final_estimate - true_altitude);
    EXPECT_LT(error, 2.0f) << "Altitude accuracy requirement not met: error = " << error << "m";
}

// Velocity accuracy test - REQ-PERF-ACC-002: Vertical velocity accuracy SHALL be ±0.5 m/s
TEST_F(KalmanFilterTest, VelocityAccuracyRequirementTest) {
    // Test constant velocity scenario
    float true_velocity = 10.0f;  // 10 m/s upward
    float dt = 0.1f;
    
    // Initialize
    filter.updateEstimate(0.0f, dt);
    
    // Simulate constant velocity with noise
    for (int i = 1; i <= 20; i++) {
        float true_altitude = true_velocity * i * dt;
        float noise = ((float)rand() / RAND_MAX - 0.5f) * 2.0f;  // ±1m noise
        filter.updateEstimate(true_altitude + noise, dt);
    }
    
    float estimated_velocity = filter.getVelocity();
    float velocity_error = abs(estimated_velocity - true_velocity);
    
    EXPECT_LT(velocity_error, 0.5f) << "Velocity accuracy requirement not met: error = " 
                                    << velocity_error << " m/s";
}

#endif // TESTING_MODE