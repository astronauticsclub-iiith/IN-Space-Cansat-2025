#include "sensor_mock.h"

#ifdef TESTING_MODE

// Global flight simulation state
FlightSimState g_flight_state;

// Global mock instances
MockLPS22 mock_lps;
MockBME680 mock_bme;
MockICM20948 mock_icm;
MockTinyGPSPlus mock_gps;
MockServo mock_servo;
MockLoRa mock_lora;

void FlightSimState::update(float dt) {
    time += dt;
    
    // Simple rocket flight simulation
    if (!launched) {
        // On ground
        altitude = 0.0f;
        velocity = 0.0f;
        acceleration = 0.0f;
        return;
    }
    
    if (!apogee_reached) {
        // Boost phase (first 3 seconds) + coast
        if (time < 3.0f) {
            // Boost phase - high acceleration
            acceleration = 15.0f;  // 15 m/s²
        } else {
            // Coast phase - just gravity
            acceleration = -9.81f;
        }
        
        // Update velocity and altitude
        velocity += acceleration * dt;
        altitude += velocity * dt;
        
        // Check for apogee
        if (velocity <= 0 && altitude > 10.0f) {
            apogee_reached = true;
            parachute_deployed = true;
        }
    } else if (!landed) {
        // Descent phase
        if (parachute_deployed) {
            // Terminal velocity with parachute
            float terminal_velocity = -8.0f;  // 8 m/s down
            velocity += (terminal_velocity - velocity) * dt * 0.5f;  // Approach terminal velocity
        } else {
            // Free fall
            velocity -= 9.81f * dt;
        }
        
        altitude += velocity * dt;
        
        // Check for landing
        if (altitude <= 0.0f) {
            altitude = 0.0f;
            velocity = 0.0f;
            acceleration = 0.0f;
            landed = true;
        }
    }
    
    // Update environmental conditions based on altitude
    if (altitude > 0) {
        temperature = 15.0f - 0.0065f * altitude;  // Standard atmosphere
        pressure = 101325.0f * pow(1.0f - 0.0065f * altitude / 288.15f, 5.255f);
        
        // Simulate slight GPS drift during flight
        latitude += (std::sin(time * 0.1f) * 0.00001f);
        longitude += (std::cos(time * 0.1f) * 0.00001f);
    }
}

void FlightSimState::reset() {
    time = 0.0f;
    altitude = 0.0f;
    velocity = 0.0f;
    acceleration = 0.0f;
    launched = false;
    apogee_reached = false;
    parachute_deployed = false;
    landed = false;
    
    temperature = 20.0f;
    pressure = 101325.0f;
    humidity = 50.0f;
    
    latitude = 40.7128;
    longitude = -74.0060;
    
    accel_x = 0.0f;
    accel_y = 0.0f;
    accel_z = 1.0f;
    gyro_x = 0.0f;
    gyro_y = 0.0f;
    gyro_z = 0.0f;
}

namespace SensorMocks {
    void initializeAll() {
        mock_lps.setEnabled(true);
        mock_lps.setFailure(false);
        mock_lps.setNoiseLevel(0.1f);
        
        mock_bme.setEnabled(true);
        mock_bme.setFailure(false);
        mock_bme.setNoiseLevel(0.1f);
        
        mock_icm.setEnabled(true);
        mock_icm.setFailure(false);
        mock_icm.setNoiseLevel(0.05f);
        
        mock_gps.setEnabled(true);
        mock_gps.setFailure(false);
        
        mock_servo.setEnabled(true);
        mock_servo.setFailure(false);
    }
    
    void resetAll() {
        g_flight_state.reset();
        initializeAll();
    }
    
    void simulateFlightProfile() {
        // Simulate a complete flight profile over time
        resetAll();
        
        // Ground phase (0-5 seconds)
        for (float t = 0; t < 5.0f; t += 0.1f) {
            g_flight_state.update(0.1f);
        }
        
        // Launch
        simulateLaunch();
        
        // Flight phase (5-30 seconds)
        for (float t = 5.0f; t < 30.0f; t += 0.1f) {
            g_flight_state.update(0.1f);
        }
        
        // Continue until landing
        while (!g_flight_state.landed && g_flight_state.time < 120.0f) {
            g_flight_state.update(0.1f);
        }
    }
    
    void setFlightTime(float time_seconds) {
        g_flight_state.time = time_seconds;
        g_flight_state.update(0.0f);  // Update state based on time
    }
    
    void simulateLaunch() {
        g_flight_state.launched = true;
        std::cout << "[SIM] Launch detected at t=" << g_flight_state.time << "s" << std::endl;
    }
    
    void simulateApogee() {
        g_flight_state.apogee_reached = true;
        g_flight_state.velocity = 0.0f;
        std::cout << "[SIM] Apogee reached at t=" << g_flight_state.time 
                  << "s, altitude=" << g_flight_state.altitude << "m" << std::endl;
    }
    
    void simulateLanding() {
        g_flight_state.landed = true;
        g_flight_state.altitude = 0.0f;
        g_flight_state.velocity = 0.0f;
        std::cout << "[SIM] Landing at t=" << g_flight_state.time << "s" << std::endl;
    }
    
    void setSensorFailure(const String& sensor, bool failed) {
        if (sensor == "lps22" || sensor == "lps") {
            mock_lps.setFailure(failed);
        } else if (sensor == "bme680" || sensor == "bme") {
            mock_bme.setFailure(failed);
        } else if (sensor == "icm20948" || sensor == "icm" || sensor == "imu") {
            mock_icm.setFailure(failed);
        } else if (sensor == "gps") {
            mock_gps.setFailure(failed);
        } else if (sensor == "servo") {
            mock_servo.setFailure(failed);
        }
        
        if (failed) {
            std::cout << "[SIM] " << sensor.c_str() << " sensor failure simulated" << std::endl;
        } else {
            std::cout << "[SIM] " << sensor.c_str() << " sensor restored" << std::endl;
        }
    }
    
    void setSensorNoise(const String& sensor, float noise_level) {
        if (sensor == "lps22" || sensor == "lps") {
            mock_lps.setNoiseLevel(noise_level);
        } else if (sensor == "bme680" || sensor == "bme") {
            mock_bme.setNoiseLevel(noise_level);
        } else if (sensor == "icm20948" || sensor == "icm" || sensor == "imu") {
            mock_icm.setNoiseLevel(noise_level);
        }
        
        std::cout << "[SIM] " << sensor.c_str() << " noise level set to " << noise_level << std::endl;
    }
}

#endif // TESTING_MODE