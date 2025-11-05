#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include <Arduino.h>

/**
 * Sensor Fusion Class for CanSat
 * Combines multiple sensor readings for improved accuracy and reliability
 * Implements weighted averaging, outlier rejection, and health monitoring
 */
class SensorFusion {
private:
    // Health tracking for sensors
    struct SensorHealth {
        uint8_t consecutiveFailures;
        uint32_t lastSuccessTime;
        float trustFactor;
        bool isHealthy;
        
        SensorHealth() : consecutiveFailures(0), lastSuccessTime(0), 
                        trustFactor(1.0f), isHealthy(true) {}
    };
    
    SensorHealth lpsHealth;
    SensorHealth bmeHealth;
    SensorHealth gpsHealth;
    SensorHealth sht41Health;
    
    // Fusion parameters
    static constexpr float OUTLIER_THRESHOLD = 3.0f;  // Standard deviations
    static constexpr uint8_t MAX_FAILURES = 5;
    static constexpr uint32_t HEALTH_TIMEOUT_MS = 10000;  // 10 seconds
    
    // Moving averages for outlier detection
    float pressureHistory[5] = {0};
    float temperatureHistory[5] = {0};
    uint8_t historyIndex = 0;
    bool historyFilled = false;
    
    /**
     * Update sensor health based on success/failure
     */
    void updateHealth(SensorHealth& health, bool success) {
        if (success) {
            health.consecutiveFailures = 0;
            health.lastSuccessTime = millis();
            health.isHealthy = true;
            health.trustFactor = min(1.0f, health.trustFactor + 0.1f);
        } else {
            health.consecutiveFailures++;
            health.trustFactor = max(0.1f, health.trustFactor - 0.2f);
            
            if (health.consecutiveFailures >= MAX_FAILURES ||
                (millis() - health.lastSuccessTime) > HEALTH_TIMEOUT_MS) {
                health.isHealthy = false;
            }
        }
    }
    
    /**
     * Detect outliers using simple statistical method
     */
    bool isOutlier(float value, float* history, int size) {
        if (!historyFilled && historyIndex < 3) return false;
        
        float mean = 0;
        float variance = 0;
        int validCount = min(size, (int)historyIndex);
        
        // Calculate mean
        for (int i = 0; i < validCount; i++) {
            mean += history[i];
        }
        mean /= validCount;
        
        // Calculate variance
        for (int i = 0; i < validCount; i++) {
            variance += (history[i] - mean) * (history[i] - mean);
        }
        variance /= validCount;
        
        float stdDev = sqrt(variance);
        return abs(value - mean) > (OUTLIER_THRESHOLD * stdDev);
    }
    
    /**
     * Add value to history for outlier detection
     */
    void addToHistory(float* history, float value) {
        history[historyIndex] = value;
        historyIndex = (historyIndex + 1) % 5;
        if (historyIndex == 0) historyFilled = true;
    }

public:
    SensorFusion() {}
    
    /**
     * Fuse pressure readings from LPS22 and BME680
     * @param lps_pressure LPS22 pressure in Pa
     * @param bme_pressure BME680 pressure in Pa
     * @param lps_valid Is LPS reading valid
     * @param bme_valid Is BME reading valid
     * @return Fused pressure in Pa
     */
    float fusePressure(float lps_pressure, float bme_pressure, bool lps_valid, bool bme_valid) {
        updateHealth(lpsHealth, lps_valid);
        updateHealth(bmeHealth, bme_valid);
        
        // If both invalid, return last known good value or zero
        if (!lps_valid && !bme_valid) {
            logEvent("WARNING: Both pressure sensors failed!");
            return 0.0f;
        }
        
        // Single sensor mode
        if (!lps_valid && bme_valid) {
            if (!isOutlier(bme_pressure, pressureHistory, 5)) {
                addToHistory(pressureHistory, bme_pressure);
                return bme_pressure;
            }
            return 0.0f;
        }
        
        if (lps_valid && !bme_valid) {
            if (!isOutlier(lps_pressure, pressureHistory, 5)) {
                addToHistory(pressureHistory, lps_pressure);
                return lps_pressure;
            }
            return 0.0f;
        }
        
        // Both sensors valid - weighted fusion
        float lpsWeight = lpsHealth.trustFactor * (lpsHealth.isHealthy ? 1.0f : 0.3f);
        float bmeWeight = bmeHealth.trustFactor * (bmeHealth.isHealthy ? 1.0f : 0.3f);
        
        // Check for outliers
        bool lpsOutlier = isOutlier(lps_pressure, pressureHistory, 5);
        bool bmeOutlier = isOutlier(bme_pressure, pressureHistory, 5);
        
        if (lpsOutlier) lpsWeight *= 0.1f;
        if (bmeOutlier) bmeWeight *= 0.1f;
        
        float totalWeight = lpsWeight + bmeWeight;
        if (totalWeight > 0) {
            float fusedPressure = (lps_pressure * lpsWeight + bme_pressure * bmeWeight) / totalWeight;
            addToHistory(pressureHistory, fusedPressure);
            return fusedPressure;
        }
        
        return 0.0f;
    }
    
    /**
     * Fuse temperature readings from LPS22, BME680, and SHT41
     * @param lps_temp LPS22 temperature in °C
     * @param bme_temp BME680 temperature in °C
     * @param sht_temp SHT41 temperature in °C (optional)
     * @param lps_valid Is LPS reading valid
     * @param bme_valid Is BME reading valid
     * @param sht_valid Is SHT reading valid
     * @return Fused temperature in °C
     */
    float fuseTemperature(float lps_temp, float bme_temp, bool lps_valid, bool bme_valid, 
                         float sht_temp = 0.0f, bool sht_valid = false) {
        updateHealth(lpsHealth, lps_valid);
        updateHealth(bmeHealth, bme_valid);
        updateHealth(sht41Health, sht_valid);
        
        float fusedTemp = 0.0f;
        float totalWeight = 0.0f;
        
        // LPS22 contribution
        if (lps_valid && !isOutlier(lps_temp, temperatureHistory, 5)) {
            float weight = lpsHealth.trustFactor * 0.8f;  // LPS22 is primary barometer
            fusedTemp += lps_temp * weight;
            totalWeight += weight;
        }
        
        // BME680 contribution
        if (bme_valid && !isOutlier(bme_temp, temperatureHistory, 5)) {
            float weight = bmeHealth.trustFactor * 1.0f;  // BME680 good for temperature
            fusedTemp += bme_temp * weight;
            totalWeight += weight;
        }
        
        // SHT41 contribution (if available)
        if (sht_valid && !isOutlier(sht_temp, temperatureHistory, 5)) {
            float weight = sht41Health.trustFactor * 1.2f;  // SHT41 most accurate for temp
            fusedTemp += sht_temp * weight;
            totalWeight += weight;
        }
        
        if (totalWeight > 0) {
            float result = fusedTemp / totalWeight;
            addToHistory(temperatureHistory, result);
            return result;
        }
        
        logEvent("WARNING: All temperature sensors failed!");
        return 0.0f;
    }
    
    /**
     * Cross-validate barometric altitude with GPS altitude
     * @param baro_altitude Barometric altitude in meters
     * @param gps_altitude GPS altitude in meters  
     * @param gps_valid Is GPS reading valid
     * @return Cross-validated altitude
     */
    float fuseAltitude(float baro_altitude, float gps_altitude, bool gps_valid) {
        updateHealth(gpsHealth, gps_valid);
        
        // Trust barometric altitude primarily
        if (!gps_valid || !gpsHealth.isHealthy) {
            return baro_altitude;
        }
        
        // Check for large discrepancy (more than 20m difference)
        float difference = abs(baro_altitude - gps_altitude);
        if (difference > 20.0f) {
            logEvent("WARNING: Large altitude discrepancy: Baro=" + String(baro_altitude, 1) + 
                    "m, GPS=" + String(gps_altitude, 1) + "m");
            
            // If GPS is very reliable and difference is huge, blend them
            if (gpsHealth.trustFactor > 0.8f && difference > 50.0f) {
                return (baro_altitude * 0.7f + gps_altitude * 0.3f);
            }
        }
        
        // Small corrections using GPS (within 5m)
        if (difference < 5.0f && gpsHealth.trustFactor > 0.5f) {
            return (baro_altitude * 0.9f + gps_altitude * 0.1f);
        }
        
        return baro_altitude;
    }
    
    /**
     * Get sensor health status
     */
    String getHealthStatus() {
        String status = "Health: ";
        status += "LPS=" + String(lpsHealth.trustFactor, 2) + 
                  (lpsHealth.isHealthy ? "✓" : "✗") + ", ";
        status += "BME=" + String(bmeHealth.trustFactor, 2) + 
                  (bmeHealth.isHealthy ? "✓" : "✗") + ", ";
        status += "GPS=" + String(gpsHealth.trustFactor, 2) + 
                  (gpsHealth.isHealthy ? "✓" : "✗");
        return status;
    }
    
    /**
     * Check if critical sensors are healthy
     */
    bool areCriticalSensorsHealthy() {
        return (lpsHealth.isHealthy || bmeHealth.isHealthy);  // At least one pressure sensor
    }
    
    /**
     * Reset all sensor health (for testing)
     */
    void resetHealth() {
        lpsHealth = SensorHealth();
        bmeHealth = SensorHealth();
        gpsHealth = SensorHealth();
        sht41Health = SensorHealth();
        historyIndex = 0;
        historyFilled = false;
    }
};

#endif