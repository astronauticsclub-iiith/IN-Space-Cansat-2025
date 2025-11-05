#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <Preferences.h>
#include "config.h"
#include "Telemetry.h"  // Include before using TelemetryData

// Forward declarations
extern Preferences preferences;
extern volatile FlightState currentFlightState;
extern TelemetryData telemetryData;  // No need for 'struct' keyword
extern float maxAltitude;
extern bool aerobrakeDeployed;
extern float groundAltitude;

enum EventType : uint8_t {
    EVENT_STATE_CHANGE = 1,
    EVENT_APOGEE = 2,
    EVENT_DEPLOY = 3,
    EVENT_RESET = 4,
    EVENT_ERROR = 5,
    EVENT_COMMAND = 6
};

/**
 * BlackBox system for persistent state recovery after reset
 * Stores critical flight data in NVS for crash recovery
 */
class BlackBox {
private:
    static const uint32_t MAGIC_NUMBER = 0xCAFEBABE;
    static const uint8_t MAX_EVENTS = 10;
    
    struct CriticalEvent {
        EventType type;
        uint32_t timestamp;
        float data1;
        float data2;
    };
    
public:
    /**
     * Save current flight state to non-volatile storage
     * Call periodically and before critical operations
     */
    void saveState() {
        preferences.begin("CansatState", false);
        preferences.putUInt("magic", MAGIC_NUMBER);
        preferences.putUChar("state", (uint8_t)currentFlightState);
        preferences.putULong("pktCount", telemetryData.packetCount);
        preferences.putFloat("maxAlt", maxAltitude);
        preferences.putFloat("gndAlt", groundAltitude);
        preferences.putBool("deployed", aerobrakeDeployed);
        preferences.putULong("bootTime", millis());
        preferences.end();
    }
    
    /**
     * Load previous flight state after reset
     * Returns true if valid state was recovered
     */
    bool loadState() {
        preferences.begin("CansatState", true);
        
        // Check if valid state exists
        uint32_t magic = preferences.getUInt("magic", 0);
        if (magic != MAGIC_NUMBER) {
            preferences.end();
            logEvent("BlackBox: Fresh boot - no recovery data");
            return false;
        }
        
        // Recover state
        uint8_t savedState = preferences.getUChar("state", BOOT);
        telemetryData.packetCount = preferences.getULong("pktCount", 0);
        maxAltitude = preferences.getFloat("maxAlt", 0.0);
        groundAltitude = preferences.getFloat("gndAlt", 0.0);
        aerobrakeDeployed = preferences.getBool("deployed", false);
        uint32_t lastBootTime = preferences.getULong("bootTime", 0);
        
        preferences.end();
        
        // Calculate downtime
        uint32_t downtime = millis();
        
        logEvent("BlackBox: RECOVERY - State=" + String(savedState) + 
                 ", PktCount=" + String(telemetryData.packetCount) +
                 ", MaxAlt=" + String(maxAltitude, 1) + "m");
        
        // Intelligent state recovery based on flight phase
        currentFlightState = recoverFlightState((FlightState)savedState, downtime);
        
        return true;
    }
    
    /**
     * Determine correct flight state after reset
     * Uses saved state and sensor data to infer current phase
     */
    FlightState recoverFlightState(FlightState savedState, uint32_t downtime) {
        // If reset during boot/test/pad, stay there
        if (savedState <= LAUNCH_PAD) {
            return savedState;
        }
        
        // If already impacted, stay impacted
        if (savedState == IMPACT) {
            return IMPACT;
        }
        
        // If parachute was deployed, we're in descent or impacted
        if (aerobrakeDeployed) {
            // Will be refined by sensor readings in state machine
            return DESCENT;
        }
        
        // If we reached high altitude, assume we're descending
        if (maxAltitude > DEPLOY_ALTITUDE) {
            return DESCENT;
        }
        
        // Default: assume still ascending if uncertain
        return ASCENT;
    }
    
    /**
     * Log critical event to NVS
     * Stores up to MAX_EVENTS in circular buffer
     */
    void logCriticalEvent(EventType type, float data1 = 0.0, float data2 = 0.0) {
        logEvent("CRITICAL: Type=" + String(type) + 
                 ", D1=" + String(data1, 2) + 
                 ", D2=" + String(data2, 2));
        
        preferences.begin("CansatEvents", false);
        
        // Get current event index
        uint8_t eventIdx = preferences.getUChar("eventIdx", 0);
        
        // Store event
        String key = "evt" + String(eventIdx);
        CriticalEvent evt = {type, millis(), data1, data2};
        preferences.putBytes(key.c_str(), &evt, sizeof(CriticalEvent));
        
        // Update index (circular)
        eventIdx = (eventIdx + 1) % MAX_EVENTS;
        preferences.putUChar("eventIdx", eventIdx);
        
        preferences.end();
    }
    
    /**
     * Clear all persistent data (for testing)
     */
    void clearAll() {
        preferences.begin("CansatState", false);
        preferences.clear();
        preferences.end();
        
        preferences.begin("CansatEvents", false);
        preferences.clear();
        preferences.end();
        
        logEvent("BlackBox: All data cleared");
    }
    
    /**
     * Get event log for debugging
     */
    void printEventLog() {
        preferences.begin("CansatEvents", true);
        uint8_t eventIdx = preferences.getUChar("eventIdx", 0);
        
        Serial.println("\n=== BlackBox Event Log ===");
        for (uint8_t i = 0; i < MAX_EVENTS; i++) {
            String key = "evt" + String(i);
            CriticalEvent evt;
            if (preferences.getBytes(key.c_str(), &evt, sizeof(CriticalEvent)) > 0) {
                Serial.printf("[%d] Type=%d, Time=%lu, D1=%.2f, D2=%.2f\n",
                             i, evt.type, evt.timestamp, evt.data1, evt.data2);
            }
        }
        Serial.println("========================\n");
        
        preferences.end();
    }
};

#endif
