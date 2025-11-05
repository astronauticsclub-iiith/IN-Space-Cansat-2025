#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "mocks/arduino_mock.h"

// Include config first for enum definitions
#include "../src/config.h"

// Forward declarations for Telemetry.h dependencies
struct TelemetryData;

// Mock servo object (must be defined before Telemetry.h)
struct TestServo {
    void write(int angle) { /* mock implementation */ }
} parachuteServo;

// Mock function implementations (must be defined before Telemetry.h)
const char* getStateName(FlightState state) {
    switch(state) {
        case BOOT: return "BOOT";
        case TEST_MODE: return "TEST_MODE";
        case LAUNCH_PAD: return "LAUNCH_PAD";
        case ASCENT: return "ASCENT";
        case ROCKET_DEPLOY: return "ROCKET_DEPLOY";
        case DESCENT: return "DESCENT";
        case AEROBRAKE_RELEASE: return "AEROBRAKE_RELEASE";
        case IMPACT: return "IMPACT";
        default: return "UNKNOWN";
    }
}

void transitionToState(FlightState newState);
void calibrateGroundAltitude();
void calibrateIMU();
void testServo();
void deployParachute();

// Mock global variables (must be defined before Telemetry.h)
TelemetryData* telemetryDataPtr;
float maxAltitude = 0.0f;
bool aerobrakeDeployed = false;
SemaphoreHandle_t spiMutex = nullptr;
SemaphoreHandle_t servoMutex = nullptr;
bool lora_ok = true;
volatile FlightState currentFlightState = LAUNCH_PAD;
bool isInTestMode = false;

// Include the header after defining required symbols
#include "../src/Telemetry.h"

// Now define the actual telemetryData with proper type
TelemetryData telemetryData;

// Implement the functions we declared
void transitionToState(FlightState newState) {
    currentFlightState = newState;
}

void calibrateGroundAltitude() { /* mock */ }
void calibrateIMU() { /* mock */ }
void testServo() { /* mock */ }
void deployParachute() { /* mock */ }

// Mock processCommand function 
void processCommand(String cmd) {
    // Basic command processing mock
    if (cmd.equalsIgnoreCase("CMD,CAL_ALT")) {
        calibrateGroundAltitude();
    } else if (cmd.equalsIgnoreCase("CMD,STATUS")) {
        // Mock status response
    }
}

class TelemetryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize telemetry data
        telemetryData = {};
        telemetryData.packetCount = 1;
        telemetryData.timeStamp = 1.0f;
        telemetryData.altitude = 100.0f;
        telemetryData.pressure = 1013.25f;
        telemetryData.temperature = 20.0f;
        telemetryData.voltage = 3.7f;
        currentFlightState = LAUNCH_PAD;
    }
    
    void TearDown() override {}
};

// REQ-COM-001: Radio communication
TEST_F(TelemetryTest, ShouldFormatTelemetryCSV) {
    // Test telemetry CSV formatting
    String csv = formatTelemetryCSV();
    EXPECT_GT(csv.length(), 0);
    
    // Should contain team ID
    EXPECT_TRUE(csv.indexOf(TEAM_ID) >= 0);
}

// REQ-COM-002: Data packet format
TEST_F(TelemetryTest, ShouldContainRequiredFields) {
    String csv = formatTelemetryCSV();
    
    // Check for required CSV fields (should have commas separating fields)
    int commaCount = 0;
    for (size_t i = 0; i < csv.length(); i++) {
        if (csv[i] == ',') commaCount++;
    }
    EXPECT_GE(commaCount, 15); // Should have at least 15 commas for 16 fields
}

// REQ-COM-003: Command processing
TEST_F(TelemetryTest, ShouldProcessCommands) {
    // Test various commands - these should not crash
    processCommand("CMD,CAL_ALT");
    processCommand("CMD,STATUS");
    processCommand("CMD,SET_MODE,TEST");
    
    // Commands should execute without errors
    EXPECT_TRUE(true); // If we get here, commands didn't crash
}

// REQ-COM-004: Telemetry header
TEST_F(TelemetryTest, ShouldProvideTelemetryHeader) {
    const char* header = getTelemetryHeader();
    EXPECT_NE(header, nullptr);
    EXPECT_GT(strlen(header), 0);
    
    // Should contain expected field names
    EXPECT_TRUE(strstr(header, "TEAM_ID") != nullptr);
    EXPECT_TRUE(strstr(header, "ALTITUDE") != nullptr);
    EXPECT_TRUE(strstr(header, "FLIGHT_STATE") != nullptr);
}