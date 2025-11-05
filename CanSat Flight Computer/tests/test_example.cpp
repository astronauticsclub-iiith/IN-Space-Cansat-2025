#include <gtest/gtest.h>

// Include the header file with the code you want to test.
// You might need to adjust the path.
#include "../src/config.h" 

// Simple function to test (pretend this exists in your code)
bool isValidFlightState(int state) {
    return state >= BOOT && state <= IMPACT;
}

// Test case 1: Check for valid flight states
TEST(FlightStateTest, HandlesValidStates) {
    EXPECT_TRUE(isValidFlightState(BOOT));
    EXPECT_TRUE(isValidFlightState(ASCENT));
    EXPECT_TRUE(isValidFlightState(IMPACT));
}

// Test case 2: Check for invalid (out-of-bounds) states
TEST(FlightStateTest, HandlesInvalidStates) {
    EXPECT_FALSE(isValidFlightState(-1));
    EXPECT_FALSE(isValidFlightState(100)); // Assuming 100 is not a valid state
}