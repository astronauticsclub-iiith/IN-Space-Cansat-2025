#include <gtest/gtest.h>
#include "mocks/arduino_mock.h"

#ifdef TESTING_MODE

#include "DataBuffer.h"

class DataBufferTest : public ::testing::Test {
protected:
    void SetUp() override {
        TestUtils::resetMocks();
    }
    
    void TearDown() override {
        // Cleanup if needed
    }
};

// Basic functionality tests
TEST_F(DataBufferTest, InitialState) {
    DataBuffer buffer;
    
    EXPECT_TRUE(buffer.isEmpty());
    EXPECT_FALSE(buffer.isFull());
    EXPECT_EQ(buffer.getCount(), 0);
}

TEST_F(DataBufferTest, AddSingleEntry) {
    DataBuffer buffer;
    
    bool success = buffer.add("Test entry");
    EXPECT_TRUE(success);
    EXPECT_FALSE(buffer.isEmpty());
    EXPECT_EQ(buffer.getCount(), 1);
}

TEST_F(DataBufferTest, AddAndRetrieve) {
    DataBuffer buffer;
    
    String test_data = "Hello, World!";
    buffer.add(test_data);
    
    String retrieved = buffer.get();
    EXPECT_EQ(retrieved.c_str(), test_data.c_str());
    EXPECT_TRUE(buffer.isEmpty());
}

TEST_F(DataBufferTest, MultipleEntries) {
    DataBuffer buffer;
    
    std::vector<String> test_entries = {
        "Entry 1",
        "Entry 2", 
        "Entry 3"
    };
    
    // Add entries
    for (const auto& entry : test_entries) {
        EXPECT_TRUE(buffer.add(entry));
    }
    
    EXPECT_EQ(buffer.getCount(), 3);
    
    // Retrieve in FIFO order
    for (const auto& expected : test_entries) {
        String retrieved = buffer.get();
        EXPECT_EQ(retrieved.c_str(), expected.c_str());
    }
    
    EXPECT_TRUE(buffer.isEmpty());
}

TEST_F(DataBufferTest, CircularBehavior) {
    DataBuffer buffer;
    
    // Add some entries
    buffer.add("First");
    buffer.add("Second");
    
    // Get one entry
    String first = buffer.get();
    EXPECT_EQ(first.c_str(), "First");
    
    // Add another entry
    buffer.add("Third");
    
    // Should get Second, then Third
    EXPECT_EQ(buffer.get().c_str(), "Second");
    EXPECT_EQ(buffer.get().c_str(), "Third");
    EXPECT_TRUE(buffer.isEmpty());
}

// REQ-PERF-MEM-003: Telemetry buffer SHALL hold minimum 20 packets
TEST_F(DataBufferTest, MinimumCapacityRequirement) {
    DataBuffer buffer;
    
    // Fill buffer to minimum required capacity
    for (int i = 0; i < 20; i++) {
        String entry = "Packet " + String(i);
        bool success = buffer.add(entry);
        EXPECT_TRUE(success) << "Failed to add entry " << i << " (buffer should hold at least 20)";
    }
    
    EXPECT_GE(buffer.getCount(), 20);
    EXPECT_FALSE(buffer.isEmpty());
}

TEST_F(DataBufferTest, OverflowBehavior) {
    DataBuffer buffer;
    
    // Fill buffer beyond capacity
    int entries_added = 0;
    for (int i = 0; i < 25; i++) {  // Try to add more than buffer size
        String entry = "Entry " + String(i);
        if (buffer.add(entry)) {
            entries_added++;
        } else {
            break;  // Buffer full
        }
    }
    
    EXPECT_GT(entries_added, 0);
    EXPECT_TRUE(buffer.isFull());
}

TEST_F(DataBufferTest, EmptyBufferRetrieval) {
    DataBuffer buffer;
    
    String result = buffer.get();
    EXPECT_EQ(result.length(), 0);  // Should return empty string
}

TEST_F(DataBufferTest, ClearFunctionality) {
    DataBuffer buffer;
    
    // Add some entries
    buffer.add("Entry 1");
    buffer.add("Entry 2");
    buffer.add("Entry 3");
    
    EXPECT_FALSE(buffer.isEmpty());
    EXPECT_EQ(buffer.getCount(), 3);
    
    // Clear buffer
    buffer.clear();
    
    EXPECT_TRUE(buffer.isEmpty());
    EXPECT_EQ(buffer.getCount(), 0);
    EXPECT_FALSE(buffer.isFull());
}

// Thread safety simulation test
TEST_F(DataBufferTest, ThreadSafetySimulation) {
    DataBuffer buffer;
    
    // Simulate concurrent access by rapidly adding and removing entries
    const int iterations = 100;
    
    for (int i = 0; i < iterations; i++) {
        // Add entry
        String entry = "Concurrent " + String(i);
        buffer.add(entry);
        
        // Sometimes retrieve immediately
        if (i % 3 == 0 && !buffer.isEmpty()) {
            String retrieved = buffer.get();
            EXPECT_GT(retrieved.length(), 0);
        }
    }
    
    // Buffer should still be in valid state
    EXPECT_GE(buffer.getCount(), 0);
    EXPECT_LE(buffer.getCount(), 20);  // Assuming BUFFER_SIZE is 20
}

TEST_F(DataBufferTest, LongStringHandling) {
    DataBuffer buffer;
    
    // Create a very long string (near buffer limit)
    String long_string = "";
    for (int i = 0; i < 300; i++) {  // Create 300 character string
        long_string += "A";
    }
    
    bool success = buffer.add(long_string);
    EXPECT_TRUE(success);
    
    String retrieved = buffer.get();
    // Should be truncated to fit buffer (MAX_ENTRY_SIZE - 1)
    EXPECT_LE(retrieved.length(), 399);  // MAX_ENTRY_SIZE - 1
    
    // Should still contain the beginning of the string
    EXPECT_TRUE(retrieved.c_str()[0] == 'A');
}

TEST_F(DataBufferTest, TelemetryDataFormatTest) {
    DataBuffer buffer;
    
    // Test with realistic telemetry data format
    String telemetry = "2024ASI-CANSAT-001,123.5,456,789.1,101325,20.5,3.85,"
                      "12:34:56,40.7128,-74.0060,102.3,8,"
                      "0.12;0.34;9.81,1.2;-0.5;0.8,3,BATT:85%;VS:5.2;HUM:45;GAS:98.5";
    
    bool success = buffer.add(telemetry);
    EXPECT_TRUE(success);
    
    String retrieved = buffer.get();
    EXPECT_EQ(retrieved.c_str(), telemetry.c_str());
}

// Performance test
TEST_F(DataBufferTest, PerformanceTest) {
    DataBuffer buffer;
    
    auto start_time = millis();
    
    // Rapid add/get operations
    const int operations = 1000;
    for (int i = 0; i < operations; i++) {
        String entry = "Performance test " + String(i);
        buffer.add(entry);
        
        if (!buffer.isEmpty()) {
            buffer.get();
        }
    }
    
    auto end_time = millis();
    auto elapsed = end_time - start_time;
    
    // Should complete operations reasonably quickly (less than 1 second)
    EXPECT_LT(elapsed, 1000) << "Buffer operations took too long: " << elapsed << "ms";
}

// Memory test - ensuring no memory leaks in repeated operations
TEST_F(DataBufferTest, MemoryStabilityTest) {
    DataBuffer buffer;
    
    // Perform many add/remove cycles
    for (int cycle = 0; cycle < 100; cycle++) {
        // Fill buffer
        for (int i = 0; i < 15; i++) {
            String entry = "Cycle " + String(cycle) + " Entry " + String(i);
            buffer.add(entry);
        }
        
        // Empty buffer
        while (!buffer.isEmpty()) {
            buffer.get();
        }
        
        // Verify clean state
        EXPECT_TRUE(buffer.isEmpty());
        EXPECT_EQ(buffer.getCount(), 0);
        EXPECT_FALSE(buffer.isFull());
    }
}

// Edge case: Null or empty strings
TEST_F(DataBufferTest, EdgeCaseHandling) {
    DataBuffer buffer;
    
    // Empty string
    bool success1 = buffer.add("");
    EXPECT_TRUE(success1);
    
    String retrieved = buffer.get();
    EXPECT_EQ(retrieved.length(), 0);
    
    // String with special characters
    String special = "Test\nWith\tSpecial\rCharacters!@#$%^&*()";
    bool success2 = buffer.add(special);
    EXPECT_TRUE(success2);
    
    String retrieved2 = buffer.get();
    EXPECT_EQ(retrieved2.c_str(), special.c_str());
}

#endif // TESTING_MODE