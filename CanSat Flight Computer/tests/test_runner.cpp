#include <gtest/gtest.h>
#include <iostream>

// Main test runner for CanSat Flight Software
// This file orchestrates all unit and integration tests

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "      CANSAT FLIGHT SOFTWARE TESTS     \n";
    std::cout << "========================================\n";
    std::cout << "Running comprehensive test suite...\n";
    std::cout << "Coverage: All CanSat requirements\n";
    std::cout << "Framework: Google Test + Mock\n";
    std::cout << "========================================\n\n";
    
    // Run all tests
    int result = RUN_ALL_TESTS();
    
    std::cout << "\n========================================\n";
    if (result == 0) {
        std::cout << "✅ ALL TESTS PASSED - SYSTEM READY\n";
    } else {
        std::cout << "❌ SOME TESTS FAILED - CHECK OUTPUT\n";
    }
    std::cout << "========================================\n\n";
    
    return result;
}