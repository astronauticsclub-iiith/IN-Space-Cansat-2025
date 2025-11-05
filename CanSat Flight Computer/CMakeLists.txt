cmake_minimum_required(VERSION 3.16)

# Project Configuration
project(CANSAT_ONBOARD_FINAL
    VERSION 1.0.0
    DESCRIPTION "CanSat Flight Software with Comprehensive Testing"
    LANGUAGES CXX C
)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build configuration
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
)

# Find required packages
find_package(Threads REQUIRED)

# Source files (for testing - excluding Arduino-specific code)
set(CANSAT_SOURCES
    # Core algorithm files that can be tested
    src/KalmanFilter.h
    src/SensorFusion.h
    src/DataBuffer.h
    src/BlackBox.h
    src/config.h
)

# Create a library for testable components
add_library(cansat_core INTERFACE)
target_include_directories(cansat_core INTERFACE src/)

# Option to build tests
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Coverage reporting
option(ENABLE_COVERAGE "Enable code coverage" OFF)
if(ENABLE_COVERAGE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()

# Documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_SOURCE_DIR}/Doxyfile.in ${CMAKE_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(docs
        ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif()

# Custom targets for Arduino build (informational)
add_custom_target(arduino_info
    COMMAND echo "To build for Arduino ESP32:"
    COMMAND echo "1. Install Arduino IDE"
    COMMAND echo "2. Install ESP32 board package"
    COMMAND echo "3. Install required libraries (see README.md)"
    COMMAND echo "4. Open src/main.ino in Arduino IDE"
    COMMAND echo "5. Select ESP32 Dev Module board"
    COMMAND echo "6. Configure: CPU 240MHz, Flash 4MB, Upload 921600"
    COMMAND echo "7. Compile and upload"
    VERBATIM
)

# Print configuration
message(STATUS "CanSat Flight Software Configuration:")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Tests: ${BUILD_TESTS}")
message(STATUS "  Coverage: ${ENABLE_COVERAGE}")
message(STATUS "  Source Dir: ${CMAKE_SOURCE_DIR}")
message(STATUS "  Binary Dir: ${CMAKE_BINARY_DIR}")