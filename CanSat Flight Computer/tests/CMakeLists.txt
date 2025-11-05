# Test Configuration for CanSat Flight Software
cmake_minimum_required(VERSION 3.16)

# Find GTest
find_package(GTest REQUIRED)
if(NOT GTest_FOUND)
    # Fallback: fetch GTest from GitHub
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
    )
    FetchContent_MakeAvailable(googletest)
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/tests/mocks
    ${GTEST_INCLUDE_DIRS}
)

# Test sources
set(TEST_SOURCES
    test_kalman_filter.cpp
    test_sensor_fusion.cpp
    test_data_buffer.cpp
    test_blackbox.cpp
    test_telemetry.cpp
    test_state_machine.cpp
    test_sensors.cpp
    test_integration.cpp
    test_performance.cpp
    test_safety.cpp
    test_system_simulation.cpp
    mocks/arduino_mock.cpp
    mocks/sensor_mock.cpp
)

# Create test executable
add_executable(cansat_tests
    ${TEST_SOURCES}
    test_runner.cpp
)

# Link libraries
target_link_libraries(cansat_tests
    GTest::GTest
    GTest::Main
    cansat_core
    pthread
)

# Test compiler flags
target_compile_definitions(cansat_tests PRIVATE
    -DTESTING_MODE=1
    -DARDUINO_MOCK=1
)

# Individual test executables for better organization
function(create_test_executable test_name source_file)
    add_executable(${test_name} ${source_file} mocks/arduino_mock.cpp)
    target_link_libraries(${test_name} GTest::GTest GTest::Main cansat_core pthread)
    target_compile_definitions(${test_name} PRIVATE -DTESTING_MODE=1 -DARDUINO_MOCK=1)
    gtest_discover_tests(${test_name})
endfunction()

# Create individual test executables
create_test_executable(test_kalman_filter test_kalman_filter.cpp)
create_test_executable(test_sensor_fusion test_sensor_fusion.cpp)
create_test_executable(test_data_buffer test_data_buffer.cpp)
create_test_executable(test_blackbox test_blackbox.cpp)
create_test_executable(test_telemetry test_telemetry.cpp)
create_test_executable(test_state_machine test_state_machine.cpp)
create_test_executable(test_sensors test_sensors.cpp)
create_test_executable(test_integration test_integration.cpp)
create_test_executable(test_performance test_performance.cpp)
create_test_executable(test_safety test_safety.cpp)
create_test_executable(test_system_simulation test_system_simulation.cpp)

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(cansat_tests)

# Custom test targets
add_custom_target(test_unit
    COMMAND test_kalman_filter
    COMMAND test_sensor_fusion
    COMMAND test_data_buffer
    COMMAND test_blackbox
    COMMAND test_telemetry
    DEPENDS test_kalman_filter test_sensor_fusion test_data_buffer test_blackbox test_telemetry
    COMMENT "Running unit tests"
)

add_custom_target(test_integration_only
    COMMAND test_integration
    DEPENDS test_integration
    COMMENT "Running integration tests"
)

add_custom_target(test_performance_only
    COMMAND test_performance
    DEPENDS test_performance
    COMMENT "Running performance tests"
)

add_custom_target(test_safety_only
    COMMAND test_safety
    DEPENDS test_safety
    COMMENT "Running safety tests"
)

add_custom_target(test_system_only
    COMMAND test_system_simulation
    DEPENDS test_system_simulation
    COMMENT "Running system simulation tests"
)

# Coverage target
if(ENABLE_COVERAGE)
    add_custom_target(coverage
        COMMAND lcov --capture --directory . --output-file coverage.info
        COMMAND lcov --remove coverage.info '/usr/*' --output-file coverage.info
        COMMAND lcov --remove coverage.info '*/tests/*' --output-file coverage.info
        COMMAND lcov --remove coverage.info '*/mocks/*' --output-file coverage.info
        COMMAND genhtml coverage.info --output-directory coverage_html
        DEPENDS cansat_tests
        COMMENT "Generating code coverage report"
    )
endif()

# Test data directory
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/test_data)

# Copy test data files
configure_file(
    ${CMAKE_SOURCE_DIR}/tests/data/sample_telemetry.csv
    ${CMAKE_BINARY_DIR}/test_data/sample_telemetry.csv
    COPYONLY
)

message(STATUS "Test configuration complete:")
message(STATUS "  GTest found: ${GTest_FOUND}")
message(STATUS "  Test executables: 11")
message(STATUS "  Coverage enabled: ${ENABLE_COVERAGE}")