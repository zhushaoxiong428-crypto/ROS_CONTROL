# Host-side unit tests for hardware-independent firmware logic.
#   cmake -S tests -B build-tests && cmake --build build-tests && ctest --test-dir build-tests
cmake_minimum_required(VERSION 3.16)
project(leap_ros_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Tests use assert(); keep it active in every build type.
add_compile_options(-Wall -Wextra -Werror -UNDEBUG)

set(FIRMWARE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../main)

enable_testing()

add_executable(differential_kinematics_test
    differential_kinematics_test.cpp
    ${FIRMWARE_DIR}/control/differential_kinematics.cpp)
target_include_directories(differential_kinematics_test PRIVATE
    ${FIRMWARE_DIR}/config ${FIRMWARE_DIR}/control)
add_test(NAME differential_kinematics COMMAND differential_kinematics_test)

add_executable(command_watchdog_test command_watchdog_test.cpp)
target_include_directories(command_watchdog_test PRIVATE ${FIRMWARE_DIR}/control)
add_test(NAME command_watchdog COMMAND command_watchdog_test)

add_executable(agent_discovery_test agent_discovery_test.cpp)
target_include_directories(agent_discovery_test PRIVATE ${FIRMWARE_DIR}/network)
add_test(NAME agent_discovery COMMAND agent_discovery_test)

add_executable(cycle_timing_stats_test cycle_timing_stats_test.cpp)
target_include_directories(cycle_timing_stats_test PRIVATE ${FIRMWARE_DIR}/control)
add_test(NAME cycle_timing_stats COMMAND cycle_timing_stats_test)

add_executable(wheel_feedforward_test wheel_feedforward_test.cpp)
target_include_directories(wheel_feedforward_test PRIVATE ${FIRMWARE_DIR}/control)
add_test(NAME wheel_feedforward COMMAND wheel_feedforward_test)

add_executable(velocity_ramp_test velocity_ramp_test.cpp)
target_include_directories(velocity_ramp_test PRIVATE ${FIRMWARE_DIR}/control)
add_test(NAME velocity_ramp COMMAND velocity_ramp_test)

add_executable(gyro_bias_tracker_test gyro_bias_tracker_test.cpp)
target_include_directories(gyro_bias_tracker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../components/lsm6ds3_driver)
add_test(NAME gyro_bias_tracker COMMAND gyro_bias_tracker_test)

add_executable(runtime_stats_test runtime_stats_test.cpp)
target_include_directories(runtime_stats_test PRIVATE ${FIRMWARE_DIR}/core)
add_test(NAME runtime_stats COMMAND runtime_stats_test)
