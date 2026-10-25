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

add_executable(pwm_output_test pwm_output_test.cpp)
target_include_directories(pwm_output_test PRIVATE ${FIRMWARE_DIR}/control)
add_test(NAME pwm_output COMMAND pwm_output_test)
