cmake_minimum_required(VERSION 3.16)
project(leap_host_tests LANGUAGES CXX)

enable_testing()

add_executable(
    wheel_pair_controller_test
    wheel_pair_controller_test.cpp
    ../main/control/wheel_pair_controller.cpp
)
target_include_directories(
    wheel_pair_controller_test
    PRIVATE ../main/control
)
target_compile_features(wheel_pair_controller_test PRIVATE cxx_std_17)
target_compile_options(wheel_pair_controller_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME wheel_pair_controller_test COMMAND wheel_pair_controller_test)

add_executable(publish_probe_stats_test publish_probe_stats_test.cpp)
target_include_directories(publish_probe_stats_test PRIVATE ../main/tasks/network)
target_compile_features(publish_probe_stats_test PRIVATE cxx_std_17)
target_compile_options(publish_probe_stats_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME publish_probe_stats_test COMMAND publish_probe_stats_test)

add_executable(agent_ping_health_test agent_ping_health_test.cpp)
target_include_directories(agent_ping_health_test PRIVATE ../main/tasks/network)
target_compile_features(agent_ping_health_test PRIVATE cxx_std_17)
target_compile_options(agent_ping_health_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME agent_ping_health_test COMMAND agent_ping_health_test)

add_executable(laser_scan_downsampler_test laser_scan_downsampler_test.cpp)
target_include_directories(laser_scan_downsampler_test PRIVATE ../main/tasks/network)
target_compile_features(laser_scan_downsampler_test PRIVATE cxx_std_17)
target_compile_options(laser_scan_downsampler_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME laser_scan_downsampler_test COMMAND laser_scan_downsampler_test)

add_executable(
    battery_motion_interlock_test
    battery_motion_interlock_test.cpp
    ../main/control/battery_motion_interlock.cpp
)
target_include_directories(
    battery_motion_interlock_test
    PRIVATE ../main ../main/control
)
target_compile_features(battery_motion_interlock_test PRIVATE cxx_std_17)
target_compile_options(battery_motion_interlock_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME battery_motion_interlock_test COMMAND battery_motion_interlock_test)

add_executable(emergency_stop_gate_test emergency_stop_gate_test.cpp)
target_include_directories(emergency_stop_gate_test PRIVATE ../main/control)
target_compile_features(emergency_stop_gate_test PRIVATE cxx_std_17)
target_compile_options(emergency_stop_gate_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME emergency_stop_gate_test COMMAND emergency_stop_gate_test)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(
    NAME analyze_motor_startup_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_analyze_motor_startup.py
)
add_test(
    NAME motor_startup_recording_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_motor_startup_recording.py
)
add_test(
    NAME analyze_motor_timing_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_analyze_motor_timing.py
)
