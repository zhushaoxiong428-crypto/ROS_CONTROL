# Linux-side MAVLink bridge to the leap_ros firmware over its UART.
#   cmake -S host/mavlink_bridge -B build-bridge && cmake --build build-bridge
#   ctest --test-dir build-bridge --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(leap_mavlink_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra -Werror)

# The same header-only MAVLink dialect the firmware is built with, so both
# sides always agree on message layouts and CRC extras.
set(MAVLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../components/mavlink_lib)

add_library(leap_bridge STATIC
    src/serial_port.cpp
    src/mavlink_link.cpp)
target_include_directories(leap_bridge PUBLIC include)
# SYSTEM: the generated MAVLink headers do not build cleanly under -Wextra.
target_include_directories(leap_bridge SYSTEM PUBLIC ${MAVLINK_DIR})

add_executable(leap_mavlink_bridge src/main.cpp)
target_link_libraries(leap_mavlink_bridge PRIVATE leap_bridge)

enable_testing()

add_executable(mavlink_link_test test/mavlink_link_test.cpp)
target_link_libraries(mavlink_link_test PRIVATE leap_bridge)
# Tests use assert(); keep it active in every build type.
target_compile_options(mavlink_link_test PRIVATE -UNDEBUG)
add_test(NAME mavlink_link COMMAND mavlink_link_test)

add_executable(rate_stats_test test/rate_stats_test.cpp)
target_link_libraries(rate_stats_test PRIVATE leap_bridge)
target_compile_options(rate_stats_test PRIVATE -UNDEBUG)
add_test(NAME rate_stats COMMAND rate_stats_test)
