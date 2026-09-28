include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)

set(idf_target "esp32s3")
set(idf_path "/home/wrj/esp/esp-idf")

set(RISCV_TARGETS "esp32c3" "esp32c6" "esp32p4")
set(RISCV_HARD_FLOAT_TARGETS "esp32p4")

if("${idf_target}" IN_LIST RISCV_TARGETS)
    set(CMAKE_SYSTEM_PROCESSOR riscv)
    if("${idf_target}" IN_LIST RISCV_HARD_FLOAT_TARGETS)
        # ESP32-P4 uses hardware floating point
        set(FLAGS "-march=rv32imafc -mabi=ilp32f -ffunction-sections -fdata-sections" CACHE STRING "" FORCE)
    else()
        # ESP32-C3/C6 use soft-float
        set(FLAGS "-ffunction-sections -fdata-sections" CACHE STRING "" FORCE)
    endif()
else()
    set(CMAKE_SYSTEM_PROCESSOR xtensa)
    set(FLAGS "-mlongcalls -ffunction-sections -fdata-sections" CACHE STRING "" FORCE)
endif()

set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(PLATFORM_NAME "LwIP")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_C_COMPILER /home/wrj/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc)
set(CMAKE_CXX_COMPILER /home/wrj/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32s3-elf-g++)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FLAGS} ${IDF_INCLUDES}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti ${FLAGS} ${IDF_INCLUDES}")

add_compile_definitions(ESP_PLATFORM LWIP_IPV4 LWIP_IPV6 PLATFORM_NAME_FREERTOS)

include_directories(
        "/home/wrj/leap_ros_mcu_driver-main/build/config"
        ${idf_path}/components/soc/${idf_target}/include
    )

if("${idf_target}" IN_LIST RISCV_TARGETS)
    include_directories(
        ${idf_path}/components/riscv/include
    )
endif()
