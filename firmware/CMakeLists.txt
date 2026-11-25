cmake_minimum_required(VERSION 3.16)
project(mamba_f405_drone C ASM)

set(CMAKE_C_STANDARD 11)

option(BUILD_HOST_TESTS "Build and run host SIL/Unit tests with native GCC" OFF)

if(BUILD_HOST_TESTS)
    add_executable(test_firmware
        src/biquad.c
        src/attitude_mahony.c
        src/flight_controller.c
        src/motors.c
        src/rc_receiver.c
        src/telemetry.c
        src/blackbox.c
        src/mpu6000.c
        ../tests/test_firmware.c
    )
    target_include_directories(test_firmware PRIVATE inc)
    target_compile_options(test_firmware PRIVATE -Wall -Wextra -Werror -O2)
    if(NOT WIN32)
        target_link_libraries(test_firmware m)
    endif()
else()
    # Cross-compiling bare-metal STM32F405RGT6 firmware
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR arm)

    set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_FLAGS} -Wall -Wextra -O2 -ffunction-sections -fdata-sections")
    set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/bsp/STM32F405RGTx_FLASH.ld")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${CPU_FLAGS} -T${LINKER_SCRIPT} -Wl,--gc-sections --specs=nosys.specs")

    include_directories(inc bsp)

    file(GLOB CORE_SOURCES "src/*.c")
    file(GLOB BSP_SOURCES  "bsp/*.c")

    add_executable(${PROJECT_NAME}.elf ${CORE_SOURCES} ${BSP_SOURCES})
    target_link_libraries(${PROJECT_NAME}.elf m)

    add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND arm-none-eabi-objcopy -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
        COMMAND arm-none-eabi-objcopy -O ihex   ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-size ${PROJECT_NAME}.elf
        COMMENT "Generating STM32F405 binary, hex, and reporting memory footprint"
    )
endif()
