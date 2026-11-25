cmake_minimum_required(VERSION 3.16)

# Cross-compiling for STM32F405
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

project(mamba_f405_drone C ASM)

set(CMAKE_C_STANDARD 11)

# Compiler flags for ARM Cortex-M4 with hardware FPU
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_FLAGS} -Wall -Wextra -O2 -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${CPU_FLAGS} -Wl,--gc-sections")

include_directories(
    inc
)

file(GLOB_RECURSE SOURCES
    src/*.c
)

add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Target binary generation
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND arm-none-eabi-objcopy -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Building binary and hex images"
)
