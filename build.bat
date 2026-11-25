@echo off
setlocal enabledelayedexpansion

echo ====================================================
echo   MAMBA F405 Drone Build and Test System
echo ====================================================

if "%1"=="" goto host_test
if "%1"=="test" goto host_test
if "%1"=="firmware" goto arm_firmware
if "%1"=="bench" goto arm_bench
if "%1"=="flash" goto flash_dfu
if "%1"=="flash_bench" goto flash_bench
if "%1"=="clean" goto clean

:host_test
echo [1/2] Compiling Host Unit Test Suite (tests/test_firmware.c)...
gcc -Wall -Wextra -Werror -O2 -Ifirmware/inc ^
    firmware/src/biquad.c ^
    firmware/src/attitude_mahony.c ^
    firmware/src/tiny_nn.c ^
    firmware/src/ai_fault_detector.c ^
    firmware/src/flight_controller.c ^
    firmware/src/motors.c ^
    firmware/src/rc_receiver.c ^
    firmware/src/telemetry.c ^
    firmware/src/blackbox.c ^
    firmware/src/mpu6000.c ^
    tests/test_firmware.c ^
    -o tests/test_firmware.exe -lm
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Host test compilation failed!
    exit /b %ERRORLEVEL%
)

echo [2/2] Running Unit Tests...
tests\test_firmware.exe
exit /b %ERRORLEVEL%

:arm_firmware
set "ARDUINO_GCC=%LOCALAPPDATA%\Arduino15\packages\STMicroelectronics\tools\xpack-arm-none-eabi-gcc\14.2.1-1.1\bin"
where arm-none-eabi-gcc >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    if exist "%ARDUINO_GCC%\arm-none-eabi-gcc.exe" (
        set "PATH=%ARDUINO_GCC%;%PATH%"
        echo [ARM] Using Arduino STM32 xpack-arm-none-eabi-gcc
    ) else (
        echo [WARNING] arm-none-eabi-gcc is not found in PATH or Arduino15.
        exit /b 1
    )
)

if not exist build mkdir build
echo [ARM] Building STM32F405 TinyML + Aerospace firmware...
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -std=c11 -Wall -Wextra -O2 -ffunction-sections -fdata-sections --specs=nosys.specs -Ifirmware/inc -Ifirmware/bsp -T firmware/bsp/STM32F405RGTx_FLASH.ld -Wl,--gc-sections -Wl,-Map=build/mamba_f405_drone.map firmware/src/*.c firmware/bsp/*.c -lm -o build/mamba_f405_drone.elf
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] ARM firmware build failed!
    exit /b %ERRORLEVEL%
)
arm-none-eabi-objcopy -O binary build/mamba_f405_drone.elf build/mamba_f405_drone.bin
arm-none-eabi-objcopy -O ihex build/mamba_f405_drone.elf build/mamba_f405_drone.hex
arm-none-eabi-size build/mamba_f405_drone.elf
echo [ARM] Build complete. Artifacts in build/
exit /b 0

:arm_bench
set "ARDUINO_GCC=%LOCALAPPDATA%\Arduino15\packages\STMicroelectronics\tools\xpack-arm-none-eabi-gcc\14.2.1-1.1\bin"
where arm-none-eabi-gcc >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    if exist "%ARDUINO_GCC%\arm-none-eabi-gcc.exe" (
        set "PATH=%ARDUINO_GCC%;%PATH%"
    ) else (
        echo [WARNING] arm-none-eabi-gcc is not found.
        exit /b 1
    )
)
if not exist build mkdir build
echo [BENCH] Building TEMPORARY NO-RC Bench Test firmware...
arm-none-eabi-gcc -DBENCH_TEST_NO_RC -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -std=c11 -Wall -Wextra -O2 -ffunction-sections -fdata-sections --specs=nosys.specs -Ifirmware/inc -Ifirmware/bsp -T firmware/bsp/STM32F405RGTx_FLASH.ld -Wl,--gc-sections firmware/src/*.c firmware/bsp/*.c -lm -o build/mamba_f405_bench_test.elf
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%
arm-none-eabi-objcopy -O binary build/mamba_f405_bench_test.elf build/mamba_f405_bench_test.bin
arm-none-eabi-objcopy -O ihex build/mamba_f405_bench_test.elf build/mamba_f405_bench_test.hex
arm-none-eabi-size build/mamba_f405_bench_test.elf
exit /b 0

:flash_bench
set "DFU_UTIL=%~dp0tools\dfu-util\win64\dfu-util.exe"
if not exist "%DFU_UTIL%" exit /b 1
call "%~f0" bench
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%
echo [DFU] Flashing TEMPORARY Bench Test firmware to 0x08000000...
"%DFU_UTIL%" -d 0483:df11 -a 0 -s 0x08000000:leave -D build\mamba_f405_bench_test.bin
exit /b %ERRORLEVEL%

:flash_dfu
set "DFU_UTIL=%~dp0tools\dfu-util\win64\dfu-util.exe"
if not exist "%DFU_UTIL%" (
    echo [ERROR] dfu-util.exe not found in tools\dfu-util\win64!
    exit /b 1
)
call "%~f0" firmware
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

echo [DFU] Flashing build\mamba_f405_drone.bin to 0x08000000...
"%DFU_UTIL%" -d 0483:df11 -a 0 -s 0x08000000:leave -D build\mamba_f405_drone.bin
exit /b %ERRORLEVEL%

:clean
echo Cleaning build outputs...
if exist tests\test_firmware.exe del tests\test_firmware.exe
if exist build rmdir /s /q build
echo Done.
exit /b 0
