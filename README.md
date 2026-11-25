# Mamba F405 MK2 V2 Drone Flight Controller (MATLAB & Embedded C)

This repository contains a complete Model-Based Design flight control system for a quadcopter equipped with the **Diatone MAMBA F405 MK2 V2** flight controller (STM32F405RGT6 ARM Cortex-M4, MPU6000 IMU).

---

## 📂 Project Structure

```
C:\Users\djall\Desktop\drone\
├── FULL_PROJECT_GUIDE_AR.md          # الدليل العربي الشامل والمفصل لكل ملف وسطر ودالة في المشروع
├── .gitignore                        # استثناءات Git لملفات البناء والمخلفات المؤقتة
├── docs/
│   ├── mamba_f405_pinout.md          # Complete hardware pin mapping & electrical specs
│   └── quadcopter_dynamics.md        # Mathematical equations of motion, mixer matrix, & PID theory
├── matlab/
│   ├── init_drone_parameters.m       # Physical parameters (mass, inertia, motor constants, PID gains)
│   ├── attitude_estimator.m          # Sensor fusion (Complementary filter for Gyro + Accel)
│   ├── flight_controller.m           # Cascaded P-PID attitude & rate controller + Quad-X mixer
│   ├── drone_simulation.m            # 6-DOF non-linear dynamic simulator with step responses & plots
│   ├── generate_c_code.m             # Embedded Coder automated C export script
│   └── telemetry_ground_station.m    # Real-time MATLAB Ground Station GUI over USB COM port
├── firmware/                         # Native C Embedded Flight Controller for STM32F405
│   ├── inc/
│   │   ├── config.h                  # Hardware pinouts, loop frequencies (1 kHz), safety limits
│   │   ├── mpu6000.h                 # SPI driver definitions for MPU6000 IMU
│   │   ├── motors.h                  # 4-Channel PWM motor driver
│   │   ├── rc_receiver.h             # CRSF (ExpressLRS/Crossfire) & SBUS receiver parser
│   │   ├── flight_controller.h       # C interface for the MATLAB controller
│   │   └── telemetry.h               # Telemetry packet streaming
│   ├── src/
│   │   ├── main.c                    # Deterministic 1 kHz flight loop with SysTick interrupt
│   │   ├── mpu6000.c                 # Full register-level SPI burst read & gyro auto-calibration
│   │   ├── motors.c                  # Motor pulse width generation (1000 - 2000 µs)
│   │   ├── rc_receiver.c             # Non-blocking CRSF frame decoding & failsafe handler
│   │   ├── flight_controller.c       # Embedded implementation of the MATLAB flight algorithms
│   │   └── telemetry.c               # Packet formatter ($TLM) for MATLAB
│   └── CMakeLists.txt                # Build configuration for ARM GCC / STM32CubeIDE
└── tools/
    └── test_serial.py                # Standalone Python monitor to test telemetry over USB
```

---

## 🚀 Quick Start Guide

### 1. Run the 6-DOF Flight Simulation in MATLAB
1. Open MATLAB.
2. In the MATLAB Current Folder bar, navigate to:
   ```matlab
   cd('C:\Users\djall\Desktop\drone\matlab');
   ```
3. Run the simulation script:
   ```matlab
   drone_simulation
   ```
4. This executes an 8-second simulated flight mission (Takeoff -> Hover -> Roll Step -> Pitch Step -> Yaw Rotation) and plots:
   * **Euler Angles** ($\phi, \theta, \psi$) vs. commanded setpoints.
   * **Body Angular Rates** ($p, q, r$).
   * **Individual Motor ESC Outputs** ($M_1, M_2, M_3, M_4$).
   * **Altitude response**.

---

### 2. Tuning PID Controller Gains in MATLAB
To adjust the flight performance:
1. Open `matlab/init_drone_parameters.m`.
2. Modify the gains:
   * `Kp_angle_roll` / `Kp_angle_pitch`: Outer attitude responsiveness.
   * `Kp_rate_roll` / `Ki_rate_roll` / `Kd_rate_roll`: Inner gyro rate damping and stiffness.
3. Re-run `drone_simulation.m` to verify stability before flying!

---

### 3. Generate Embedded C Code
If you have **MATLAB Coder** / **Embedded Coder**:
1. Run:
   ```matlab
   generate_c_code
   ```
2. The generated optimized C functions can be copied into `firmware/src/`.
*(Note: A pre-optimized C implementation matching the exact equations is already provided in `firmware/src/flight_controller.c`).*

---

### 4. Flashing the Firmware to Mamba F405 MK2

#### Method A: ST-Link V2 / V3 (Recommended for Debugging)
1. Solder 4 header pins or wires to the SWD pads under the Mamba board:
   * `3V3` -> ST-Link 3.3V
   * `GND` -> ST-Link GND
   * `CLK` -> ST-Link SWCLK
   * `DIO` -> ST-Link SWDIO
2. Open **STM32CubeIDE**, import the `firmware` folder, build, and click **Debug / Run**.

#### Method B: USB DFU (No Extra Hardware Needed)
1. Hold down the **BOOT button** on the Mamba F405 board.
2. While holding the button, plug the USB-C / micro-USB cable into your PC.
3. Open **STM32CubeProgrammer** (or Betaflight Configurator).
4. Select **USB** connection (device appears in DFU mode).
5. Load the compiled `.bin` or `.hex` file and click **Start Programming**.

---

### 5. Real-Time Telemetry with MATLAB Ground Station
1. Connect the Mamba F405 to your PC via USB.
2. Open MATLAB and run:
   ```matlab
   cd('C:\Users\djall\Desktop\drone\matlab');
   telemetry_ground_station
   ```
3. The script opens a live oscilloscope displaying the quadcopter's roll, pitch, yaw, and motor outputs at 50 Hz.

---

> [!CAUTION]
> **Bench Safety Rule #1**: ALWAYS remove all 4 propeller blades whenever the drone is connected to your computer or powered on your work bench!
