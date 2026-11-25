# MAMBA F405 MK2 V2 - Hardware Pinout & Architecture Reference

## 1. Microcontroller Overview
* **Processor**: STM32F405RGT6 (ARM Cortex-M4 with FPU)
* **Clock Speed**: 168 MHz
* **Flash Memory**: 1024 KB (1 MB)
* **SRAM**: 192 KB (128 KB SRAM + 64 KB CCM RAM)
* **Operating Voltage**: 3.3V internal logic (5V / V_BAT external supply)

---

## 2. Onboard Sensors & Peripherals

### 2.1 IMU (Gyroscope & Accelerometer)
* **Sensor**: InvenSense MPU6000 (or BMI270 on select revisions)
* **Bus**: SPI1
* **Pin Mapping**:
  * `SCK`:  `PA5` (SPI1_SCK)
  * `MISO`: `PA6` (SPI1_MISO)
  * `MOSI`: `PA7` (SPI1_MOSI)
  * `CS`:   `PC2` (GPIO Output, Active LOW)
  * `INT`:  `PC4` (Data Ready Interrupt, Rising Edge)

### 2.2 OSD (On-Screen Display - Optional for FPV)
* **Chip**: Maxim AT7456E
* **Bus**: SPI2
* **Pin Mapping**:
  * `SCK`:  `PB13`
  * `MISO`: `PB14`
  * `MOSI`: `PB15`
  * `CS`:   `PB12`

### 2.3 Onboard Flash (Blackbox)
* **Chip**: Winbond W25Q128 (16MB NOR Flash)
* **Bus**: SPI3
* **Pin Mapping**:
  * `SCK`:  `PC10`
  * `MISO`: `PC11`
  * `MOSI`: `PC12`
  * `CS`:   `PC13`

---

## 3. External I/O, UARTs, and Motors

### 3.1 Motor Outputs (Quad-X Configuration)
The 4 ESC control signals are mapped to hardware timer PWM outputs:

| Motor | Physical Pad | STM32 Pin | Timer Channel | Betaflight Quad-X Position |
| :--- | :--- | :--- | :--- | :--- |
| **M1** | Motor 1 | `PB0` | `TIM3_CH3` | Rear-Right (CCW) |
| **M2** | Motor 2 | `PB1` | `TIM3_CH4` | Front-Right (CW) |
| **M3** | Motor 3 | `PA3` | `TIM2_CH4` / `TIM5_CH4` | Rear-Left (CW) |
| **M4** | Motor 4 | `PA2` | `TIM2_CH3` / `TIM5_CH3` | Front-Left (CCW) |

*Standard PWM*: 1000 µs (idle) to 2000 µs (full throttle) at 50 Hz to 400 Hz.  
*DShot600*: 600 kbps digital protocol supported on these timer pins with DMA.

### 3.2 Serial Ports (UARTs)

| UART | TX Pin | RX Pin | Typical Usage |
| :--- | :--- | :--- | :--- |
| **UART1** | `PA9`  | `PA10` | RC Receiver (ExpressLRS / CRSF / SBUS) |
| **UART3** | `PB10` | `PB11` | Telemetry / GPS / VTX MSP |
| **UART6** | `PC6`  | `PC7`  | MATLAB Ground Station Telemetry (Bidirectional) |
| **USB**   | `PA11` (DM) | `PA12` (DP) | Virtual COM Port / DFU Bootloader |

### 3.3 Analog Sensing (Battery Monitoring)
* **Battery Voltage (`V_BAT`)**: `PC0` (ADC1 Channel 10, typically 11:1 voltage divider).
* **Battery Current (`CURR`)**:  `PC1` (ADC1 Channel 11, analog voltage from ESC shunt resistor).

### 3.4 Status Indicators
* **Status LED**: `PC15` (Active LOW / HIGH depending on board version)
* **Buzzer**: `PC14` (Open-drain / Transistor-driven)

---

## 4. SWD Debugging Pads
On the underside of the Mamba F405 MK2 board, you will find 4 small round pads labeled:
1. `3V3`: 3.3V reference from ST-Link
2. `GND`: Ground connection
3. `DIO`: SWDIO (`PA13`)
4. `CLK`: SWCLK (`PA14`)

Connecting these 4 lines to an **ST-Link V2/V3** gives full in-circuit debugging (breakpoints, variable watch, direct flash) from STM32CubeIDE or Keil.
