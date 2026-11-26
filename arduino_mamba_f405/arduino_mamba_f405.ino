/**
 ******************************************************************************
 * @file    arduino_mamba_f405.ino
 * @brief   Arduino IDE (STM32duino) Firmware for Diatone MAMBA F405 MK2 V2
 *          Enables Native USB CDC (Virtual COM Port on PA11/PA12) so MATLAB
 *          Ground Station & test_serial.py work directly over the USB cable!
 *
 *          Arduino IDE Board Settings (STMicroelectronics STM32):
 *            - Board:       Generic STM32F4 series
 *            - Board part:  Generic F405RGTx
 *            - USB support: CDC (generic 'Serial' supersede U(S)ART)
 *            - Upload:      STM32CubeProgrammer (DFU)
 ******************************************************************************
 */

#include <SPI.h>
#include <math.h>

extern "C" {
#include "src/config.h"
#include "src/mpu6000.h"
#include "src/rc_receiver.h"
#include "src/motors.h"
#include "src/flight_controller.h"
#include "src/telemetry.h"
#include "src/blackbox.h"
#include "src/ai_fault_detector.h"
}

/* ========================================================================== */
/*                    HARDWARE INSTANCES (STM32duino)                         */
/* ========================================================================== */
static SPIClass SPI_MPU(PA7, PA6, PA5); /* MOSI=PA7, MISO=PA6, SCK=PA5 */
static Uart     SerialCRSF(PA10, PA9);    /* UART1: CRSF RX=PA10, TX=PA9 */

static void Direct_PWM_Timers_Init(void) {
    /* Enable GPIOA, GPIOB, TIM2, TIM3 clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Configure PB0 (TIM3_CH3, AF2), PB1 (TIM3_CH4, AF2) */
    GPIOB->MODER   = (GPIOB->MODER & ~((3UL << 0) | (3UL << 2))) | ((2UL << 0) | (2UL << 2));
    GPIOB->OSPEEDR |= (3UL << 0) | (3UL << 2);
    GPIOB->AFR[0]  = (GPIOB->AFR[0] & ~((0xFUL << 0) | (0xFUL << 4))) | ((2UL << 0) | (2UL << 4));

    /* Configure PA2 (TIM2_CH3, AF1), PA3 (TIM2_CH4, AF1) */
    GPIOA->MODER   = (GPIOA->MODER & ~((3UL << 4) | (3UL << 6))) | ((2UL << 4) | (2UL << 6));
    GPIOA->OSPEEDR |= (3UL << 4) | (3UL << 6);
    GPIOA->AFR[0]  = (GPIOA->AFR[0] & ~((0xFUL << 8) | (0xFUL << 12))) | ((1UL << 8) | (1UL << 12));

    /* APB1 timer clock = 84 MHz -> PSC = 83 gives 1 MHz (1 us); ARR = 2499 gives 400 Hz */
    TIM3->PSC   = 83U;
    TIM3->ARR   = 2499U;
    TIM3->CCMR2 = (6UL << 4) | (1UL << 3) | (6UL << 12) | (1UL << 11);
    TIM3->CCER  = (1UL << 8) | (1UL << 12);
    TIM3->CCR3  = 1000U;
    TIM3->CCR4  = 1000U;
    TIM3->CR1   = (1UL << 7) | (1UL << 0);

    TIM2->PSC   = 83U;
    TIM2->ARR   = 2499U;
    TIM2->CCMR2 = (6UL << 4) | (1UL << 3) | (6UL << 12) | (1UL << 11);
    TIM2->CCER  = (1UL << 8) | (1UL << 12);
    TIM2->CCR3  = 1000U;
    TIM2->CCR4  = 1000U;
    TIM2->CR1   = (1UL << 7) | (1UL << 0);
}

static MPU6000_Data_t   s_imu;
static RC_Channels_t    s_rc;
static Motor_Outputs_t  s_motors;
static Attitude_t       s_attitude;
static AI_Diagnostics_t s_ai_diag;
static bool             s_is_armed = false;
static bool             s_imu_ok   = false;
static bool             s_bench_test_active = false;
static bool             s_all_motors_active = false;
static bool             s_esc_cal_active    = false;
static bool             s_manual_pwm_active = false;
static float            s_manual_pwm[4]     = {1000.0f, 1000.0f, 1000.0f, 1000.0f};
static uint32_t         s_bench_start_ms    = 0;
static uint32_t         s_imu_cs_pin        = PC2; /* Auto-switched to PA4 if ICM42688-P detected */
static uint32_t         s_vbat_pin          = 0U;  /* 0 = auto-detect (PC1 vs PC0) */
static bool             s_web_rc_active     = false;
static uint32_t         s_web_rc_last_ms    = 0U;
static RC_Channels_t    s_web_rc            = {};

/* ========================================================================== */
/*                    C-LINKAGE HARDWARE HOOKS FOR FIRMWARE                   */
/* ========================================================================== */
extern "C" {

#define DFU_BOOT_MAGIC 0xB00710ADUL

/* Override STM32duino weak HSI clock with exact 8 MHz HSE Crystal -> 168 MHz + 48 MHz USB */
void SystemClock_Config(void) {
    /* Enter STM32 ROM DFU bootloader BEFORE USB/clock init if requested before reset */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;
    (void)PWR->CR;
    if (RTC->BKP0R == DFU_BOOT_MAGIC) {
        RTC->BKP0R = 0UL;
        __DSB();
        __disable_irq();
        SysTick->CTRL = 0UL;
        SysTick->LOAD = 0UL;
        SysTick->VAL  = 0UL;
        HAL_RCC_DeInit();
        __HAL_RCC_SYSCFG_CLK_ENABLE();
        __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
        __set_MSP(*(__IO uint32_t *)0x1FFF0000UL);
        __enable_irq();
        ((void (*)(void))(*(__IO uint32_t *)0x1FFF0004UL))();
        while (1) {}
    }

    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

uint32_t System_Millis(void) {
    return millis();
}

void Delay_Ms(uint32_t ms) {
    delay(ms);
}

void SPI1_Select(bool select) {
    if (select) {
        SPI_MPU.beginTransaction(SPISettings(5250000, MSBFIRST, SPI_MODE3));
        digitalWrite(s_imu_cs_pin, LOW);
    } else {
        digitalWrite(s_imu_cs_pin, HIGH);
        SPI_MPU.endTransaction();
    }
}

uint8_t SPI1_TransferByte(uint8_t byte) {
    return SPI_MPU.transfer(byte);
}

void SPI1_TransferBuffer(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        uint8_t tx = tx_buf ? tx_buf[i] : 0x00U;
        uint8_t rx = SPI_MPU.transfer(tx);
        if (rx_buf) {
            rx_buf[i] = rx;
        }
    }
}

void Timer_Set_PWM_Pulse_Us(uint8_t channel, float pulse_us) {
    uint32_t val = (uint32_t)(pulse_us + 0.5f);
    if (val < 1000U) val = 1000U;
    if (val > 2000U) val = 2000U;

    switch (channel) {
        case 1: TIM3->CCR3 = val; break; /* M1: PB0 */
        case 2: TIM3->CCR4 = val; break; /* M2: PB1 */
        case 3: TIM2->CCR4 = val; break; /* M3: PA3 */
        case 4: TIM2->CCR3 = val; break; /* M4: PA2 */
        default: break;
    }
}

void UART6_SendString(const char *str) {
    if (!str) return;
    /* Send telemetry directly over Native USB CDC (Serial) */
    Serial.print(str);
}

void System_Enter_DFU_Bootloader(void) {
    Serial.println("$ACK,DFU_RESET");
    Serial.flush();
    delay(30);
    Serial.end();
    delay(50);

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;
    (void)PWR->CR;
    RTC->BKP0R = DFU_BOOT_MAGIC;
    __DSB();
    NVIC_SystemReset();
    while (1) {}
}

} /* extern "C" */

static float Read_Vbat_Auto_Volts(void) {
    if (s_vbat_pin == 0U) {
        (void)analogRead(PC1);
        uint32_t c1 = (uint32_t)analogRead(PC1);
        (void)analogRead(PC0);
        uint32_t c0 = (uint32_t)analogRead(PC0);
        pinMode(PC3, INPUT_ANALOG);
        (void)analogRead(PC3);
        uint32_t c3 = (uint32_t)analogRead(PC3);

        if (c1 >= 250U && c0 < 150U && c3 < 150U) {
            s_vbat_pin = PC1;
        } else if (c0 >= 250U && c1 < 150U && c3 < 150U) {
            s_vbat_pin = PC0;
        } else if (c3 >= 250U && c1 < 150U && c0 < 150U) {
            s_vbat_pin = PC3;
        } else if (s_imu_cs_pin == PA4 && c3 >= 250U) {
            s_vbat_pin = PC3;
        } else if (s_imu_cs_pin == PA4 && c1 >= 250U) {
            s_vbat_pin = PC1;
        } else {
            uint32_t raw = (c3 > c1) ? c3 : c1;
            if (c0 > raw) raw = c0;
            return ((float)raw / 4095.0f) * 3.3f * 11.0f;
        }
    }
    uint32_t raw = (uint32_t)analogRead(s_vbat_pin);
    return ((float)raw / 4095.0f) * 3.3f * 11.0f;
}

/* ========================================================================== */
/*                              SETUP                                         */
/* ========================================================================== */
void setup() {
    /* Start 400 Hz 1000 us PWM immediately so ESCs arm cleanly without beeping */
    Direct_PWM_Timers_Init();

    pinMode(PC15, OUTPUT); /* Status LED */
    pinMode(PC14, OUTPUT); /* Buzzer */
    digitalWrite(PC15, LOW);
    digitalWrite(PC14, LOW);

    /* Initialize USB CDC Virtual COM Port & CRSF UART1 (PA10 RX / PA9 TX) */
    Serial.begin(115200);
    SerialCRSF.begin(RC_UART_BAUDRATE);

    /* Initialize ADC for VBat (auto-detect PC1 vs PC0) */
    analogReadResolution(12);
    pinMode(PC0, INPUT_ANALOG);
    pinMode(PC1, INPUT_ANALOG);

    /* Initialize SPI1 and auto-detect IMU CS pin (PC2 vs PA4) */
    pinMode(PC2, OUTPUT);
    pinMode(PA4, OUTPUT);
    digitalWrite(PC2, HIGH);
    digitalWrite(PA4, HIGH);
    SPI_MPU.begin();

    auto probeWhoAmI = [&](uint32_t cs) -> uint8_t {
        SPI_MPU.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
        digitalWrite(cs, LOW);
        SPI_MPU.transfer(0x75 | 0x80);
        uint8_t id = SPI_MPU.transfer(0x00);
        digitalWrite(cs, HIGH);
        SPI_MPU.endTransaction();
        return id;
    };
    uint8_t id_pc2 = probeWhoAmI(PC2);
    uint8_t id_pa4 = probeWhoAmI(PA4);
    if (id_pa4 == 0x47U || id_pa4 == 0x68U) {
        s_imu_cs_pin = PA4;
    } else if (id_pc2 == 0x47U || id_pc2 == 0x68U) {
        s_imu_cs_pin = PC2;
    }

    /* Initialize Flight Controller & AI Modules */
    Motors_Init();
    RC_Receiver_Init();
    Telemetry_Init();
    Blackbox_Init();
    Flight_Controller_Init();
    AI_Fault_Detector_Init();

    /* Initialize & Calibrate MPU6000 */
    s_imu_ok = MPU6000_Init();
    if (s_imu_ok) {
        digitalWrite(PC15, HIGH);
        MPU6000_Calibrate_Gyro(&s_imu, 1000);
        MPU6000_Read_All(&s_imu);
        Flight_Controller_Reset_Attitude(s_imu.ax, s_imu.ay, s_imu.az);
        digitalWrite(PC15, LOW);
    }
}

/* ========================================================================== */
/*                           1 kHz MAIN FLIGHT LOOP                           */
/* ========================================================================== */
void loop() {
    static uint32_t last_loop_us      = 0;
    static uint32_t telemetry_counter = 0;
    static uint32_t ai_tlm_counter    = 0;
    static uint32_t led_counter       = 0;
    static bool     led_state         = false;

    uint32_t now_us = micros();
    if ((now_us - last_loop_us) < 1000UL) {
        return; /* Wait for 1 ms (1 kHz) tick */
    }
    last_loop_us = now_us;
    uint32_t now_ms = millis();

    /* Read incoming CRSF receiver bytes on UART1 (PA10/PA9) */
    while (SerialCRSF.available() > 0) {
        RC_Receiver_Process_Byte((uint8_t)SerialCRSF.read(), now_ms);
    }

    /* Check USB Serial Commands ($CMD/PWM/RC,... or single-char 'T','A','C','S') */
    static char serial_line[80];
    static uint8_t serial_idx = 0;
    static bool in_dollar_cmd = false;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        Telemetry_Process_Rx_Byte((uint8_t)c);
        if (c == '$') {
            in_dollar_cmd = true;
            serial_idx = 0;
            serial_line[serial_idx++] = c;
        } else if (c == '\r' || c == '\n') {
            if (in_dollar_cmd && serial_idx > 0) {
                serial_line[serial_idx] = '\0';
                if (strncmp(serial_line, "$PWM,", 5) == 0) {
                    int m1 = 1000, m2 = 1000, m3 = 1000, m4 = 1000;
                    int parsed = sscanf(serial_line + 5, "%d,%d,%d,%d", &m1, &m2, &m3, &m4);
                    if (parsed == 1) { m2 = m1; m3 = m1; m4 = m1; }
                    if (parsed >= 1) {
                        auto clampBench = [](int v) -> float {
                            if (v < 1000) return 1000.0f;
                            if (v > 2000) return 2000.0f;
                            return (float)v;
                        };
                        s_manual_pwm[0] = clampBench(m1);
                        s_manual_pwm[1] = clampBench(m2);
                        s_manual_pwm[2] = clampBench(m3);
                        s_manual_pwm[3] = clampBench(m4);
                        s_manual_pwm_active = (m1 > 1000 || m2 > 1000 || m3 > 1000 || m4 > 1000);
                        s_bench_test_active = false;
                        s_all_motors_active = false;
                        s_esc_cal_active = false;
                        s_web_rc_active = false;
                        if (!s_manual_pwm_active) {
                            s_is_armed = false;
                            Motors_Disarm();
                        }
                        Serial.print("$ACK,PWM,");
                        Serial.print((int)s_manual_pwm[0]); Serial.print(',');
                        Serial.print((int)s_manual_pwm[1]); Serial.print(',');
                        Serial.print((int)s_manual_pwm[2]); Serial.print(',');
                        Serial.println((int)s_manual_pwm[3]);
                    }
                } else if (strncmp(serial_line, "$RC,", 4) == 0) {
                    /* Format: $RC,<thr_1000>,<roll_mrad>,<pitch_mrad>,<yaw_mrad>,<mode>,<arm> */
                    int thr_i = 0, r_i = 0, p_i = 0, y_i = 0, mode_i = 1, arm_i = 0;
                    if (sscanf(serial_line + 4, "%d,%d,%d,%d,%d,%d", &thr_i, &r_i, &p_i, &y_i, &mode_i, &arm_i) >= 5) {
                        if (thr_i < 0) thr_i = 0;
                        if (thr_i > 1000) thr_i = 1000;
                        if (mode_i < 0 || mode_i > 2) mode_i = 1;
                        s_web_rc.throttle    = (float)thr_i * 0.001f;
                        s_web_rc.roll_cmd    = (float)r_i * 0.001f;
                        s_web_rc.pitch_cmd   = (float)p_i * 0.001f;
                        s_web_rc.yaw_cmd     = (float)y_i * 0.001f;
                        s_web_rc.flight_mode = (uint8_t)mode_i;
                        s_web_rc.arm_switch  = (arm_i != 0);
                        s_web_rc.airmode     = false;
                        s_web_rc.failsafe    = false;
                        s_web_rc_active      = (arm_i != 0);
                        s_web_rc_last_ms     = now_ms;
                        if (!s_web_rc_active) {
                            s_is_armed = false;
                            Motors_Disarm();
                        }
                        s_bench_test_active = false;
                        s_all_motors_active = false;
                        s_esc_cal_active    = false;
                        s_manual_pwm_active = false;
                    }
                } else if (strcmp(serial_line, "$CMD,ADC_SCAN") == 0) {
                    pinMode(PC3, INPUT_ANALOG);
                    pinMode(PC5, INPUT_ANALOG);
                    uint32_t c0 = analogRead(PC0);
                    uint32_t c1 = analogRead(PC1);
                    uint32_t c3 = analogRead(PC3);
                    uint32_t c5 = analogRead(PC5);
                    uint32_t c2 = 0U;
                    if (s_imu_cs_pin == PA4) {
                        pinMode(PC2, INPUT_ANALOG);
                        (void)analogRead(PC2);
                        c2 = analogRead(PC2);
                        pinMode(PC2, OUTPUT);
                        digitalWrite(PC2, HIGH);
                    }
                    Serial.print("$ADC,PC0="); Serial.print(c0);
                    Serial.print(",PC1="); Serial.print(c1);
                    Serial.print(",PC2="); Serial.print(c2);
                    Serial.print(",PC3="); Serial.print(c3);
                    Serial.print(",PC5="); Serial.println(c5);
                } else if (strcmp(serial_line, "$CMD,RAW_IMU") == 0) {
                    Serial.print("$RAW_IMU,A=(");
                    Serial.print(s_imu.ax, 2); Serial.print(',');
                    Serial.print(s_imu.ay, 2); Serial.print(',');
                    Serial.print(s_imu.az, 2); Serial.print("),G=(");
                    Serial.print(s_imu.gx, 4); Serial.print(',');
                    Serial.print(s_imu.gy, 4); Serial.print(',');
                    Serial.print(s_imu.gz, 4); Serial.print("),BIAS=(");
                    Serial.print(s_imu.gx_bias, 4); Serial.print(',');
                    Serial.print(s_imu.gy_bias, 4); Serial.print(',');
                    Serial.print(s_imu.gz_bias, 4); Serial.println(')');
                } else if (strcmp(serial_line, "$CMD,INFO") == 0) {
                    const char *cs_str   = (s_imu_cs_pin == PA4) ? "PA4" : "PC2";
                    const char *vbat_str = (s_vbat_pin == PC3) ? "PC3" : ((s_vbat_pin == PC1) ? "PC1" : ((s_vbat_pin == PC0) ? "PC0" : "AUTO"));
                    Serial.print("$INFO,IMU_OK="); Serial.print(s_imu_ok ? 1 : 0);
                    Serial.print(",CS="); Serial.print(cs_str);
                    Serial.print(",VBAT="); Serial.print(vbat_str);
                    Serial.print(",CRSF="); Serial.print(s_rc.valid_frames);
                    Serial.print(",FS="); Serial.print(s_rc.failsafe ? 1 : 0);
                    Serial.print(",ARM="); Serial.println(s_is_armed ? 1 : 0);
                } else if (strcmp(serial_line, "$CMD,IMU_ID") == 0) {
                    auto readRegPin = [&](uint32_t csPin, uint8_t reg, bool dummy) -> uint8_t {
                        pinMode(csPin, OUTPUT);
                        digitalWrite(csPin, HIGH);
                        delay(1);
                        SPI_MPU.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
                        digitalWrite(csPin, LOW);
                        SPI_MPU.transfer(reg | 0x80);
                        if (dummy) SPI_MPU.transfer(0x00);
                        uint8_t v = SPI_MPU.transfer(0x00);
                        digitalWrite(csPin, HIGH);
                        SPI_MPU.endTransaction();
                        return v;
                    };
                    uint8_t pc2_75 = readRegPin(PC2, 0x75, false);
                    uint8_t pa4_75 = readRegPin(PA4, 0x75, false);
                    uint8_t pc2_00 = readRegPin(PC2, 0x00, true);
                    uint8_t pa4_00 = readRegPin(PA4, 0x00, true);
                    Serial.print("$IMU_ID,PC2_75=0x"); Serial.print(pc2_75, HEX);
                    Serial.print(",PA4_75=0x"); Serial.print(pa4_75, HEX);
                    Serial.print(",PC2_00=0x"); Serial.print(pc2_00, HEX);
                    Serial.print(",PA4_00=0x"); Serial.println(pa4_00, HEX);
                } else if (strcmp(serial_line, "$CMD,CAL_GYRO") == 0) {
                    if (s_imu_ok) {
                        MPU6000_Calibrate_Gyro(&s_imu, 500);
                        MPU6000_Read_All(&s_imu);
                        Flight_Controller_Reset_Attitude(s_imu.ax, s_imu.ay, s_imu.az);
                        Serial.println("$ACK,CAL_GYRO_OK");
                    } else {
                        Serial.println("$ERR,IMU_NOT_FOUND");
                    }
                }
            }
            in_dollar_cmd = false;
            serial_idx = 0;
        } else if (in_dollar_cmd) {
            if (serial_idx < sizeof(serial_line) - 1) {
                serial_line[serial_idx++] = c;
            }
        } else {
            if (c == 'T' || c == 't') {
                s_bench_test_active = true;
                s_all_motors_active = false;
                s_manual_pwm_active = false;
                s_bench_start_ms = now_ms;
            } else if (c == 'A' || c == 'a') {
                s_all_motors_active = true;
                s_bench_test_active = false;
                s_esc_cal_active = false;
                s_manual_pwm_active = false;
                s_bench_start_ms = now_ms;
            } else if (c == 'C' || c == 'c') {
                s_esc_cal_active = true;
                s_bench_test_active = false;
                s_all_motors_active = false;
                s_manual_pwm_active = false;
                s_bench_start_ms = now_ms;
            } else if (c == 'S' || c == 's') {
                s_bench_test_active = false;
                s_all_motors_active = false;
                s_esc_cal_active = false;
                s_manual_pwm_active = false;
                s_is_armed = false;
                Motors_Disarm();
            }
        }
    }

    /* 1. Read IMU */
    if (s_imu_ok) {
        MPU6000_Read_All(&s_imu);
    }

    /* 2. Read Battery Voltage (auto-detected PC1 vs PC0 with 11:1 divider) */
    float vbat = Read_Vbat_Auto_Volts();

    /* 3. Control Mode: Bench Test (triggered via USB 'T') or Safe Disarmed */
    if (s_bench_test_active) {
        uint32_t elapsed = now_ms - s_bench_start_ms;
        s_rc.failsafe = false;
        s_rc.flight_mode = 1;
        s_rc.airmode = false;
        s_rc.roll_cmd = 0.0f;
        s_rc.pitch_cmd = 0.0f;
        s_rc.yaw_cmd = 0.0f;

        if (elapsed >= 6000U && elapsed < 12000U) {
            s_is_armed = true;
            s_rc.arm_switch = true;
            s_rc.throttle = 0.08f;
            Flight_Controller_Update(&s_rc, &s_imu, true, false, vbat, LOOP_DT, &s_motors, &s_attitude);
            if (s_motors.m1 > 1300.0f) s_motors.m1 = 1300.0f;
            if (s_motors.m2 > 1300.0f) s_motors.m2 = 1300.0f;
            if (s_motors.m3 > 1300.0f) s_motors.m3 = 1300.0f;
            if (s_motors.m4 > 1300.0f) s_motors.m4 = 1300.0f;
            Motors_Set_All_Pwm(&s_motors);
        } else {
            s_is_armed = false;
            s_rc.arm_switch = false;
            s_rc.throttle = 0.0f;
            Flight_Controller_Update(&s_rc, &s_imu, false, false, vbat, LOOP_DT, &s_motors, &s_attitude);
            if (elapsed < 1500U) {
                s_motors.m1 = 1150.0f; s_motors.m2 = 1000.0f; s_motors.m3 = 1000.0f; s_motors.m4 = 1000.0f;
                Motors_Set_All_Pwm(&s_motors);
            } else if (elapsed < 3000U) {
                s_motors.m1 = 1000.0f; s_motors.m2 = 1150.0f; s_motors.m3 = 1000.0f; s_motors.m4 = 1000.0f;
                Motors_Set_All_Pwm(&s_motors);
            } else if (elapsed < 4500U) {
                s_motors.m1 = 1000.0f; s_motors.m2 = 1000.0f; s_motors.m3 = 1150.0f; s_motors.m4 = 1000.0f;
                Motors_Set_All_Pwm(&s_motors);
            } else if (elapsed < 6000U) {
                s_motors.m1 = 1000.0f; s_motors.m2 = 1000.0f; s_motors.m3 = 1000.0f; s_motors.m4 = 1150.0f;
                Motors_Set_All_Pwm(&s_motors);
            } else {
                s_bench_test_active = false;
                Motors_Disarm();
            }
        }
    } else if (s_all_motors_active) {
        uint32_t elapsed = now_ms - s_bench_start_ms;
        if (elapsed < 8000U) {
            s_is_armed = true;
            s_rc.failsafe = false;
            s_rc.arm_switch = true;
            s_rc.flight_mode = 1;
            s_rc.airmode = false;
            s_rc.roll_cmd = 0.0f;
            s_rc.pitch_cmd = 0.0f;
            s_rc.yaw_cmd = 0.0f;
            s_rc.throttle = 0.12f; /* ~1173 us base so all 4 motors start cleanly together */
            Flight_Controller_Update(&s_rc, &s_imu, true, false, vbat, LOOP_DT, &s_motors, &s_attitude);
            if (s_motors.m1 > 1350.0f) s_motors.m1 = 1350.0f;
            if (s_motors.m2 > 1350.0f) s_motors.m2 = 1350.0f;
            if (s_motors.m3 > 1350.0f) s_motors.m3 = 1350.0f;
            if (s_motors.m4 > 1350.0f) s_motors.m4 = 1350.0f;
            Motors_Set_All_Pwm(&s_motors);
        } else {
            s_all_motors_active = false;
            s_is_armed = false;
            Motors_Disarm();
        }
    } else if (s_esc_cal_active) {
        /* ESC Calibration: 2000 us for 8 seconds (plug battery in during this window),
         * then drops to 1000 us (or press 'S' right after the 2 beeps) */
        uint32_t elapsed = now_ms - s_bench_start_ms;
        s_is_armed = false;
        Flight_Controller_Update(&s_rc, &s_imu, false, false, vbat, LOOP_DT, &s_motors, &s_attitude);
        if (elapsed < 8000U) {
            s_motors.m1 = 2000.0f; s_motors.m2 = 2000.0f;
            s_motors.m3 = 2000.0f; s_motors.m4 = 2000.0f;
            Motors_Set_All_Pwm(&s_motors);
        } else {
            s_esc_cal_active = false;
            Motors_Disarm();
        }
    } else if (s_manual_pwm_active) {
        s_is_armed = true;
        Flight_Controller_Update(&s_rc, &s_imu, false, false, vbat, LOOP_DT, &s_motors, &s_attitude);
        s_motors.m1 = s_manual_pwm[0];
        s_motors.m2 = s_manual_pwm[1];
        s_motors.m3 = s_manual_pwm[2];
        s_motors.m4 = s_manual_pwm[3];
        Motors_Set_All_Pwm(&s_motors);
    } else if (s_web_rc_active && (now_ms - s_web_rc_last_ms) < 500U) {
        s_rc = s_web_rc;
        if (!s_imu_ok || !s_rc.arm_switch) {
            s_is_armed = false;
        } else if (!s_is_armed) {
            if (s_rc.throttle <= MIN_THROTTLE_FOR_ARM &&
                fabsf(s_attitude.roll_deg) <= MAX_ARM_TILT_DEG &&
                fabsf(s_attitude.pitch_deg) <= MAX_ARM_TILT_DEG) {
                s_is_armed = true;
            }
        }
        if (s_is_armed) {
            Flight_Controller_Update(&s_rc, &s_imu, true, s_rc.airmode, vbat, LOOP_DT, &s_motors, &s_attitude);
            Motors_Set_All_Pwm(&s_motors);
        } else {
            Flight_Controller_Update(&s_rc, &s_imu, false, false, vbat, LOOP_DT, &s_motors, &s_attitude);
            Motors_Disarm();
        }
    } else {
        s_web_rc_active = false;
        /* 3b. Real Flight Mode via CRSF Receiver (UART1 PA10/PA9) */
        RC_Get_Channels(&s_rc);
        RC_Receiver_Check_Failsafe(&s_rc, now_ms);

        if (!s_imu_ok || s_rc.failsafe || !s_rc.arm_switch) {
            s_is_armed = false;
        } else if (!s_is_armed) {
            if (s_rc.throttle <= MIN_THROTTLE_FOR_ARM &&
                fabsf(s_attitude.roll_deg) <= MAX_ARM_TILT_DEG &&
                fabsf(s_attitude.pitch_deg) <= MAX_ARM_TILT_DEG) {
                s_is_armed = true;
            }
        }

        digitalWrite(PC14, s_rc.beeper ? HIGH : LOW);

        if (s_is_armed) {
            Flight_Controller_Update(&s_rc, &s_imu, true, s_rc.airmode, vbat, LOOP_DT, &s_motors, &s_attitude);
            Motors_Set_All_Pwm(&s_motors);
        } else {
            Flight_Controller_Update(&s_rc, &s_imu, false, false, vbat, LOOP_DT, &s_motors, &s_attitude);
            Motors_Disarm();
        }
    }

    /* 4. TinyML Fault Detector */
    AI_Fault_Detector_Update(
        s_imu.gx, s_imu.gy, s_attitude.gx_filt, s_attitude.gy_filt,
        s_imu.ax, s_imu.ay, s_imu.az, s_attitude.err_p, s_attitude.err_q,
        &s_motors, s_is_armed, &s_ai_diag
    );

    /* 5. Status LED Heartbeat */
    led_counter++;
    uint32_t period = s_is_armed ? 50U : (s_imu_ok ? 500U : 100U);
    if (led_counter >= period) {
        led_counter = 0U;
        led_state = !led_state;
        digitalWrite(PC15, led_state ? HIGH : LOW);
    }

    /* 6. Stream $TLM Telemetry over USB CDC at 50 Hz (every 20 ms) */
    telemetry_counter++;
    if (telemetry_counter >= 20U) {
        telemetry_counter = 0U;
        Telemetry_Send_Frame(&s_attitude, &s_motors, vbat, s_is_armed, s_rc.flight_mode);
    }

    /* 7. Stream $AI Telemetry at 10 Hz (every 100 ms) */
    ai_tlm_counter++;
    if (ai_tlm_counter >= 100U) {
        ai_tlm_counter = 0U;
        Telemetry_Send_AI_Frame(&s_attitude, &s_ai_diag);
    }
}
