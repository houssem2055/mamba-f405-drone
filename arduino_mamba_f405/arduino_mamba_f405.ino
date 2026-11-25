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
static uint32_t         s_bench_start_ms    = 0;

/* ========================================================================== */
/*                    C-LINKAGE HARDWARE HOOKS FOR FIRMWARE                   */
/* ========================================================================== */
extern "C" {

/* Override STM32duino weak HSI clock with exact 8 MHz HSE Crystal -> 168 MHz + 48 MHz USB */
void SystemClock_Config(void) {
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
        digitalWrite(PC2, LOW);
    } else {
        digitalWrite(PC2, HIGH);
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
    delay(20);
    Serial.end();
    delay(50);

    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    for (uint32_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    HAL_RCC_DeInit();
    HAL_DeInit();

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    uint32_t boot_msp   = *(__IO uint32_t *)0x1FFF0000UL;
    uint32_t boot_reset = *(__IO uint32_t *)0x1FFF0004UL;
    __set_MSP(boot_msp);
    __enable_irq();
    ((void (*)(void))boot_reset)();
    while (1) {}
}

} /* extern "C" */

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

    /* Initialize USB CDC Virtual COM Port */
    Serial.begin(115200);

    /* Initialize ADC for VBat on PC0 */
    analogReadResolution(12);
    pinMode(PC0, INPUT_ANALOG);

    /* Initialize SPI1 for MPU6000 */
    pinMode(PC2, OUTPUT);
    digitalWrite(PC2, HIGH);
    SPI_MPU.begin();

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

    /* Check USB Serial Commands ($CMD,... or single-char 'T'=seq, 'A'=all 4 together, 'S'=stop) */
    static bool in_dollar_cmd = false;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        Telemetry_Process_Rx_Byte((uint8_t)c);
        if (c == '$') {
            in_dollar_cmd = true;
        } else if (c == '\r' || c == '\n') {
            in_dollar_cmd = false;
        } else if (!in_dollar_cmd) {
            if (c == 'T' || c == 't') {
                s_bench_test_active = true;
                s_all_motors_active = false;
                s_bench_start_ms = now_ms;
            } else if (c == 'A' || c == 'a') {
                s_all_motors_active = true;
                s_bench_test_active = false;
                s_esc_cal_active = false;
                s_bench_start_ms = now_ms;
            } else if (c == 'C' || c == 'c') {
                s_esc_cal_active = true;
                s_bench_test_active = false;
                s_all_motors_active = false;
                s_bench_start_ms = now_ms;
            } else if (c == 'S' || c == 's') {
                s_bench_test_active = false;
                s_all_motors_active = false;
                s_esc_cal_active = false;
                s_is_armed = false;
                Motors_Disarm();
            }
        }
    }

    /* 1. Read IMU */
    if (s_imu_ok) {
        MPU6000_Read_All(&s_imu);
    }

    /* 2. Read Battery Voltage (PC0 with 11:1 divider) */
    float vbat = ((float)analogRead(PC0) / 4095.0f) * 3.3f * 11.0f;

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
    } else {
        s_is_armed = false;
        Flight_Controller_Update(&s_rc, &s_imu, false, false, vbat, LOOP_DT, &s_motors, &s_attitude);
        Motors_Disarm();
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
        Serial.print("$TLM,");
        Serial.print(s_attitude.roll_deg, 2); Serial.print(',');
        Serial.print(s_attitude.pitch_deg, 2); Serial.print(',');
        Serial.print(s_attitude.yaw_deg, 2); Serial.print(',');
        Serial.print(s_motors.m1, 0); Serial.print(',');
        Serial.print(s_motors.m2, 0); Serial.print(',');
        Serial.print(s_motors.m3, 0); Serial.print(',');
        Serial.print(s_motors.m4, 0); Serial.print(',');
        Serial.print(vbat, 2); Serial.print(',');
        Serial.print(s_is_armed ? 1 : 0); Serial.print(',');
        Serial.println((int)s_rc.flight_mode);
    }

    /* 7. Stream $AI Telemetry at 10 Hz (every 100 ms) */
    ai_tlm_counter++;
    if (ai_tlm_counter >= 100U) {
        ai_tlm_counter = 0U;
        Telemetry_Send_AI_Frame(&s_attitude, &s_ai_diag);
    }
}
