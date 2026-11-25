#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                    SYSTEM & CLOCK CONFIGURATION                            */
/* ========================================================================== */
#define MCU_CLOCK_HZ            168000000UL   /* 168 MHz STM32F405 */
#define LOOP_FREQUENCY_HZ       1000.0f       /* Main control loop: 1 kHz */
#define LOOP_DT                 (1.0f / LOOP_FREQUENCY_HZ) /* 0.001 s (1 ms) */

/* ========================================================================== */
/*                       SAFETY & ARMING LIMITS                               */
/* ========================================================================== */
#define MAX_ARM_TILT_DEG        25.0f         /* Prevent arming if drone tilted > 25 deg */
#define MIN_THROTTLE_FOR_ARM    0.05f         /* Throttle must be at lowest position to arm */
#define RC_FAILSAFE_TIMEOUT_MS  300U          /* Failsafe disarm if no RC packet for 300 ms */

#define MOTOR_MIN_PWM_US        1000.0f       /* Disarmed / Off pulse width */
#define MOTOR_IDLE_PWM_US       1060.0f       /* Armed idle spin pulse width */
#define MOTOR_MAX_PWM_US        2000.0f       /* Full power pulse width */

/* ========================================================================== */
/*                   DIGITAL FILTER & CONTROL SETTINGS                        */
/* ========================================================================== */
#define GYRO_LPF_CUTOFF_HZ      90.0f         /* 2nd-order Butterworth Gyro LPF */
#define GYRO_NOTCH_CENTER_HZ    220.0f        /* 2nd-order Gyro Notch Filter */
#define GYRO_NOTCH_Q            2.5f
#define DTERM_LPF_CUTOFF_HZ     70.0f         /* 2nd-order Butterworth D-Term LPF */
#define BUTTERWORTH_Q           0.70710678f

#define MAHONY_KP               2.5f          /* Quaternion AHRS proportional gain */
#define MAHONY_KI               0.05f         /* Quaternion AHRS integral gyro-bias gain */

#define TPA_BREAKPOINT          0.50f         /* Throttle PID Attenuation start */
#define TPA_RATE                0.25f         /* Max P/D attenuation at 100% throttle */
#define VBAT_NOMINAL_4S         16.0f         /* Nominal 4S LiPo reference voltage */
#define RC_EXPO_FACTOR          0.25f         /* Stick center sensitivity smoothing */

/* ========================================================================== */
/*                    MAMBA F405 MK2 V2 PIN DEFINITIONS                       */
/* ========================================================================== */

/* IMU (MPU6000) on SPI1 */
#define MPU_SPI                 SPI1
#define MPU_SCK_PIN             5U            /* PA5 */
#define MPU_MISO_PIN            6U            /* PA6 */
#define MPU_MOSI_PIN            7U            /* PA7 */
#define MPU_CS_PIN              2U            /* PC2 (Active LOW) */
#define MPU_INT_PIN             4U            /* PC4 (Data Ready) */

/* Onboard Flash (W25Q128 16MB Blackbox) on SPI3 */
#define FLASH_SCK_PIN           10U           /* PC10 */
#define FLASH_MISO_PIN          11U           /* PC11 */
#define FLASH_MOSI_PIN          12U           /* PC12 */
#define FLASH_CS_PIN            13U           /* PC13 */

/* Motor PWM / DShot600 Outputs (Quad-X) */
#define MOTOR_1_PIN             0U            /* PB0 - TIM3_CH3 (Rear-Right, CCW) */
#define MOTOR_2_PIN             1U            /* PB1 - TIM3_CH4 (Front-Right, CW) */
#define MOTOR_3_PIN             3U            /* PA3 - TIM2_CH4 (Rear-Left, CW) */
#define MOTOR_4_PIN             2U            /* PA2 - TIM2_CH3 (Front-Left, CCW) */

/* UART Ports */
#define RC_UART_BAUDRATE        420000U       /* UART1: CRSF (PA9 TX / PA10 RX) */
#define TELEMETRY_BAUDRATE      115200U       /* UART6: MATLAB Ground Station (PC6 TX / PC7 RX) */

/* Analog Sensing (ADC1) */
#define VBAT_ADC_PIN            0U            /* PC0 - ADC1_IN10 (11:1 divider) */
#define CURR_ADC_PIN            1U            /* PC1 - ADC1_IN11 */

/* Status LED & Buzzer */
#define LED_PIN                 15U           /* PC15 */
#define BUZZER_PIN              14U           /* PC14 */

#endif /* CONFIG_H */
