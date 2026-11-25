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
#define MAX_ARM_TILT_DEG        25.0f         /* Prevent arming if drone is tilted > 25 deg */
#define MIN_THROTTLE_FOR_ARM    0.05f         /* Throttle must be at lowest position to arm */
#define RC_FAILSAFE_TIMEOUT_MS  300           /* Failsafe disarm if no RC packet for 300 ms */

#define MOTOR_MIN_PWM_US        1000.0f       /* Disarmed / Off pulse width */
#define MOTOR_IDLE_PWM_US       1060.0f       /* Armed idle spin pulse width */
#define MOTOR_MAX_PWM_US        2000.0f       /* Full power pulse width */

/* ========================================================================== */
/*                    MAMBA F405 MK2 V2 PIN DEFINITIONS                       */
/* ========================================================================== */

/* IMU (MPU6000) on SPI1 */
#define MPU_SPI                 SPI1
#define MPU_SCK_PIN             GPIO_PIN_5    /* PA5 */
#define MPU_MISO_PIN            GPIO_PIN_6    /* PA6 */
#define MPU_MOSI_PIN            GPIO_PIN_7    /* PA7 */
#define MPU_SPI_GPIO_PORT       GPIOA
#define MPU_CS_PIN              GPIO_PIN_2    /* PC2 (Active LOW) */
#define MPU_CS_GPIO_PORT        GPIOC

/* Motor PWM Outputs (Quad-X) */
#define MOTOR_1_PIN             GPIO_PIN_0    /* PB0 - TIM3_CH3 (Rear-Right, CCW) */
#define MOTOR_2_PIN             GPIO_PIN_1    /* PB1 - TIM3_CH4 (Front-Right, CW) */
#define MOTOR_3_PIN             GPIO_PIN_3    /* PA3 - TIM2_CH4 (Rear-Left, CW) */
#define MOTOR_4_PIN             GPIO_PIN_2    /* PA2 - TIM2_CH3 (Front-Left, CCW) */

/* UART Ports */
/* UART1: RC Receiver (CRSF / SBUS) */
#define RC_UART                 USART1
#define RC_RX_PIN               GPIO_PIN_10   /* PA10 */
#define RC_TX_PIN               GPIO_PIN_9    /* PA9 */

/* UART6: MATLAB Telemetry Ground Station */
#define TELEMETRY_UART          USART6
#define TELEMETRY_TX_PIN        GPIO_PIN_6    /* PC6 */
#define TELEMETRY_RX_PIN        GPIO_PIN_7    /* PC7 */
#define TELEMETRY_BAUDRATE      115200

/* Status LED & Buzzer */
#define LED_PIN                 GPIO_PIN_15   /* PC15 */
#define LED_GPIO_PORT           GPIOC
#define BUZZER_PIN              GPIO_PIN_14   /* PC14 */
#define BUZZER_GPIO_PORT        GPIOC

#endif /* CONFIG_H */
