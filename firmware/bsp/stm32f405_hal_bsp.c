/**
 * @file    stm32f405_hal_bsp.c
 * @brief   Bare-metal Register-Level Board Support Package (BSP) for
 *          Diatone MAMBA F405 MK2 V2 (STM32F405RGT6 @ 168 MHz).
 *
 *          Zero external SDK dependencies required — directly configures:
 *            - RCC PLL (8 MHz HSE -> 168 MHz SYSCLK, 84 MHz APB2, 42 MHz APB1)
 *            - FPU CP10/CP11 full access
 *            - SysTick 1000 Hz interrupt
 *            - SPI1 (PA5, PA6, PA7) + PC2 CS for MPU6000
 *            - TIM3 CH3/CH4 (PB0, PB1) + TIM2 CH3/CH4 (PA2, PA3) 400 Hz PWM
 *            - USART1 (PA10 RX @ 420k baud CRSF) with RXNE IRQ
 *            - USART6 (PC6 TX / PC7 RX @ 115200 baud Telemetry) with RXNE IRQ
 *            - ADC1 (PC0 Ch10) for 4S LiPo voltage sense
 *            - GPIO PC15 Status LED & PC14 Buzzer
 */

#include "stm32f405_regs.h"
#include "config.h"
#include "rc_receiver.h"
#include "telemetry.h"

extern uint32_t System_Millis(void);

static void GPIO_SetAltFunc(GPIO_TypeDef *port, uint8_t pin, uint8_t af) {
    port->MODER   = (port->MODER & ~(3UL << (pin * 2U))) | (2UL << (pin * 2U));
    port->OSPEEDR |= (3UL << (pin * 2U)); /* High speed */
    uint8_t idx = pin >> 3U;
    uint8_t shift = (uint8_t)((pin & 7U) * 4U);
    port->AFR[idx] = (port->AFR[idx] & ~(0xFUL << shift)) | ((uint32_t)af << shift);
}

static void GPIO_SetOutput(GPIO_TypeDef *port, uint8_t pin) {
    port->MODER = (port->MODER & ~(3UL << (pin * 2U))) | (1UL << (pin * 2U));
    port->OSPEEDR |= (2UL << (pin * 2U));
}

static void SystemClock_Config_168MHz(void) {
    /* Enable FPU (CP10 & CP11 full access) */
    SCB_CPACR |= (0xFUL << 20);

    /* Enable PWR clock and set voltage regulator scale 1 */
    RCC->APB1ENR |= (1UL << 28);
    PWR->CR |= (1UL << 14);

    /* Enable HSE (8 MHz crystal on Mamba F405) */
    RCC->CR |= (1UL << 16);
    while (!(RCC->CR & (1UL << 17))) {}

    /* Configure Flash prefetch, instruction/data cache, and 5 wait states */
    FLASH->ACR = (1UL << 8) | (1UL << 9) | (1UL << 10) | 5UL;

    /* Configure PLL: HSE source (bit 22), M=8, N=336, P=2 (PLLP=0), Q=7 */
    RCC->PLLCFGR = (8UL) | (336UL << 6) | (0UL << 16) | (1UL << 22) | (7UL << 24);
    RCC->CR |= (1UL << 24); /* PLLON */
    while (!(RCC->CR & (1UL << 25))) {}

    /* Prescalers: AHB=/1 (168MHz), APB1=/4 (42MHz), APB2=/2 (84MHz) */
    RCC->CFGR = (0x5UL << 10) | (0x4UL << 13);
    /* Switch SYSCLK to PLL */
    RCC->CFGR |= 0x2UL;
    while ((RCC->CFGR & 0xCUL) != 0x8UL) {}

    /* SysTick at 1 kHz (168,000,000 / 1000 = 168,000 cycles) */
    SysTick->LOAD = (MCU_CLOCK_HZ / 1000UL) - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = 0x07UL; /* CLKSOURCE=AHB, TICKINT=1, ENABLE=1 */
}

void System_Hardware_Init(void) {
    SystemClock_Config_168MHz();

    /* Enable GPIOA, GPIOB, GPIOC clocks */
    RCC->AHB1ENR |= (1UL << 0) | (1UL << 1) | (1UL << 2);

    /* LED (PC15), Buzzer (PC14), MPU6000 CS (PC2) */
    GPIO_SetOutput(GPIOC, LED_PIN);
    GPIO_SetOutput(GPIOC, BUZZER_PIN);
    GPIO_SetOutput(GPIOC, MPU_CS_PIN);
    GPIOC->BSRR = (1UL << MPU_CS_PIN); /* CS High (deselected) */

    /* SPI1 on PA5 (SCK), PA6 (MISO), PA7 (MOSI) -> AF5 */
    RCC->APB2ENR |= (1UL << 12);
    GPIO_SetAltFunc(GPIOA, MPU_SCK_PIN,  5U);
    GPIO_SetAltFunc(GPIOA, MPU_MISO_PIN, 5U);
    GPIO_SetAltFunc(GPIOA, MPU_MOSI_PIN, 5U);
    /* Master, CPOL=1, CPHA=1 (SPI Mode 3), BR=/16 (~5.25 MHz -> safe for MPU6000 registers), SSM+SSI */
    SPI1->CR1 = (1UL << 0) | (1UL << 1) | (1UL << 2) | (3UL << 3) | (1UL << 8) | (1UL << 9) | (1UL << 6);

    /* Motor PWM Timers: TIM3 (PB0=CH3, PB1=CH4, AF2) & TIM2 (PA2=CH3, PA3=CH4, AF1) */
    /* Timer input clock on APB1 is 84 MHz -> PSC = 83 gives 1 MHz (1 us tick); ARR = 2499 gives 400 Hz */
    RCC->APB1ENR |= (1UL << 0) | (1UL << 1);
    GPIO_SetAltFunc(GPIOB, MOTOR_1_PIN, 2U);
    GPIO_SetAltFunc(GPIOB, MOTOR_2_PIN, 2U);
    GPIO_SetAltFunc(GPIOA, MOTOR_3_PIN, 1U);
    GPIO_SetAltFunc(GPIOA, MOTOR_4_PIN, 1U);

    TIM3->PSC   = 83U;
    TIM3->ARR   = 2499U;
    TIM3->CCMR2 = (6UL << 4) | (1UL << 3) | (6UL << 12) | (1UL << 11); /* PWM Mode 1 + Preload CH3/CH4 */
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

    /* USART1 (PA9 TX / PA10 RX, AF7) -> CRSF 420,000 baud on 84 MHz APB2 */
    RCC->APB2ENR |= (1UL << 4);
    GPIO_SetAltFunc(GPIOA, 9U,  7U);
    GPIO_SetAltFunc(GPIOA, 10U, 7U);
    USART1->BRR = (84000000UL / RC_UART_BAUDRATE);
    USART1->CR1 = (1UL << 13) | (1UL << 5) | (1UL << 2); /* UE, RXNEIE, RE */
    NVIC->ISER[37U >> 5U] = (1UL << (37U & 31U));       /* Enable USART1 IRQ */

    /* USART6 (PC6 TX / PC7 RX, AF8) -> Telemetry 115,200 baud on 84 MHz APB2 */
    RCC->APB2ENR |= (1UL << 5);
    GPIO_SetAltFunc(GPIOC, 6U, 8U);
    GPIO_SetAltFunc(GPIOC, 7U, 8U);
    USART6->BRR = (84000000UL / TELEMETRY_BAUDRATE);
    USART6->CR1 = (1UL << 13) | (1UL << 5) | (1UL << 3) | (1UL << 2); /* UE, RXNEIE, TE, RE */
    NVIC->ISER[71U >> 5U] = (1UL << (71U & 31U));       /* Enable USART6 IRQ */

    /* IMU PA4 CS candidate (for ICM-42688-P variants) */
    GPIO_SetOutput(GPIOA, 4U);
    GPIOA->BSRR = (1UL << 4U);

    /* ADC1 on PC0 (Ch 10), PC1 (Ch 11) & PC3 (Ch 13) for auto-detected V_BAT sense */
    RCC->APB2ENR |= (1UL << 8);
    GPIOC->MODER |= (3UL << (VBAT_ADC_PIN * 2U)) | (3UL << (CURR_ADC_PIN * 2U)) | (3UL << (3U * 2U)); /* Analog mode */
    ADC1->SMPR1 |= (4UL << 0) | (4UL << 3) | (4UL << 9); /* 84-cycle sample time on Ch 10, 11, 13 */
    ADC1->SQR3 = 10UL;
    ADC1->CR2  = (1UL << 0); /* ADON */
}

static uint8_t s_imu_cs_pin_bsp = 0U; /* 0 = unprobed, 4 = PA4, 2 = PC2 */
extern uint8_t SPI1_TransferByte(uint8_t byte);

static void BSP_AutoDetect_IMU_CS(void) {
    /* Probe PA4 first (ICM-42688-P = 0x47, MPU6000 = 0x68) */
    GPIOA->BSRR = (1UL << (4U + 16U));
    (void)SPI1_TransferByte(0x75U | 0x80U);
    uint8_t id_pa4 = SPI1_TransferByte(0x00U);
    GPIOA->BSRR = (1UL << 4U);

    if (id_pa4 == 0x47U || id_pa4 == 0x68U) {
        s_imu_cs_pin_bsp = 4U;
        return;
    }

    /* Fallback to PC2 */
    s_imu_cs_pin_bsp = 2U;
}

void SPI1_Select(bool select) {
    if (s_imu_cs_pin_bsp == 0U) {
        BSP_AutoDetect_IMU_CS();
    }
    if (s_imu_cs_pin_bsp == 4U) {
        if (select) {
            GPIOA->BSRR = (1UL << (4U + 16U)); /* PA4 LOW */
        } else {
            GPIOA->BSRR = (1UL << 4U);         /* PA4 HIGH */
        }
    } else {
        if (select) {
            GPIOC->BSRR = (1UL << (MPU_CS_PIN + 16U)); /* Reset LOW */
        } else {
            GPIOC->BSRR = (1UL << MPU_CS_PIN);         /* Set HIGH */
        }
    }
}

uint8_t SPI1_TransferByte(uint8_t byte) {
    while (!(SPI1->SR & (1UL << 1))) {} /* TXE */
    SPI1->DR = byte;
    while (!(SPI1->SR & (1UL << 0))) {} /* RXNE */
    return (uint8_t)SPI1->DR;
}

void SPI1_TransferBuffer(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        uint8_t tx = tx_buf ? tx_buf[i] : 0x00U;
        uint8_t rx = SPI1_TransferByte(tx);
        if (rx_buf) {
            rx_buf[i] = rx;
        }
    }
}

void Delay_Ms(uint32_t ms) {
    uint32_t start = System_Millis();
    while ((System_Millis() - start) < ms) {}
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
    while (*str) {
        while (!(USART6->SR & (1UL << 7))) {} /* TXE */
        USART6->DR = (uint8_t)(*str++);
    }
}

static uint32_t ADC_Read_Channel(uint32_t ch) {
    ADC1->SQR3 = ch;
    ADC1->CR2 |= (1UL << 30); /* SWSTART */
    while (!(ADC1->SR & (1UL << 1))) {} /* EOC */
    (void)ADC1->DR;
    /* Second read after channel mux settles high-impedance voltage divider */
    ADC1->CR2 |= (1UL << 30); /* SWSTART */
    while (!(ADC1->SR & (1UL << 1))) {} /* EOC */
    return ADC1->DR & 0xFFFU;
}

float ADC_Read_Vbat_Volts(void) {
    static uint8_t s_vbat_ch = 0U; /* 0 = auto-detect, 10 = PC0, 11 = PC1, 13 = PC3 */
    uint32_t raw;

    if (s_vbat_ch == 0U) {
        uint32_t raw_c1 = ADC_Read_Channel(11UL);
        uint32_t raw_c0 = ADC_Read_Channel(10UL);
        uint32_t raw_c3 = ADC_Read_Channel(13UL);
        if (raw_c1 >= 250U && raw_c0 < 150U && raw_c3 < 150U) {
            s_vbat_ch = 11U;
            raw = raw_c1;
        } else if (raw_c0 >= 250U && raw_c1 < 150U && raw_c3 < 150U) {
            s_vbat_ch = 10U;
            raw = raw_c0;
        } else if (raw_c3 >= 250U && raw_c1 < 150U && raw_c0 < 150U) {
            s_vbat_ch = 13U;
            raw = raw_c3;
        } else if (s_imu_cs_pin_bsp == 4U && raw_c3 >= 250U) {
            s_vbat_ch = 13U;
            raw = raw_c3;
        } else if (s_imu_cs_pin_bsp == 4U && raw_c1 >= 250U) {
            s_vbat_ch = 11U;
            raw = raw_c1;
        } else {
            raw = (raw_c3 > raw_c1) ? raw_c3 : raw_c1;
            if (raw_c0 > raw) raw = raw_c0;
        }
    } else {
        raw = ADC_Read_Channel((uint32_t)s_vbat_ch);
    }

    /* 11:1 voltage divider on MAMBA F405 Vbat pin */
    return ((float)raw / 4095.0f) * 3.3f * 11.0f;
}

void Status_LED_Set(bool on) {
    if (on) GPIOC->BSRR = (1UL << LED_PIN);
    else    GPIOC->BSRR = (1UL << (LED_PIN + 16U));
}

void Buzzer_Set(bool on) {
    if (on) GPIOC->BSRR = (1UL << BUZZER_PIN);
    else    GPIOC->BSRR = (1UL << (BUZZER_PIN + 16U));
}

void USART1_IRQHandler(void) {
    if (USART1->SR & (1UL << 5)) {
        uint8_t b = (uint8_t)USART1->DR;
        RC_Receiver_Process_Byte(b, System_Millis());
    }
}

void USART6_IRQHandler(void) {
    if (USART6->SR & (1UL << 5)) {
        uint8_t b = (uint8_t)USART6->DR;
        Telemetry_Process_Rx_Byte(b);
    }
}

void System_Enter_DFU_Bootloader(void) {
    /* Wait for UART6 TX complete so $ACK,BOOTLOADER finishes sending */
    while (!(USART6->SR & (1UL << 6))) {}

    /* Disable all interrupts */
    __asm__ volatile ("cpsid i" : : : "memory");

    /* Disable SysTick */
    SysTick->CTRL = 0UL;
    SysTick->LOAD = 0UL;
    SysTick->VAL  = 0UL;

    /* Clear all NVIC enabled & pending interrupts */
    for (uint32_t i = 0; i < 8; i++) {
        *(volatile uint32_t *)(uintptr_t)(0xE000E180UL + i * 4U) = 0xFFFFFFFFUL; /* ICER */
        *(volatile uint32_t *)(uintptr_t)(0xE000E280UL + i * 4U) = 0xFFFFFFFFUL; /* ICPR */
    }

    /* Reset RCC clocks to default HSI */
    RCC->CR |= (1UL << 0);     /* HSION */
    RCC->CFGR = 0UL;
    RCC->CR &= ~((1UL << 24) | (1UL << 16)); /* PLLOFF, HSEOFF */

    /* Enable SYSCFG clock and remap System Memory (0x1FFF0000) to 0x00000000 */
    RCC->APB2ENR |= (1UL << 14);
    *(volatile uint32_t *)0x40013800UL = 0x01UL; /* SYSCFG->MEMRMP = 0x01 */

    /* Set Main Stack Pointer and jump to STM32F405 ROM DFU Bootloader */
    uint32_t boot_msp   = *(volatile uint32_t *)0x1FFF0000UL;
    uint32_t boot_reset = *(volatile uint32_t *)0x1FFF0004UL;
    __asm__ volatile ("msr msp, %0" : : "r" (boot_msp) : "memory");
    __asm__ volatile ("cpsie i" : : : "memory");
    ((void (*)(void))(uintptr_t)boot_reset)();

    while (1) {}
}
