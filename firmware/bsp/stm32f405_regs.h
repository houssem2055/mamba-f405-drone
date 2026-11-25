#ifndef STM32F405_REGS_H
#define STM32F405_REGS_H

#include <stdint.h>

/* Minimal self-contained STM32F405RGT6 CMSIS/Register definitions */
#define __IO volatile

#define PERIPH_BASE           0x40000000UL
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define SCS_BASE              0xE000E000UL

/* Core SysTick & NVIC */
typedef struct {
    __IO uint32_t CTRL;
    __IO uint32_t LOAD;
    __IO uint32_t VAL;
    __IO uint32_t CALIB;
} SysTick_Type;
#define SysTick ((SysTick_Type *)(SCS_BASE + 0x0010UL))

typedef struct {
    __IO uint32_t ISER[8];
} NVIC_Type;
#define NVIC ((NVIC_Type *)(SCS_BASE + 0x0100UL))

#define SCB_CPACR (*((__IO uint32_t *)(SCS_BASE + 0x0D88UL)))

/* FLASH */
typedef struct {
    __IO uint32_t ACR;
} FLASH_TypeDef;
#define FLASH ((FLASH_TypeDef *)(AHB1PERIPH_BASE + 0x3C00UL))

/* RCC */
typedef struct {
    __IO uint32_t CR;
    __IO uint32_t PLLCFGR;
    __IO uint32_t CFGR;
    __IO uint32_t CIR;
    __IO uint32_t AHB1RSTR;
    __IO uint32_t AHB2RSTR;
    __IO uint32_t AHB3RSTR;
    uint32_t      RESERVED0;
    __IO uint32_t APB1RSTR;
    __IO uint32_t APB2RSTR;
    uint32_t      RESERVED1[2];
    __IO uint32_t AHB1ENR;
    __IO uint32_t AHB2ENR;
    __IO uint32_t AHB3ENR;
    uint32_t      RESERVED2;
    __IO uint32_t APB1ENR;
    __IO uint32_t APB2ENR;
} RCC_TypeDef;
#define RCC ((RCC_TypeDef *)(AHB1PERIPH_BASE + 0x3800UL))

/* PWR */
typedef struct {
    __IO uint32_t CR;
    __IO uint32_t CSR;
} PWR_TypeDef;
#define PWR ((PWR_TypeDef *)(APB1PERIPH_BASE + 0x7000UL))

/* GPIO */
typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t OTYPER;
    __IO uint32_t OSPEEDR;
    __IO uint32_t PUPDR;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
    __IO uint32_t BSRR;
    __IO uint32_t LCKR;
    __IO uint32_t AFR[2];
} GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef *)(AHB1PERIPH_BASE + 0x0000UL))
#define GPIOB ((GPIO_TypeDef *)(AHB1PERIPH_BASE + 0x0400UL))
#define GPIOC ((GPIO_TypeDef *)(AHB1PERIPH_BASE + 0x0800UL))

/* SPI */
typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SR;
    __IO uint32_t DR;
} SPI_TypeDef;
#define SPI1 ((SPI_TypeDef *)(APB2PERIPH_BASE + 0x3000UL))

/* TIM */
typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CCMR1;
    __IO uint32_t CCMR2;
    __IO uint32_t CCER;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    uint32_t      RESERVED0;
    __IO uint32_t CCR1;
    __IO uint32_t CCR2;
    __IO uint32_t CCR3;
    __IO uint32_t CCR4;
} TIM_TypeDef;
#define TIM2 ((TIM_TypeDef *)(APB1PERIPH_BASE + 0x0000UL))
#define TIM3 ((TIM_TypeDef *)(APB1PERIPH_BASE + 0x0400UL))

/* USART */
typedef struct {
    __IO uint32_t SR;
    __IO uint32_t DR;
    __IO uint32_t BRR;
    __IO uint32_t CR1;
} USART_TypeDef;
#define USART1 ((USART_TypeDef *)(APB2PERIPH_BASE + 0x1000UL))
#define USART6 ((USART_TypeDef *)(APB2PERIPH_BASE + 0x1400UL))

/* ADC */
typedef struct {
    __IO uint32_t SR;
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMPR1;
    __IO uint32_t SMPR2;
    uint32_t      RESERVED0[4];
    __IO uint32_t SQR1;
    __IO uint32_t SQR2;
    __IO uint32_t SQR3;
    uint32_t      RESERVED1[6];
    __IO uint32_t DR;
} ADC_TypeDef;
#define ADC1 ((ADC_TypeDef *)(APB2PERIPH_BASE + 0x2000UL))

#endif /* STM32F405_REGS_H */
