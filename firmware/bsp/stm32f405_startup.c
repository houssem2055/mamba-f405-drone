/**
 * @file    stm32f405_startup.c
 * @brief   Minimal ARM Cortex-M4 Vector Table & Reset Handler for STM32F405RGT6.
 */

#include <stdint.h>

extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

extern int  main(void);
extern void SysTick_Handler(void);
extern void USART1_IRQHandler(void);
extern void USART6_IRQHandler(void);

void Default_Handler(void) {
    while (1) {}
}

void Reset_Handler(void) {
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) { *dst++ = *src++; }

    dst = &_sbss;
    while (dst < &_ebss) { *dst++ = 0UL; }

    main();
    while (1) {}
}

__attribute__((section(".isr_vector")))
void (*const g_pfnVectors[])(void) = {
    (void (*)(void))(&_estack),
    Reset_Handler,
    Default_Handler, /* NMI */
    Default_Handler, /* HardFault */
    Default_Handler, /* MemManage */
    Default_Handler, /* BusFault */
    Default_Handler, /* UsageFault */
    0, 0, 0, 0,
    Default_Handler, /* SVCall */
    Default_Handler, /* DebugMon */
    0,
    Default_Handler, /* PendSV */
    SysTick_Handler, /* SysTick */
    /* External IRQs (0..71) */
    [16 + 37] = USART1_IRQHandler,
    [16 + 71] = USART6_IRQHandler
};
