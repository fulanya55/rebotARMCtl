/* Reset entry and vector table for the STM32G474. */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "stm32g4xx.h"

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);

uint32_t SystemCoreClock = 16000000u;

void Reset_Handler(void);
void Default_Handler(void);

void Default_Handler(void)
{
    /* An unexpected fault: stop kicking the watchdog. The reset that follows drops
     * the safety relay, because its enable line is pulled down off-chip. */
    for (;;) {
    }
}

#define WEAK_HANDLER __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK_HANDLER;
void HardFault_Handler(void) WEAK_HANDLER;
void MemManage_Handler(void) WEAK_HANDLER;
void BusFault_Handler(void) WEAK_HANDLER;
void UsageFault_Handler(void) WEAK_HANDLER;
void SVC_Handler(void) WEAK_HANDLER;
void DebugMon_Handler(void) WEAK_HANDLER;
void PendSV_Handler(void) WEAK_HANDLER;
void SysTick_Handler(void) WEAK_HANDLER;
void FDCAN2_IT0_IRQHandler(void) WEAK_HANDLER;
void FDCAN3_IT0_IRQHandler(void) WEAK_HANDLER;
void USART2_IRQHandler(void) WEAK_HANDLER;
void USART3_IRQHandler(void) WEAK_HANDLER;
void USB_HP_IRQHandler(void) WEAK_HANDLER;
void USB_LP_IRQHandler(void) WEAK_HANDLER;
void USBWakeUp_IRQHandler(void) WEAK_HANDLER;

typedef void (*vector_t)(void);

/* 16 core exceptions, then the device interrupts up to FMAC (101). Anything not
 * named lands in Default_Handler. */
#define IRQ(n) [16 + (n)]
#pragma clang diagnostic ignored "-Winitializer-overrides"
__attribute__((section(".isr_vector"), used)) const vector_t vector_table[16 + 102] = {
    [0] = (vector_t)&_estack,
    [1] = Reset_Handler,
    [2] = NMI_Handler,
    [3] = HardFault_Handler,
    [4] = MemManage_Handler,
    [5] = BusFault_Handler,
    [6] = UsageFault_Handler,
    [11] = SVC_Handler,
    [12] = DebugMon_Handler,
    [14] = PendSV_Handler,
    [15] = SysTick_Handler,
    [16 ... 117] = Default_Handler,
    IRQ(USB_HP_IRQn) = USB_HP_IRQHandler,
    IRQ(USB_LP_IRQn) = USB_LP_IRQHandler,
    IRQ(USART2_IRQn) = USART2_IRQHandler,
    IRQ(USART3_IRQn) = USART3_IRQHandler,
    IRQ(USBWakeUp_IRQn) = USBWakeUp_IRQHandler,
    IRQ(FDCAN2_IT0_IRQn) = FDCAN2_IT0_IRQHandler,
    IRQ(FDCAN3_IT0_IRQn) = FDCAN3_IT0_IRQHandler,
};

void Reset_Handler(void)
{
    /* The FPU has to be on before any code that may touch float registers. */
    SCB->CPACR |= (3u << 20) | (3u << 22);
    __DSB();
    __ISB();
    SCB->VTOR = (uint32_t)&vector_table[0];

    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }
    for (dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0u;
    }
    (void)main();
    Default_Handler();
}
