/*
 * startup.c — Table des vecteurs et demarrage du STM32F405.
 *
 * Reset : maintien de l'alimentation de la carte (si la carte en a besoin),
 * copie .data et .ramfunc, mise a zero .bss et .ccmbss, FPU, puis main().
 * Toute exception inattendue coupe le pont avant toute autre action.
 */
#include <stdint.h>
#include "stm32.h"
#include "board.h"

extern uint32_t _sidata, _sdata, _edata;
extern uint32_t _siramfunc, _sramfunc, _eramfunc;
extern uint32_t _sbss, _ebss;
extern uint32_t _sccmbss, _eccmbss;
extern uint32_t _estack;

int main(void);
void Reset_Handler(void);

/* Coupure immediate du pont : MOE = 0 (sorties a l'etat de repos), driver desactive */
static inline void kill_bridge_raw(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    board_gate_enable_raw(false);
}

typedef struct {
    uint32_t magic;
    uint32_t cfsr, hfsr, pc, lr;
} fault_log_t;
__attribute__((section(".noinit"))) volatile fault_log_t g_hardfault_log;

void Default_Handler(void)
{
    kill_bridge_raw();
    for (;;) { }   /* le watchdog reinitialise la carte */
}

void HardFault_Handler_C(uint32_t *sp)
{
    kill_bridge_raw();
    g_hardfault_log.magic = 0x48464C54u;   /* 'HFLT' */
    g_hardfault_log.cfsr = SCB->CFSR;
    g_hardfault_log.hfsr = SCB->HFSR;
    g_hardfault_log.lr = sp[5];
    g_hardfault_log.pc = sp[6];
    for (;;) { }
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "tst lr, #4      \n"
        "ite eq          \n"
        "mrseq r0, msp   \n"
        "mrsne r0, psp   \n"
        "b HardFault_Handler_C \n");
}

#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK_ALIAS;
void MemManage_Handler(void) WEAK_ALIAS;
void BusFault_Handler(void) WEAK_ALIAS;
void UsageFault_Handler(void) WEAK_ALIAS;
void SVC_Handler(void) WEAK_ALIAS;
void DebugMon_Handler(void) WEAK_ALIAS;
void PendSV_Handler(void) WEAK_ALIAS;
void SysTick_Handler(void) WEAK_ALIAS;
void ADC_IRQHandler(void) WEAK_ALIAS;
void TIM1_BRK_TIM9_IRQHandler(void) WEAK_ALIAS;
void TIM1_UP_TIM10_IRQHandler(void) WEAK_ALIAS;
void USART1_IRQHandler(void) WEAK_ALIAS;
void USART2_IRQHandler(void) WEAK_ALIAS;
void USART3_IRQHandler(void) WEAK_ALIAS;
void UART4_IRQHandler(void) WEAK_ALIAS;
void UART5_IRQHandler(void) WEAK_ALIAS;
void USART6_IRQHandler(void) WEAK_ALIAS;
void CAN1_TX_IRQHandler(void) WEAK_ALIAS;
void CAN1_RX0_IRQHandler(void) WEAK_ALIAS;
void CAN1_SCE_IRQHandler(void) WEAK_ALIAS;
void OTG_FS_IRQHandler(void) WEAK_ALIAS;
void EXTI0_IRQHandler(void) WEAK_ALIAS;
void EXTI1_IRQHandler(void) WEAK_ALIAS;
void EXTI2_IRQHandler(void) WEAK_ALIAS;
void EXTI3_IRQHandler(void) WEAK_ALIAS;
void EXTI4_IRQHandler(void) WEAK_ALIAS;
void EXTI9_5_IRQHandler(void) WEAK_ALIAS;
void EXTI15_10_IRQHandler(void) WEAK_ALIAS;
void TIM2_IRQHandler(void) WEAK_ALIAS;

#pragma GCC diagnostic ignored "-Woverride-init"   /* remplissage par defaut puis vecteurs utilises */
#define NVEC (16 + 82)
typedef void (*vector_t)(void);

__attribute__((section(".isr_vector"), used))
const vector_t g_vectors[NVEC] = {
    [0] = (vector_t)&_estack,
    [1] = Reset_Handler,
    [2 ... NVEC - 1] = Default_Handler,
    [2] = NMI_Handler,
    [3] = HardFault_Handler,
    [4] = MemManage_Handler,
    [5] = BusFault_Handler,
    [6] = UsageFault_Handler,
    [11] = SVC_Handler,
    [12] = DebugMon_Handler,
    [14] = PendSV_Handler,
    [15] = SysTick_Handler,
    [16 + ADC_IRQn] = ADC_IRQHandler,
    [16 + TIM1_BRK_TIM9_IRQn] = TIM1_BRK_TIM9_IRQHandler,
    [16 + TIM1_UP_TIM10_IRQn] = TIM1_UP_TIM10_IRQHandler,
    [16 + USART1_IRQn] = USART1_IRQHandler,
    [16 + USART2_IRQn] = USART2_IRQHandler,
    [16 + USART3_IRQn] = USART3_IRQHandler,
    [16 + UART4_IRQn] = UART4_IRQHandler,
    [16 + UART5_IRQn] = UART5_IRQHandler,
    [16 + USART6_IRQn] = USART6_IRQHandler,
    [16 + CAN1_TX_IRQn] = CAN1_TX_IRQHandler,
    [16 + CAN1_RX0_IRQn] = CAN1_RX0_IRQHandler,
    [16 + CAN1_SCE_IRQn] = CAN1_SCE_IRQHandler,
    [16 + OTG_FS_IRQn] = OTG_FS_IRQHandler,
    [16 + EXTI0_IRQn] = EXTI0_IRQHandler,
    [16 + EXTI1_IRQn] = EXTI1_IRQHandler,
    [16 + EXTI2_IRQn] = EXTI2_IRQHandler,
    [16 + EXTI3_IRQn] = EXTI3_IRQHandler,
    [16 + EXTI4_IRQn] = EXTI4_IRQHandler,
    [16 + EXTI9_5_IRQn] = EXTI9_5_IRQHandler,
    [16 + EXTI15_10_IRQn] = EXTI15_10_IRQHandler,
    [16 + TIM2_IRQn] = TIM2_IRQHandler,
};

void Reset_Handler(void)
{
    /* 1. Maintien de l'alimentation, avant tout (Ubox : PA15) */
    board_power_hold_early();

    /* 2. Sections */
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    src = &_siramfunc;
    dst = &_sramfunc;
    while (dst < &_eramfunc) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;
    RCC->AHB1ENR |= RCC_AHB1ENR_CCMDATARAMEN;
    __DSB();
    for (dst = &_sccmbss; dst < &_eccmbss; ) *dst++ = 0;

    /* 3. FPU (CP10/CP11 plein acces), empilement paresseux par defaut */
    SCB->CPACR |= (0xFu << 20);
    __DSB();
    __ISB();

    main();
    for (;;) { }
}
