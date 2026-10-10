#include "systime.h"
#include "stm32.h"

static volatile uint32_t s_ms;

void systime_init(void)
{
    SysTick->LOAD = SYSCLK_HZ / 1000u - 1u;
    SysTick->VAL = 0;
    NVIC_SetPriority(SysTick_IRQn, PRIO_SYSTICK);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

uint32_t systime_ms(void) { return s_ms; }

void SysTick_Handler(void)
{
    s_ms++;
    app_tick_1khz();
}
