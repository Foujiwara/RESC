/*
 * clock.c — 168 MHz depuis le quartz de 8 MHz (HSE), repli sur HSI 16 MHz.
 * AHB 168 MHz, APB1 42 MHz, APB2 84 MHz, USB 48 MHz (PLLQ = 7).
 */
#include "clock.h"
#include "stm32.h"

static bool s_hse_ok;

bool clock_init(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    RCC->CR |= RCC_CR_HSEON;
    uint32_t t = 0;
    while (!(RCC->CR & RCC_CR_HSERDY) && t < 500000u) t++;
    s_hse_ok = (RCC->CR & RCC_CR_HSERDY) != 0;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();
    PWR->CR |= PWR_CR_VOS;

    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    uint32_t pllm = s_hse_ok ? 8u : 16u;
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }
    RCC->PLLCFGR = (pllm << RCC_PLLCFGR_PLLM_Pos) | (336u << RCC_PLLCFGR_PLLN_Pos) |
                   (0u << RCC_PLLCFGR_PLLP_Pos) | (7u << RCC_PLLCFGR_PLLQ_Pos) |
                   (s_hse_ok ? RCC_PLLCFGR_PLLSRC_HSE : RCC_PLLCFGR_PLLSRC_HSI);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    /* Detection de panne d'horloge : le CSS declenche aussi le break de TIM1 */
    if (s_hse_ok) RCC->CR |= RCC_CR_CSSON;
    return s_hse_ok;
}

bool clock_hse_ok(void) { return s_hse_ok; }
