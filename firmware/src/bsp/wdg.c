#include "wdg.h"
#include "stm32.h"

static bool s_wd_reset;
static bool s_started;
static uint32_t s_csr;
/* compteur conserve en RAM a travers les resets (perdu seulement a la coupure franche) */
__attribute__((section(".noinit"))) static volatile uint32_t s_boot_magic, s_boot_n;

static void set(uint32_t pr, uint32_t rlr)
{
    IWDG->KR = 0x5555u;
    while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) { }
    IWDG->PR = pr;
    IWDG->RLR = rlr;
    while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) { }
    IWDG->KR = 0xAAAAu;
}

void wdg_init(void)
{
    s_csr = RCC->CSR;
    s_wd_reset = (s_csr & RCC_CSR_IWDGRSTF) != 0;
    if (s_boot_magic != 0x424F4F54u || (s_csr & RCC_CSR_PORRSTF)) { s_boot_magic = 0x424F4F54u; s_boot_n = 0; }
    s_boot_n++;
    RCC->CSR |= RCC_CSR_RMVF;
    IWDG->KR = 0xCCCCu;
    set(2u, 100u);        /* LSI ~32 kHz / 16 = 2 kHz, 100 -> ~50 ms */
    s_started = true;
}

void wdg_kick(void) { if (s_started) IWDG->KR = 0xAAAAu; }

void wdg_slow(bool slow)
{
    if (!s_started) return;
    if (slow) set(6u, 0xFFFu);   /* /256 : ~32 s */
    else set(2u, 100u);
}

bool wdg_was_reset(void) { return s_wd_reset; }
uint32_t wdg_reset_flags(void) { return s_csr; }
uint32_t wdg_boot_count(void) { return s_boot_n; }
