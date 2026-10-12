#ifndef WDG_H
#define WDG_H
#include <stdbool.h>
#include <stdint.h>
void wdg_init(void);           /* IWDG ~50 ms */
void wdg_kick(void);
void wdg_slow(bool slow);      /* ~16 s pendant les operations flash */
bool wdg_was_reset(void);      /* reset precedent cause par le watchdog */
uint32_t wdg_reset_flags(void);  /* RCC->CSR au demarrage (cause du dernier reset) */
uint32_t wdg_boot_count(void);
bool hardfault_info(uint32_t out[4]);   /* startup.c : plantage enregistre (pc, lr, cfsr, hfsr) */   /* demarrages depuis la derniere mise sous tension franche */
#endif
