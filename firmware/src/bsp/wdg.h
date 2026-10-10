#ifndef WDG_H
#define WDG_H
#include <stdbool.h>
#include <stdint.h>
void wdg_init(void);           /* IWDG ~50 ms */
void wdg_kick(void);
void wdg_slow(bool slow);      /* ~16 s pendant les operations flash */
bool wdg_was_reset(void);      /* reset precedent cause par le watchdog */
#endif
