#ifndef SYSTIME_H
#define SYSTIME_H
#include <stdint.h>
void systime_init(void);          /* SysTick 1 kHz */
uint32_t systime_ms(void);
/* appele a 1 kHz depuis l'interruption SysTick (defini par l'application) */
void app_tick_1khz(void);
#endif
