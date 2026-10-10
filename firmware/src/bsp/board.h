/* board.h — Fonctions de carte derivees de hw_config.h */
#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

void board_power_hold_early(void);     /* appele depuis Reset_Handler, avant .data/.bss */
void board_init(void);                 /* LED, filtres, enable driver (coupe), entrees capteurs */
void board_gate_enable(bool on);
void board_gate_enable_raw(bool on);   /* version sans etat, utilisable en exception */
bool board_gate_fault(void);           /* defaut signale par le driver */
void board_led(bool green, bool red);
void board_phase_filter(bool on);
void board_power_off(void);            /* relache le maintien d'alimentation */
bool board_power_key(void);            /* bouton (cartes a alimentation maintenue) */
uint8_t board_uid_hash(void);
const uint8_t *board_uid(void);        /* 12 octets */

#endif
