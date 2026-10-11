/*
 * apps.h — Applications de commande RESC : PPM, ADC, VESC Remote (manette), PAS,
 * combinaisons avec l'UART, commande de plusieurs controleurs par CAN, antipatinage,
 * interrupteur d'arret, demarrage securise.
 */
#ifndef APPS_H
#define APPS_H

#include <stdbool.h>
#include <stdint.h>

/* (Re)configure les entrees selon app_to_use et kill_sw_mode (boucle principale) */
void apps_configure(void);
/* Pas des applications, 1 kHz */
void apps_tick_1khz(uint32_t now_ms);

/* Donnees de la manette (COMM_SET_CHUCK_DATA) */
void apps_set_chuck(const uint8_t *d, uint32_t n, uint32_t now_ms);
/* COMM_APP_DISABLE_OUTPUT : ms > 0 : coupe les sorties ce temps ; < 0 : jusqu'a nouvel ordre ; 0 : reactive */
void apps_disable_output(int32_t ms, uint32_t now_ms);

/* Valeurs decodees pour l'outil de configuration */
float apps_ppm_decoded(void);      /* -1..1 */
float apps_ppm_pulse_ms(void);
float apps_adc_decoded(int n);     /* n = 1, 2 */
float apps_adc_voltage(int n);
float apps_chuk_decoded(void);     /* -1..1 */

/* Derniere activite (commande recue, entree active, moteur en mouvement), pour l'arret automatique */
uint32_t apps_last_activity_ms(void);
void apps_note_activity(uint32_t now_ms);

#endif
