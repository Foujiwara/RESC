/*
 * inputs.h — Entrees des applications : servo/PPM, tensions ADC externes,
 * boutons et capteur de pedalage (PAS) sur les broches du port UART.
 */
#ifndef INPUTS_H
#define INPUTS_H

#include <stdbool.h>
#include <stdint.h>

/* Utilisation des broches du port UART (TX, RX) */
typedef enum {
    IN_UART_COMM = 0,      /* liaison serie (paquets) */
    IN_UART_BUTTONS,       /* TX : regulateur de vitesse, RX : marche arriere (actifs a 0) */
    IN_UART_PAS            /* capteur de pedalage en quadrature : TX = A, RX = B */
} in_uart_use_t;

/* servo_pin_owned : la broche servo sert a un autre usage (codeur PWM), n'y pas toucher */
void inputs_init(bool servo, in_uart_use_t uart_use, bool servo_pin_owned);

/* Derniere impulsion servo (ms) ; false si aucune impulsion depuis 100 ms */
bool inputs_servo_ms(float *ms, uint32_t now_ms);
/* Niveau logique de la broche servo (interrupteur d'arret) */
bool inputs_servo_level(void);

/* Tension sur les entrees ADC externes 1 et 2 (V) */
float inputs_adc_v(int n);

/* Boutons (true = appuye) */
bool inputs_btn_cruise(void);
bool inputs_btn_reverse(void);

/* Capteur de pedalage : a appeler a 1 kHz ; renvoie les fronts signes depuis le dernier appel */
int32_t inputs_pas_poll(void);

#endif
