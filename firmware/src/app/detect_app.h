/*
 * detect_app.h — Detections manuelles RESC, declenchees par le protocole
 * (boutons de VESC Tool) ou par le terminal. La detection automatique (assistant de
 * VESC Tool) enchaine ces mesures : detect_all.c.
 */
#ifndef DETECT_APP_H
#define DETECT_APP_H
#include <stdbool.h>
#include "packet.h"

bool detect_app_start_offsets(const comm_iface_t *reply);
/* Moteur a induction (apres detect_rl) : Lm, Lr, Rr par echelon de courant continu */
bool detect_app_start_acim(const comm_iface_t *reply, float cur);
/* cur <= 0 : foc_resc_rl_frac x courant moteur max */
bool detect_app_start_rl(const comm_iface_t *reply, float cur);
bool detect_app_start_lambda(const comm_iface_t *reply, float current, float erpm_per_s, float duty, float r, float l);
bool detect_app_start_hall(const comm_iface_t *reply, float current);
bool detect_app_start_encoder(const comm_iface_t *reply, float current);
bool detect_app_busy(void);
int detect_app_last_error(void);          /* resultat de la derniere detection (0 = reussie) */
void detect_app_poll(uint32_t now_ms);

#endif
