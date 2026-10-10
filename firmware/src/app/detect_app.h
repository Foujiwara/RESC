/*
 * detect_app.h — Detections manuelles RESC, declenchees par le protocole
 * (boutons de VESC Tool) ou par le terminal. Pas de sequence automatique
 * globale ni d'auto-configuration : chaque mesure est lancee explicitement.
 */
#ifndef DETECT_APP_H
#define DETECT_APP_H
#include <stdbool.h>
#include "packet.h"

bool detect_app_start_offsets(const comm_iface_t *reply);
/* cur <= 0 : RESC_RL_CURRENT_FRAC x courant moteur max */
bool detect_app_start_rl(const comm_iface_t *reply, float cur);
bool detect_app_start_lambda(const comm_iface_t *reply, float current, float erpm_per_s, float duty, float r, float l);
bool detect_app_start_hall(const comm_iface_t *reply, float current);
bool detect_app_start_encoder(const comm_iface_t *reply, float current);
bool detect_app_busy(void);
void detect_app_poll(uint32_t now_ms);

#endif
