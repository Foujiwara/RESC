/* can_vesc.h — Protocole CAN compatible (identifiants etendus, statuts, transport de paquets) */
#ifndef CAN_VESC_H
#define CAN_VESC_H

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"

void can_vesc_init(void);
void can_vesc_poll(uint32_t now_ms);                  /* trames recues + statuts periodiques */
void can_vesc_send_buffer(uint8_t target, const uint8_t *d, uint32_t n, uint8_t send_mode);
void can_vesc_set_forward_iface(const comm_iface_t *iface);   /* destination des reponses relayees */
void can_vesc_start_ping_scan(const comm_iface_t *reply);
uint32_t can_baud_from_enum(uint8_t e);

/* Mode pont (can_mode COMM_BRIDGE) : interface vers laquelle relayer les trames recues */
void can_vesc_set_bridge_iface(const comm_iface_t *iface);
/* Trame brute (COMM_CAN_FWD_FRAME) */
void can_vesc_send_raw(uint32_t id, bool ext, const uint8_t *d, uint8_t n);
/* Coupure batterie diffusee a tous les controleurs (COMM_SET_BATTERY_CUT) */
void can_vesc_conf_battery_cut(bool store, float start, float end);
/* Nouvelle vitesse du bus pour tous (COMM_CAN_UPDATE_BAUD_ALL) : annonces, puis bascule apres
 * delay_ms ; false si la vitesse n'existe pas */
bool can_vesc_update_baud_all(uint32_t kbits, uint32_t delay_ms);
/* Controleurs actifs (statut recu depuis moins de 100 ms) */
int can_vesc_active_count(void);

/* Autres controleurs vus sur le bus (trame STATUS 1) */
typedef struct {
    uint8_t id;
    uint32_t ms;            /* derniere reception */
    float erpm, current, duty;
} can_peer_t;
#define CAN_PEERS_MAX 16
const can_peer_t *can_vesc_peers(int *n);

/* Consignes vers un autre controleur */
void can_vesc_send_current(uint8_t id, float amps);
void can_vesc_send_current_rel(uint8_t id, float rel);
void can_vesc_send_brake(uint8_t id, float amps);
void can_vesc_send_duty(uint8_t id, float duty);
void can_vesc_send_rpm(uint8_t id, float erpm);

#endif
