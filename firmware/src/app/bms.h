/*
 * bms.h — BMS VESC sur le bus CAN (bms.* de la configuration moteur).
 *
 * Lit les trames du BMS (tensions, courants, cellules, equilibrage, temperatures, humidite,
 * etat de charge, compteurs, etat texte), retient pour les limites le BMS le plus chaud, le
 * moins charge, la cellule la plus basse et la plus haute (2 s de validite), et reduit le
 * courant batterie : temperature (decharge et regeneration), etat de charge et cellule basse
 * (decharge), cellule haute (regeneration). Relaie les trames vers l'outil (COMM_BMS_FWD_CAN_RX)
 * et les commandes de l'outil vers le BMS (COMM_BMS_*).
 */
#ifndef BMS_H
#define BMS_H

#include <stdbool.h>
#include <stdint.h>
#include "packet.h"

/* trame CAN etendue recue (commande, emetteur) ; true si elle concerne le BMS */
bool bms_can_frame(uint8_t cmd, uint8_t id, const uint8_t *d, uint8_t len, uint32_t now_ms);
/* limites de courant batterie a appliquer (appele toutes les 10 ms) */
void bms_update(uint32_t now_ms);
/* commandes de l'outil (COMM_BMS_GET_VALUES et commandes relayees au BMS) */
void bms_command(const comm_iface_t *src, const uint8_t *data, uint32_t len, uint32_t now_ms);
/* interface vers laquelle relayer les trames (derniere interface de l'outil, USB) */
void bms_set_fwd_iface(const comm_iface_t *any, const comm_iface_t *usb);

#endif
