/*
 * detect_all.h — Detection automatique FOC (assistant de VESC Tool, COMM_DETECT_APPLY_ALL_FOC).
 *
 * Enchaine les detections RESC : R/L, flux, sondes Hall, puis codeur absolu s'il est configure ;
 * courants deduits de la puissance de pertes admise (courant max = sqrt(P / 1.5 R), flux a
 * courant max / 2.5, capteurs a courant max / 3). Applique et enregistre R, L, Ld - Lq, lambda,
 * gains, courants max, source d'angle ; en cas d'echec, la configuration precedente est remise.
 * Avec detect_can, les autres controleurs du bus font la meme detection en parallele.
 *
 * Resultat (comme VESC) : 0 sans capteur, 1 sondes Hall, 2 codeur ; < 0 erreur
 * (defaut - 100, -10 flux, -50 delai CAN, -51 echec d'un autre controleur).
 */
#ifndef DETECT_ALL_H
#define DETECT_ALL_H

#include <stdbool.h>
#include <stdint.h>
#include "packet.h"

/* COMM_DETECT_APPLY_ALL_FOC : [detect_can u8][puissance f32 1e3][i_in_min][i_in_max][openloop_rpm][sl_erpm] */
void detect_all_request(const comm_iface_t *reply, const uint8_t *d, uint32_t n);
/* CAN_PACKET_DETECT_APPLY_ALL_FOC recu d'un maitre : detection locale, reponse a 'master' */
void detect_all_can_request(uint8_t master, bool activate_status, float max_power_loss);
/* CAN_PACKET_DETECT_APPLY_ALL_FOC_RES recu */
void detect_all_can_result(int8_t res);
void detect_all_poll(uint32_t now_ms);
bool detect_all_busy(void);

#endif
