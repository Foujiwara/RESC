/*
 * stats.h — Statistiques de trajet (COMM_GET_STATS) et odometre persistant (COMM_SET_ODOMETER).
 *
 * Statistiques (comme VESC) : moyennes et maxima de vitesse, puissance batterie, courant moteur,
 * temperatures, duree depuis la remise a zero ; echantillonnees toutes les 10 ms.
 * Odometre : base enregistree en flash (avec la configuration des applications) + distance
 * absolue parcourue depuis le demarrage ; enregistre a l'arret et au redemarrage de la carte,
 * et quand il est modifie.
 */
#ifndef STATS_H
#define STATS_H

#include <stdint.h>

void stats_update(uint32_t now_ms);               /* boucle principale, toutes les 10 ms */
void stats_reset(uint32_t now_ms);
/* champs dans l'ordre de COMM_GET_STATS (bits 0..10) */
float stats_get(int field, uint32_t now_ms);

float stats_speed_ms(void);                       /* vitesse (m/s) d'apres la configuration du vehicule */
float stats_distance_m(void);                     /* distance signee depuis le demarrage */
float stats_distance_abs_m(void);
uint32_t stats_odometer_m(void);
void stats_set_odometer_m(uint32_t m);
/* base de l'odometre lue en flash / a enregistrer */
void stats_odometer_restore(uint32_t base_m);
uint32_t stats_odometer_to_store(void);

#endif
