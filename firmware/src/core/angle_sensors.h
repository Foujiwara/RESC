/*
 * angle_sensors.h — Sources d'angle a capteur : sondes Hall et codeur incremental ABI.
 */
#ifndef ANGLE_SENSORS_H
#define ANGLE_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

/* ---- Hall : etat = H1 | H2 << 1 | H3 << 2 */
typedef struct {
    float angle[8];        /* centre electrique de chaque etat (rad) */
    bool  valid[8];
    float interp_omega;    /* rad/s : interpolation entre fronts au-dessus */
} hall_cfg_t;

typedef struct {
    int8_t prev;           /* etat precedent valide (-1 : aucun) */
    float dt_now;          /* s depuis le dernier front */
    float dt_last;         /* s, duree du dernier secteur, signee par le sens */
    float ang;             /* angle Hall (front, interpolation) */
    float omega;           /* rad/s el. (secteur / duree, decroit sans front) */
    float theta;           /* angle de sortie, vitesse de variation limitee */
    bool init;
    bool ok;               /* etat courant valide */
    uint32_t n_invalid;
} hall_t;

void hall_reset(hall_t *h);
void hall_update(hall_t *h, const hall_cfg_t *c, uint8_t state, float dt);

/* ---- Codeur ABI (timer en mode codeur x4) */
typedef struct {
    uint32_t cpr;          /* fronts par tour mecanique */
    float ratio;           /* tours electriques par tour mecanique (paires de poles) */
    float offset;          /* rad electriques */
    bool inverted;
} enc_cfg_t;

typedef struct {
    float theta, omega;
    float theta_prev;
    bool init;
} enc_t;

void enc_reset(enc_t *e);
float enc_raw_angle(const enc_cfg_t *c, uint32_t count);   /* angle electrique sans offset */
void enc_update(enc_t *e, const enc_cfg_t *c, uint32_t count, float dt);

#endif
