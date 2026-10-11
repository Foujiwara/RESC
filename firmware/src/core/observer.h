/*
 * observer.h — Observateurs de position sensorless + PLL.
 *
 * foc_observer_type (valeurs du protocole) :
 *  0 ORTEGA_ORIGINAL        \
 *  1 MXLEMMING               |  observateurs de VESC 7.00 (motor/foc_math.c, foc_observer_update),
 *  2 ORTEGA_LAMBDA_COMP      |  reimplementes avec les memes equations, coefficients et bornes
 *  3 MXLEMMING_LAMBDA_COMP   |  (voir observer.c)
 *  4 MXV                     |
 *  5 MXV_LAMBDA_COMP         |
 *  6 MXV_LAMBDA_COMP_LIN    /
 *  7 RESC_ACTIVE_FLUX       observateur RESC : flux actif (Boldea) avec la correction non lineaire
 *                           de Lee, Ortega et al. 2010 ; module attendu lambda + (Ld - Lq) id,
 *                           valable pour les moteurs a poles lisses comme saillants
 *
 * Modele commun (repere alpha/beta) : d(psi_s)/dt = v - R i, flux de l'aimant = psi_s - L i.
 * Gain a basse vitesse : gamma_eff = gamma * (gain_slow + (1 - gain_slow) * min(1, 2 |m|)),
 * m = max(modulation appliquee, FEM / (Vbus/sqrt3)). Decalage : angle avance de offset x w x dt.
 */
#ifndef OBSERVER_H
#define OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OBS_ORTEGA_ORIGINAL = 0,
    OBS_MXLEMMING,
    OBS_ORTEGA_LAMBDA_COMP,
    OBS_MXLEMMING_LAMBDA_COMP,
    OBS_MXV,
    OBS_MXV_LAMBDA_COMP,
    OBS_MXV_LAMBDA_COMP_LIN,
    OBS_RESC_ACTIVE_FLUX,
    OBS_COUNT
} obs_kind_t;

typedef struct {
    float x1, x2;           /* etat (flux statorique ou flux de l'aimant selon l'observateur) */
    float ia_last, ib_last; /* courant precedent (MXLEMMING) */
    float lambda_est;       /* lambda estime (variantes LAMBDA_COMP) */
    /* sorties */
    float theta;            /* angle rotor */
    float flux;             /* module du flux de l'aimant estime (Wb) */
    float lambda;           /* lambda utilise (estime ou configure) */
    float pll_theta;
    float pll_omega;        /* integrateur de la PLL */
    float omega;            /* vitesse estimee = derivee de l'angle PLL */
} obs_t;

typedef struct {
    obs_kind_t kind;
    float r, ld, lq, lambda;
    float id, iq;           /* courants d/q (inductance de saillance) */
    float gamma;
    float gain_slow;        /* fraction du gain a modulation nulle (0..1) */
    float offset;           /* periodes */
    float pll_kp, pll_ki;
} obs_cfg_t;

void obs_reset(obs_t *o, const obs_cfg_t *c, float theta, float omega);
/* mod : niveau de tension (0..1) pour le gain a basse vitesse */
void obs_update(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib, float mod, float dt);
/* true : lambda estime par l'observateur */
bool obs_lambda_comp(obs_kind_t k);

#endif /* OBSERVER_H */
