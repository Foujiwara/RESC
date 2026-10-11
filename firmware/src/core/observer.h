/*
 * observer.h — Observateurs de position sensorless RESC + PLL.
 *
 * Plusieurs observateurs interchangeables (foc_observer_type), tous ecrits pour RESC.
 * Ils partagent le modele en tension (repere alpha/beta) :
 *   d(psi_s)/dt = v - R i                         (flux statorique)
 *   eta         = psi_s - Lq i                    (flux actif, aligne sur l'axe d)
 *   |eta|       = lambda + (Ld - Lq) id           (module attendu)
 *
 *  OBS_FLUX_NL   flux actif, correction non lineaire a gradient (Lee, Hong, Nam, Ortega
 *                et al. 2010, ecrite sur le flux actif de Boldea 2008) :
 *                  d(psi_s)/dt = v - R i + (gamma/2) eta (lambda_a^2 - |eta|^2)
 *  OBS_FLUX_PROJ flux actif integre puis projete sur le cercle de rayon lambda_a a chaque
 *                pas (contrainte imposee, sans gain) : tres robuste, aucune derive
 *  OBS_BEMF      observateur de FEM (Luenberger sur le courant, FEM integree, sans
 *                derivee du courant mesure), bande adaptee a la vitesse, retard et gain
 *                compenses ; ne depend pas de lambda pour l'angle
 *  OBS_HYBRID    flux actif a basse vitesse, FEM a haute vitesse, melange lineaire des
 *                vecteurs unitaires entre hyb_w1 et hyb_w2
 *
 * Option lambda adaptatif : lambda est reestime en marche a partir de |e| / |w|
 * (constante de temps 1 s, borne a +-30 % de la valeur configuree, au-dessus d'une
 * vitesse ou la FEM domine la chute ohmique). Il remplace lambda dans la contrainte de
 * module, l'anticipation de la boucle de courant et la surveillance de synchronisme.
 *
 * Gain a basse vitesse : gamma_eff = gamma * (gain_slow + (1 - gain_slow) * min(1, |m| / 0.5)),
 * m = max(modulation appliquee, FEM / (Vbus/sqrt3)) (0..1). Decalage : l'angle est avance de offset x w x dt (retard du filtre
 * de mesure, en periodes).
 */
#ifndef OBSERVER_H
#define OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OBS_FLUX_NL = 0,
    OBS_FLUX_PROJ,
    OBS_BEMF,
    OBS_HYBRID
} obs_kind_t;

typedef struct {
    /* modele en tension */
    float psi_a, psi_b;     /* flux statorique estime */
    float eta_a, eta_b;     /* flux actif estime */
    /* estimateur de FEM */
    float ia_hat, ib_hat;   /* courant estime (observateur de FEM) */
    float ea, eb;           /* FEM estimee */
    float e_gain;           /* gain |H| de l'observateur de FEM a la vitesse actuelle */
    float e_omega;          /* vitesse de rotation de e^ (rad/s, filtree) */
    float e_theta;
    /* sorties */
    float theta;            /* angle rotor */
    float flux;             /* module du flux estime (Wb) */
    float lambda;           /* lambda utilise (adaptatif ou configure) */
    float pll_theta;
    float pll_omega;        /* integrateur de la PLL */
    float omega;            /* vitesse estimee = derivee de l'angle PLL */
} obs_t;

typedef struct {
    obs_kind_t kind;
    bool  lambda_adapt;
    bool  adapt_on;         /* reestimation autorisee maintenant (marche sur l'observateur) */
    float r, ld, lq, lambda;
    float gamma;
    float gain_slow;        /* fraction du gain a modulation nulle (0..1) */
    float offset;           /* periodes */
    float hyb_w1, hyb_w2;   /* rad/s : melange flux -> FEM */
    float pll_kp, pll_ki;
} obs_cfg_t;

void obs_reset(obs_t *o, const obs_cfg_t *c, float theta, float omega);
/* mod : modulation appliquee (0..1), pour le gain a basse vitesse */
void obs_update(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib, float id_est, float mod, float dt);

/* Correspondance foc_observer_type (valeurs du protocole) -> observateur RESC */
void obs_kind_from_type(uint8_t type, obs_kind_t *kind, bool *lambda_adapt);

#endif /* OBSERVER_H */
