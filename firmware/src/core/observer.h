/*
 * observer.h — Observateur de flux actif non lineaire + PLL.
 *
 * Modele (repere alpha/beta) :
 *   d(psi_s)/dt = v - R i                         (flux statorique)
 *   eta         = psi_s - Lq i                    (flux actif, aligne sur l'axe d)
 *   |eta|       = lambda + (Ld - Lq) id           (contrainte de module)
 * Correction non lineaire a gradient (Lee, Hong, Nam, Ortega et al., 2010)
 * appliquee au flux actif (Boldea 2008), valable pour les moteurs a poles
 * lisses (Ld = Lq) comme saillants :
 *   d(psi_s)/dt = v - R i + (gamma/2) eta (lambda_a^2 - |eta|^2)
 * L'angle rotor est atan2(eta). La PLL de type 2 fournit la vitesse.
 */
#ifndef OBSERVER_H
#define OBSERVER_H

typedef struct {
    float psi_a, psi_b;     /* flux statorique estime */
    float eta_a, eta_b;     /* flux actif estime */
    float theta;            /* angle rotor (atan2 du flux actif) */
    float flux;             /* |eta| */
    float pll_theta;
    float pll_omega;        /* integrateur de la PLL */
    float omega;            /* vitesse estimee = derivee de l'angle PLL */
} obs_t;

typedef struct {
    float r, ld, lq, lambda;
    float gamma;
    float pll_kp, pll_ki;
} obs_cfg_t;

void obs_reset(obs_t *o, const obs_cfg_t *c, float theta, float omega);
void obs_update(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib, float id_est, float dt);

#endif /* OBSERVER_H */
