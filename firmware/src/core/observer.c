#include "observer.h"
#include "fmath.h"
#include "platform.h"

void obs_reset(obs_t *o, const obs_cfg_t *c, float theta, float omega)
{
    float s, co;
    fm_sincos(theta, &s, &co);
    o->eta_a = c->lambda * co;
    o->eta_b = c->lambda * s;
    o->psi_a = o->eta_a;
    o->psi_b = o->eta_b;
    o->theta = theta;
    o->flux = c->lambda;
    o->pll_theta = theta;
    o->pll_omega = omega;
    o->omega = omega;
}

RAMFUNC void obs_update(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib, float id_est, float dt)
{
    /* Module attendu du flux actif */
    float lam_a = c->lambda + (c->ld - c->lq) * id_est;
    if (lam_a < 0.1f * c->lambda) lam_a = 0.1f * c->lambda;

    float eta_a = o->psi_a - c->lq * ia;
    float eta_b = o->psi_b - c->lq * ib;
    float err = lam_a * lam_a - (eta_a * eta_a + eta_b * eta_b);

    /* Correction radiale, bornee pour rester stable en discret */
    float corr = 0.5f * c->gamma * err * dt;
    corr = fm_sat_sym(corr, 0.5f);

    o->psi_a += (va - c->r * ia) * dt + corr * eta_a;
    o->psi_b += (vb - c->r * ib) * dt + corr * eta_b;

    o->eta_a = o->psi_a - c->lq * ia;
    o->eta_b = o->psi_b - c->lq * ib;
    o->flux = fm_sqrt(o->eta_a * o->eta_a + o->eta_b * o->eta_b);
    o->theta = fm_atan2(o->eta_b, o->eta_a);

    /* PLL de type 2 sur l'angle observe */
    float e = fm_wrap_pi(o->theta - o->pll_theta);
    o->omega = o->pll_omega + c->pll_kp * e;
    o->pll_theta = fm_wrap_pi(o->pll_theta + o->omega * dt);
    o->pll_omega += c->pll_ki * e * dt;
}
