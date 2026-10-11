/*
 * observer.c — Observateurs de position sensorless.
 *
 * Les observateurs 0 a 6 reprennent les equations des observateurs de VESC 7.00
 * (motor/foc_math.c, foc_observer_update, Benjamin Vedder, GPL v3), memes coefficients et
 * memes bornes, reecrits avec les structures RESC. References :
 *  - Lee, Hong, Nam, Ortega, Praly, Astolfi, IEEE TPEL 2010 (observateur non lineaire)
 *  - Bernard, Praly, IFAC 2017 (estimation du flux de l'aimant)
 *
 * MXLEMMING, note de licence de son auteur (reproduite comme elle le demande) :
 *   "This function deviates slightly from the BSD 3 clause licence.
 *    The work here is entirely original to the MESC FOC project, and not based
 *    on any appnotes, or borrowed from another project. This work is free to
 *    use, as granted in BSD 3 clause, with the exception that this note must
 *    be included in where this code is implemented/modified to use your
 *    variable names, structures containing variables or other minor
 *    rearrangements in place of the original names I have chosen, and credit
 *    to David Molony as the original author must be noted."
 *   Auteur de l'observateur MXLEMMING : David Molony (projet MESC FOC).
 *
 * L'observateur 7 (RESC_ACTIVE_FLUX) est propre a RESC.
 */
#include "observer.h"
#include "fmath.h"
#include "platform.h"

bool obs_lambda_comp(obs_kind_t k)
{
    return k == OBS_ORTEGA_LAMBDA_COMP || k == OBS_MXLEMMING_LAMBDA_COMP || k == OBS_MXV_LAMBDA_COMP ||
           k == OBS_MXV_LAMBDA_COMP_LIN;
}

void obs_reset(obs_t *o, const obs_cfg_t *c, float theta, float omega)
{
    float s, co;
    fm_sincos(theta, &s, &co);
    o->x1 = c->lambda * co;          /* courant nul : flux statorique = flux de l'aimant */
    o->x2 = c->lambda * s;
    o->ia_last = o->ib_last = 0.0f;
    o->lambda_est = c->lambda;
    o->theta = theta;
    o->flux = c->lambda;
    o->lambda = c->lambda;
    o->pll_theta = theta;
    o->pll_omega = omega;
    o->omega = omega;
}

/* Observateur RESC : flux actif eta = psi_s - Lq i, contrainte |eta| = lambda + (Ld - Lq) id */
RAMFUNC static void resc_active_flux(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib,
                                     float gamma, float dt)
{
    float lam_a = c->lambda + (c->ld - c->lq) * c->id;
    if (lam_a < 0.1f * c->lambda) lam_a = 0.1f * c->lambda;
    o->x1 += (va - c->r * ia) * dt;
    o->x2 += (vb - c->r * ib) * dt;
    float eta_a = o->x1 - c->lq * ia;
    float eta_b = o->x2 - c->lq * ib;
    float err = lam_a * lam_a - (eta_a * eta_a + eta_b * eta_b);
    float corr = fm_sat_sym(0.5f * gamma * err * dt, 0.5f);   /* bornee pour rester stable en discret */
    eta_a += corr * eta_a;
    eta_b += corr * eta_b;
    o->x1 = eta_a + c->lq * ia;
    o->x2 = eta_b + c->lq * ib;
    o->theta = fm_atan2(eta_b, eta_a);
    o->flux = fm_sqrt(eta_a * eta_a + eta_b * eta_b);
    o->lambda = c->lambda;
}

RAMFUNC void obs_update(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib, float mod, float dt)
{
    float slow = c->gain_slow + (1.0f - c->gain_slow) * fm_min(fm_abs(mod) * 2.0f, 1.0f);
    float gamma = c->gamma * slow;
    float lambda = c->lambda;

    if (c->kind == OBS_RESC_ACTIVE_FLUX) {
        resc_active_flux(o, c, va, vb, ia, ib, gamma, dt);
    } else {
        /* inductance vue par l'observateur : moyenne corrigee de la saillance (comme VESC) */
        float L = 0.5f * (c->ld + c->lq);
        float diff = c->lq - c->ld;
        float i2 = c->id * c->id + c->iq * c->iq;
        if (fm_abs(c->id) > 0.1f || fm_abs(c->iq) > 0.1f) L = L - 0.5f * diff + diff * c->iq * c->iq / i2;
        float lia = L * ia, lib = L * ib;
        const float ria = c->r * ia, rib = c->r * ib;
        const float gh = 0.5f * gamma;
        if (!(o->lambda_est > 0.0f)) o->lambda_est = lambda;

        switch (c->kind) {
        case OBS_ORTEGA_ORIGINAL:
        case OBS_ORTEGA_LAMBDA_COMP: {
            float lr = c->kind == OBS_ORTEGA_LAMBDA_COMP ? o->lambda_est : lambda;
            float err = lr * lr - ((o->x1 - lia) * (o->x1 - lia) + (o->x2 - lib) * (o->x2 - lib));
            if (c->kind == OBS_ORTEGA_LAMBDA_COMP) {
                o->lambda_est += 0.2f * gh * o->lambda_est * -err * dt;
                o->lambda_est = fm_sat(o->lambda_est, 0.3f * lambda, 2.5f * lambda);
            }
            if (err > 0.0f) err = 0.0f;          /* terme force negatif : meilleure convergence */
            o->x1 += (va - ria + gh * (o->x1 - lia) * err) * dt;
            o->x2 += (vb - rib + gh * (o->x2 - lib) * err) * dt;
            break;
        }
        case OBS_MXLEMMING:
        case OBS_MXLEMMING_LAMBDA_COMP:
            /* MXLEMMING (David Molony, MESC) : flux de l'aimant integre, composantes bornees */
            o->x1 += (va - ria) * dt - L * (ia - o->ia_last);
            o->x2 += (vb - rib) * dt - L * (ib - o->ib_last);
            if (c->kind == OBS_MXLEMMING_LAMBDA_COMP) {
                float err = o->lambda_est * o->lambda_est - (o->x1 * o->x1 + o->x2 * o->x2);
                o->lambda_est += 0.1f * gh * o->lambda_est * -err * dt;
                o->lambda_est = fm_sat(o->lambda_est, 0.3f * lambda, 2.5f * lambda);
                o->x1 = fm_sat_sym(o->x1, o->lambda_est);
                o->x2 = fm_sat_sym(o->x2, o->lambda_est);
            } else {
                o->x1 = fm_sat_sym(o->x1, lambda);
                o->x2 = fm_sat_sym(o->x2, lambda);
            }
            lia = lib = 0.0f;                    /* x est deja le flux de l'aimant */
            break;
        default: {                               /* MXV et variantes */
            o->x1 += (va - ria) * dt;
            o->x2 += (vb - rib) * dt;
            float mag = fm_sqrt((o->x1 - lia) * (o->x1 - lia) + (o->x2 - lib) * (o->x2 - lib));
            float lim = lambda;
            if (c->kind == OBS_MXV_LAMBDA_COMP_LIN) {
                float k = fm_min(0.1f * gh * dt * o->lambda_est * o->lambda_est, 1.0f);
                o->lambda_est += (mag - o->lambda_est) * k;
                o->lambda_est = fm_sat(o->lambda_est, 0.3f * lambda, 2.5f * lambda);
                lim = o->lambda_est;
            } else if (c->kind == OBS_MXV_LAMBDA_COMP) {
                float err = o->lambda_est * o->lambda_est - mag * mag;
                o->lambda_est += 0.2f * gh * o->lambda_est * -err * dt;
                o->lambda_est = fm_sat(o->lambda_est, 0.3f * lambda, 2.5f * lambda);
                lim = o->lambda_est;
            }
            if (mag > lim) {
                /* comme la reference : x (et non x - L i) est mis a l'echelle */
                o->x1 = (o->x1 / mag) * lim;
                o->x2 = (o->x2 / mag) * lim;
            }
            break;
        }
        }
        o->ia_last = ia;
        o->ib_last = ib;
        if (!(o->x1 == o->x1)) o->x1 = 0.0f;     /* NaN -> 0 */
        if (!(o->x2 == o->x2)) o->x2 = 0.0f;
        /* module trop faible : angle instable, on le fait remonter */
        float m = fm_sqrt(o->x1 * o->x1 + o->x2 * o->x2);
        if (m < 0.5f * lambda) {
            o->x1 *= 1.1f;
            o->x2 *= 1.1f;
        }
        float fa = o->x1 - lia, fb = o->x2 - lib;
        o->theta = fm_atan2(fb, fa);
        o->flux = fm_sqrt(fa * fa + fb * fb);
        o->lambda = obs_lambda_comp(c->kind) ? o->lambda_est : lambda;
    }

    o->theta = fm_wrap_pi(o->theta + c->offset * o->omega * dt);

    /* PLL de type 2 sur l'angle observe */
    float e = fm_wrap_pi(o->theta - o->pll_theta);
    o->omega = o->pll_omega + c->pll_kp * e;
    o->pll_theta = fm_wrap_pi(o->pll_theta + o->omega * dt);
    o->pll_omega += c->pll_ki * e * dt;
}
