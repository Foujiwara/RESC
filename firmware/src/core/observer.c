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
 * Les observateurs 7 a 13 (RESC_...) sont propres a RESC.
 */
#include "observer.h"
#include "fmath.h"
#include "platform.h"

static bool resc_lambda(obs_kind_t k)
{
    return k == OBS_RESC_ACTIVE_FLUX_LAMBDA || k == OBS_RESC_RADIAL_LAMBDA || k == OBS_RESC_BEMF_LAMBDA ||
           k == OBS_RESC_HYBRID_LAMBDA;
}

bool obs_lambda_comp(obs_kind_t k)
{
    return k == OBS_ORTEGA_LAMBDA_COMP || k == OBS_MXLEMMING_LAMBDA_COMP || k == OBS_MXV_LAMBDA_COMP ||
           k == OBS_MXV_LAMBDA_COMP_LIN || resc_lambda(k);
}

RAMFUNC void obs_reset(obs_t *o, const obs_cfg_t *c, float theta, float omega)
{
    float s, co;
    fm_sincos(theta, &s, &co);
    o->x1 = c->lambda * co;          /* courant nul : flux statorique = flux de l'aimant */
    o->x2 = c->lambda * s;
    o->ia_last = o->ib_last = 0.0f;
    o->lambda_est = c->lambda;
    o->ia_hat = o->ib_hat = 0.0f;
    o->ea = -omega * c->lambda * s;
    o->eb = omega * c->lambda * co;
    o->e_gain = 1.0f;
    o->e_omega = omega;
    o->e_theta = theta;
    o->theta = theta;
    o->flux = c->lambda;
    o->lambda = c->lambda;
    o->pll_theta = theta;
    o->pll_omega = omega;
    o->omega = omega;
}

/* ------------------------------------------------------------ observateurs RESC */

/* Flux actif eta = psi_s - Lq i (x1, x2 = psi_s), contrainte |eta| = lam + (Ld - Lq) id :
 * correction non lineaire (Lee, Ortega et al.) ou rappel radial lineaire de gain 2|w| */
RAMFUNC static void resc_flux_step(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib,
                                   float lam, float gamma, bool radial, float dt, float *ea_out, float *eb_out)
{
    float lam_a = lam + (c->ld - c->lq) * c->id;
    if (lam_a < 0.1f * lam) lam_a = 0.1f * lam;
    o->x1 += (va - c->r * ia) * dt;
    o->x2 += (vb - c->r * ib) * dt;
    float eta_a = o->x1 - c->lq * ia;
    float eta_b = o->x2 - c->lq * ib;
    if (radial) {
        /* une projection instantanee (gain infini) ne ramenerait l'erreur d'angle qu'au second ordre */
        float m = fm_sqrt(eta_a * eta_a + eta_b * eta_b);
        if (m > 1e-9f) {
            float k = fm_min(fm_max(2.0f * fm_abs(o->omega), 200.0f) * dt, 1.0f);
            float f = 1.0f + k * (lam_a / m - 1.0f);
            eta_a *= f;
            eta_b *= f;
        } else {
            eta_a = lam_a;
            eta_b = 0.0f;
        }
    } else {
        float err = lam_a * lam_a - (eta_a * eta_a + eta_b * eta_b);
        float corr = fm_sat_sym(0.5f * gamma * err * dt, 0.5f);   /* bornee pour rester stable en discret */
        eta_a += corr * eta_a;
        eta_b += corr * eta_b;
    }
    o->x1 = eta_a + c->lq * ia;
    o->x2 = eta_b + c->lq * ib;
    *ea_out = eta_a;
    *eb_out = eta_b;
}

/* Observateur de FEM : Luenberger sur le courant (pas de derivee du courant mesure)
 *   L di^/dt = v - R i^ - e^ + L l1 (i - i^)      de^/dt = -k (i - i^)
 * poles s^2 + 2 z wn s + wn^2, wn = max(3 |w|, 600) borne a 0.25/dt, z = 0.7 ;
 * a la pulsation |w|, retard atan2(2 z wn |w|, wn^2 - w^2) et gain |H| compenses.
 * La vitesse utilisee est la rotation propre de e^ (pas la PLL : la compensation boucherait). */
#define BEMF_Z 0.7f
RAMFUNC static void resc_bemf_step(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib, float dt)
{
    float l = 0.5f * (c->ld + c->lq);
    float aw = fm_abs(o->e_omega);
    float wn = fm_sat(3.0f * aw, 600.0f, 0.25f / dt);
    float pa = o->ea, pb = o->eb;
    float l1 = fm_max(2.0f * BEMF_Z * wn - c->r / l, 0.0f);
    float k = l * wn * wn;
    float ea_ = ia - o->ia_hat, eb_ = ib - o->ib_hat;
    o->ia_hat += (va - c->r * o->ia_hat - o->ea) * (dt / l) + l1 * ea_ * dt;
    o->ib_hat += (vb - c->r * o->ib_hat - o->eb) * (dt / l) + l1 * eb_ * dt;
    o->ea -= k * ea_ * dt;
    o->eb -= k * eb_ * dt;
    float re = wn * wn - aw * aw, im = 2.0f * BEMF_Z * wn * aw;
    o->e_gain = wn * wn / fm_max(fm_sqrt(re * re + im * im), 1e-6f);
    float e2 = o->ea * o->ea + o->eb * o->eb;
    if (fm_sqrt(e2) < 0.05f * c->lambda * 300.0f) {      /* FEM trop faible : angle conserve */
        o->e_omega *= 0.999f;
        return;
    }
    float wr = (pa * o->eb - pb * o->ea) / (e2 * dt);
    o->e_omega += (fm_sat_sym(wr, 0.5f / dt) - o->e_omega) * fm_min(50.0f * dt, 1.0f);
    float sg = o->e_omega >= 0.0f ? 1.0f : -1.0f;       /* e = w lambda (-sin, cos) */
    o->e_theta = fm_wrap_pi(fm_atan2(-sg * o->ea, sg * o->eb) + sg * fm_atan2(im, re));
}

RAMFUNC static void resc_update(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib,
                                float gamma, float dt)
{
    obs_kind_t k = c->kind;
    bool bemf_only = k == OBS_RESC_BEMF || k == OBS_RESC_BEMF_LAMBDA;
    bool need_bemf = bemf_only || k == OBS_RESC_HYBRID_LAMBDA || resc_lambda(k);
    bool adapt = resc_lambda(k);
    if (!adapt || !(o->lambda > 0.0f)) o->lambda = c->lambda;
    float fa = 0.0f, fb = 0.0f;
    if (!bemf_only)
        resc_flux_step(o, c, va, vb, ia, ib, o->lambda, gamma,
                       k == OBS_RESC_RADIAL || k == OBS_RESC_RADIAL_LAMBDA, dt, &fa, &fb);
    if (need_bemf) resc_bemf_step(o, c, va, vb, ia, ib, dt);

    float aw = bemf_only ? fm_abs(o->e_omega) : fm_abs(o->omega);
    float emag = need_bemf ? fm_sqrt(o->ea * o->ea + o->eb * o->eb) / fm_max(o->e_gain, 0.3f) : 0.0f;
    if (adapt && c->adapt_on && aw * c->lambda > 4.0f * c->r * fm_sqrt(ia * ia + ib * ib) + 0.5f) {
        o->lambda += (emag / aw - o->lambda) * fm_min(dt, 1.0f);
        o->lambda = fm_sat(o->lambda, 0.7f * c->lambda, 1.3f * c->lambda);
    }
    if (bemf_only) {
        o->theta = o->e_theta;
        o->flux = aw > 1.0f ? emag / aw : o->lambda;
    } else if (k == OBS_RESC_HYBRID_LAMBDA) {
        float w = c->hyb_w2 > c->hyb_w1 ? fm_sat((aw - c->hyb_w1) / (c->hyb_w2 - c->hyb_w1), 0.0f, 1.0f) : 0.0f;
        float fm_ = fm_sqrt(fa * fa + fb * fb);
        float sn, cs;
        fm_sincos(o->e_theta, &sn, &cs);
        float ua = (1.0f - w) * (fm_ > 1e-9f ? fa / fm_ : 1.0f) + w * cs;
        float ub = (1.0f - w) * (fm_ > 1e-9f ? fb / fm_ : 0.0f) + w * sn;
        o->theta = fm_atan2(ub, ua);
        o->flux = (1.0f - w) * fm_ + w * (aw > 1.0f ? emag / aw : o->lambda);
    } else {
        o->theta = fm_atan2(fb, fa);
        o->flux = fm_sqrt(fa * fa + fb * fb);
    }
}

RAMFUNC void obs_update(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib, float mod, float dt)
{
    float slow = c->gain_slow + (1.0f - c->gain_slow) * fm_min(fm_abs(mod) * 2.0f, 1.0f);
    float gamma = c->gamma * slow;
    float lambda = c->lambda;

    if (c->kind >= OBS_RESC_ACTIVE_FLUX) {
        resc_update(o, c, va, vb, ia, ib, gamma, dt);
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
