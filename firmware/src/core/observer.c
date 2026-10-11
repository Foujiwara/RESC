#include "observer.h"
#include "fmath.h"
#include "platform.h"

void obs_kind_from_type(uint8_t type, obs_kind_t *kind, bool *lambda_adapt)
{
    static const uint8_t k[7] = { OBS_FLUX_NL, OBS_FLUX_PROJ, OBS_FLUX_NL, OBS_FLUX_PROJ, OBS_BEMF, OBS_BEMF, OBS_HYBRID };
    static const bool    a[7] = { false, false, true, true, false, true, true };
    if (type > 6) type = 0;
    *kind = (obs_kind_t)k[type];
    *lambda_adapt = a[type];
}

void obs_reset(obs_t *o, const obs_cfg_t *c, float theta, float omega)
{
    float s, co;
    fm_sincos(theta, &s, &co);
    o->eta_a = c->lambda * co;
    o->eta_b = c->lambda * s;
    o->psi_a = o->eta_a;
    o->psi_b = o->eta_b;
    o->ea = -omega * c->lambda * s;
    o->eb = omega * c->lambda * co;
    o->ia_hat = o->ib_hat = 0.0f;
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

/* Flux actif par le modele en tension ; correction non lineaire ou projection */
RAMFUNC static void flux_step(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib,
                              float id_est, float mod, float dt)
{
    float lam_a = o->lambda + (c->ld - c->lq) * id_est;
    if (lam_a < 0.1f * o->lambda) lam_a = 0.1f * o->lambda;

    o->psi_a += (va - c->r * ia) * dt;
    o->psi_b += (vb - c->r * ib) * dt;
    float eta_a = o->psi_a - c->lq * ia;
    float eta_b = o->psi_b - c->lq * ib;

    if (c->kind == OBS_FLUX_PROJ) {
        /* rappel radial lineaire vers lambda_a, gain 2|w| (min 200 rad/s) : une projection
         * instantanee (gain infini) ne ramenerait l'erreur d'angle qu'au second ordre */
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
        float slow = c->gain_slow + (1.0f - c->gain_slow) * fm_min(fm_abs(mod) * 2.0f, 1.0f);
        float err = lam_a * lam_a - (eta_a * eta_a + eta_b * eta_b);
        /* correction radiale, bornee pour rester stable en discret */
        float corr = fm_sat_sym(0.5f * c->gamma * slow * err * dt, 0.5f);
        eta_a += corr * eta_a;
        eta_b += corr * eta_b;
    }
    o->psi_a = eta_a + c->lq * ia;
    o->psi_b = eta_b + c->lq * ib;
    o->eta_a = eta_a;
    o->eta_b = eta_b;
}

/* Observateur de Luenberger sur le courant (pas de derivee du courant mesure) :
 *   L di^/dt = v - R i^ - e^ + L l1 (i - i^)      de^/dt = -k (i - i^)
 * poles : s^2 + 2 z wn s + wn^2, wn = max(3 |w|, 600) borne a 0.25/dt, z = 0.7.
 * e^ suit e avec la fonction de transfert H = wn^2 / (s^2 + 2 z wn s + wn^2) :
 * a la pulsation |w|, retard atan2(2 z wn |w|, wn^2 - w^2) et gain |H|, compenses. */
#define BEMF_Z 0.7f
RAMFUNC static void bemf_step(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib, float dt)
{
    float l = 0.5f * (c->ld + c->lq);
    /* vitesse propre a l'observateur de FEM : rotation du vecteur e^ (independante de la PLL,
     * sinon la compensation de retard, fonction de la vitesse, boucle sur elle-meme) */
    float aw = fm_abs(o->e_omega);
    float wn = fm_sat(3.0f * aw, 600.0f, 0.25f / dt);
    float pa = o->ea, pb = o->eb;
    float l1 = fm_max(2.0f * BEMF_Z * wn - c->r / l, 0.0f);
    float k = l * wn * wn;
    float ea_ = ia - o->ia_hat, eb_ = ib - o->ib_hat;          /* erreur d'estimation du courant */
    o->ia_hat += (va - c->r * o->ia_hat - o->ea) * (dt / l) + l1 * ea_ * dt;
    o->ib_hat += (vb - c->r * o->ib_hat - o->eb) * (dt / l) + l1 * eb_ * dt;
    o->ea -= k * ea_ * dt;
    o->eb -= k * eb_ * dt;

    float re = wn * wn - aw * aw, im = 2.0f * BEMF_Z * wn * aw;
    float hmag = wn * wn / fm_max(fm_sqrt(re * re + im * im), 1e-6f);
    o->e_gain = hmag;
    float e2 = o->ea * o->ea + o->eb * o->eb;
    float emag = fm_sqrt(e2);
    /* FEM trop faible devant le bruit : angle conserve (la PLL extrapole) */
    if (emag < 0.05f * c->lambda * 300.0f) {
        o->e_omega *= 0.999f;
        return;
    }
    float wr = (pa * o->eb - pb * o->ea) / (e2 * dt);           /* rotation de e^ (rad/s) */
    o->e_omega += (fm_sat_sym(wr, 0.5f / dt) - o->e_omega) * fm_min(50.0f * dt, 1.0f);
    /* e = w lambda (-sin, cos) : angle = atan2(-ea, eb) au signe de w pres */
    float sg = o->e_omega >= 0.0f ? 1.0f : -1.0f;
    float th = fm_atan2(-sg * o->ea, sg * o->eb) + sg * fm_atan2(im, re);
    o->e_theta = fm_wrap_pi(th);
}

RAMFUNC void obs_update(obs_t *o, const obs_cfg_t *c, float va, float vb, float ia, float ib, float id_est, float mod, float dt)
{
    bool need_flux = c->kind != OBS_BEMF;
    bool need_bemf = c->kind == OBS_BEMF || c->kind == OBS_HYBRID || c->lambda_adapt;

    if (need_flux) flux_step(o, c, va, vb, ia, ib, id_est, mod, dt);
    if (need_bemf) bemf_step(o, c, va, vb, ia, ib, dt);

    float aw = c->kind == OBS_BEMF ? fm_abs(o->e_omega) : fm_abs(o->omega);
    float emag = need_bemf ? fm_sqrt(o->ea * o->ea + o->eb * o->eb) / fm_max(o->e_gain, 0.3f) : 0.0f;

    /* lambda adaptatif : |e| / |w| quand la FEM domine (|w| lambda > 4 x chute ohmique estimee) */
    if (c->lambda_adapt && need_bemf) {
        float i_mag = fm_sqrt(ia * ia + ib * ib);
        if (c->adapt_on && aw * c->lambda > 4.0f * c->r * i_mag + 0.5f) {
            float meas = emag / aw;
            o->lambda += (meas - o->lambda) * fm_min(dt, 1.0f);
            o->lambda = fm_sat(o->lambda, 0.7f * c->lambda, 1.3f * c->lambda);
        }
    } else {
        o->lambda = c->lambda;
    }

    float th;
    switch (c->kind) {
    case OBS_BEMF:
        th = o->e_theta;
        o->flux = aw > 1.0f ? emag / aw : o->lambda;
        break;
    case OBS_HYBRID: {
        float w = c->hyb_w2 > c->hyb_w1 ? fm_sat((aw - c->hyb_w1) / (c->hyb_w2 - c->hyb_w1), 0.0f, 1.0f) : 0.0f;
        float fa = o->eta_a, fb = o->eta_b;
        float fm_ = fm_sqrt(fa * fa + fb * fb);
        float s, co;
        fm_sincos(o->e_theta, &s, &co);
        float ua = (1.0f - w) * (fm_ > 1e-9f ? fa / fm_ : 1.0f) + w * co;
        float ub = (1.0f - w) * (fm_ > 1e-9f ? fb / fm_ : 0.0f) + w * s;
        th = fm_atan2(ub, ua);
        o->flux = (1.0f - w) * fm_ + w * (aw > 1.0f ? emag / aw : o->lambda);
        break;
    }
    default:
        th = fm_atan2(o->eta_b, o->eta_a);
        o->flux = fm_sqrt(o->eta_a * o->eta_a + o->eta_b * o->eta_b);
        break;
    }
    o->theta = fm_wrap_pi(th + c->offset * o->omega * dt);

    /* PLL de type 2 sur l'angle observe */
    float e = fm_wrap_pi(o->theta - o->pll_theta);
    o->omega = o->pll_omega + c->pll_kp * e;
    o->pll_theta = fm_wrap_pi(o->pll_theta + o->omega * dt);
    o->pll_omega += c->pll_ki * e * dt;
}
