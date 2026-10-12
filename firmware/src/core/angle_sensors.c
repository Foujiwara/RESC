#include "angle_sensors.h"
#include "fmath.h"
#include "platform.h"

#define HALL_EDGE_TIMEOUT  0.15f    /* s sans front : vitesse consideree nulle */

void hall_reset(hall_t *h)
{
    h->prev = -1;
    h->dt_now = 0.0f;
    h->dt_last = 1.0f;
    h->ang = 0.0f;
    h->omega = 0.0f;
    h->theta = 0.0f;
    h->init = false;
    h->ok = false;
    h->n_invalid = 0;
}

/* Reprise exacte de foc_correct_hall (VESC 7.00, foc_math.c) : angle au milieu des deux secteurs
 * a chaque front, interpolation a la vitesse du dernier secteur au-dessus de foc_hall_interp_erpm
 * (sinon centre du secteur), puis angle de sortie a vitesse de variation limitee
 * (1.5 x max(vitesse Hall, vitesse d'interpolation)). */
RAMFUNC void hall_update(hall_t *h, const hall_cfg_t *c, uint8_t s, float dt)
{
    s &= 7;
    h->dt_now += dt;
    if (!c->valid[s]) {
        /* etat invalide : angle inchange */
        h->prev = -1;
        h->ok = false;
        h->n_invalid++;
        return;
    }
    float now = c->angle[s];
    if (h->prev < 0) {
        h->prev = (int8_t)s;
        h->ang = now;
        if (!h->init) { h->init = true; h->theta = now; }
    } else if ((uint8_t)h->prev != s) {
        float diff = fm_wrap_pi(now - c->angle[(uint8_t)h->prev]);
        /* le sens n'a pas change : duree du secteur ; sinon la derniere duree avec le nouveau sens */
        if ((diff > 0.0f) == (h->dt_last > 0.0f)) h->dt_last = diff > 0.0f ? h->dt_now : -h->dt_now;
        else h->dt_last = -h->dt_last;
        h->dt_now = 0.0f;
        h->ang = fm_wrap_pi(c->angle[(uint8_t)h->prev] + 0.5f * diff);   /* front : milieu des secteurs */
    }
    h->prev = (int8_t)s;
    h->ok = true;

    float dl = fm_abs(h->dt_last) > 1e-6f ? h->dt_last : 1e-6f;
    float rps = (FM_PI / 3.0f) / dl;                       /* signe = sens */
    float w_slow = (FM_PI / 3.0f) / fm_max(fm_abs(h->dt_now), fm_abs(h->dt_last));
    if (w_slow < c->interp_omega) {
        h->ang = now;                                       /* basse vitesse : secteur le plus proche */
    } else {
        float d = fm_wrap_pi(h->ang - now);
        if (fm_abs(d) < FM_PI / 6.0f || (d > 0.0f) != (rps > 0.0f)) h->ang += rps * dt;
        else h->ang -= d * 0.01f;                           /* trop loin : rappel doux */
    }
    h->ang = fm_wrap_pi(h->ang);
    h->omega = h->dt_now > HALL_EDGE_TIMEOUT ? 0.0f : (rps > 0.0f ? w_slow : -w_slow);

    /* vitesse de variation limitee (pointes de courant) */
    float step = fm_max(fm_abs(rps), c->interp_omega) * dt * 1.5f;
    float ad = fm_wrap_pi(h->ang - h->theta);
    if (fm_abs(ad) < step) h->theta = h->ang;
    else h->theta = fm_wrap_pi(h->theta + (ad > 0.0f ? step : -step));
}

void enc_reset(enc_t *e)
{
    e->theta = e->omega = e->theta_prev = 0.0f;
    e->init = false;
}

RAMFUNC float enc_raw_angle(const enc_cfg_t *c, uint32_t count)
{
    uint32_t cpr = c->cpr ? c->cpr : 1;
    float m = (float)(count % cpr) / (float)cpr * FM_2PI;
    float th = m * c->ratio;
    if (c->inverted) th = -th;
    return fm_wrap_pi(th);
}

RAMFUNC void enc_update(enc_t *e, const enc_cfg_t *c, uint32_t count, float dt)
{
    float th = fm_wrap_pi(enc_raw_angle(c, count) - c->offset);
    if (!e->init) {
        e->theta_prev = th;
        e->init = true;
    }
    float w = fm_wrap_pi(th - e->theta_prev) / dt;
    e->theta_prev = th;
    FM_LPF(e->omega, w, 0.05f);
    e->theta = th;
}
