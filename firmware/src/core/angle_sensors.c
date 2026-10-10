#include "angle_sensors.h"
#include "fmath.h"
#include "platform.h"

#define HALL_EDGE_TIMEOUT  0.15f    /* s sans front : vitesse consideree nulle */
#define HALL_SECTOR_HALF   (FM_PI / 6.0f + 0.17f)   /* 30 deg + 10 deg de marge */

void hall_reset(hall_t *h)
{
    h->state = 0xFF;
    h->edge_theta = 0.0f;
    h->since_edge = 1.0f;
    h->omega = 0.0f;
    h->last_dir = 0;
    h->theta = 0.0f;
    h->ok = false;
    h->n_invalid = 0;
}

/* Milieu circulaire de deux angles */
static float mid_angle(float a, float b)
{
    return fm_wrap_pi(a + 0.5f * fm_wrap_pi(b - a));
}

RAMFUNC void hall_update(hall_t *h, const hall_cfg_t *c, uint8_t s, float dt)
{
    s &= 7;
    h->since_edge += dt;
    if (!c->valid[s]) {
        h->ok = false;
        h->n_invalid++;
        return;
    }
    if (h->state <= 7 && s != h->state && c->valid[h->state]) {
        float e = mid_angle(c->angle[h->state], c->angle[s]);
        float step = fm_wrap_pi(c->angle[s] - c->angle[h->state]);
        int8_t dir = step >= 0.0f ? 1 : -1;
        if (dir == h->last_dir && h->since_edge < HALL_EDGE_TIMEOUT) {
            float w = fm_wrap_pi(e - h->edge_theta) / fm_max(h->since_edge, dt);
            h->omega = 0.5f * h->omega + 0.5f * w;
        } else {
            h->omega = 0.0f;
        }
        h->last_dir = dir;
        h->edge_theta = e;
        h->since_edge = 0.0f;
    }
    h->state = s;
    h->ok = true;
    if (h->since_edge > HALL_EDGE_TIMEOUT) h->omega = 0.0f;

    float center = c->angle[s];
    if (fm_abs(h->omega) >= c->interp_omega && h->since_edge < HALL_EDGE_TIMEOUT) {
        /* interpolation depuis le dernier front, bornee au secteur courant */
        float th = h->edge_theta + h->omega * h->since_edge;
        float d = fm_sat_sym(fm_wrap_pi(th - center), HALL_SECTOR_HALF);
        h->theta = fm_wrap_pi(center + d);
    } else {
        h->theta = center;
    }
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
