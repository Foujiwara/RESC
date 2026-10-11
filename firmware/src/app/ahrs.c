/*
 * ahrs.c — Estimation d'attitude (quaternion). Conventions et angles comme l'IMU de VESC
 * (roulis et lacet de signe oppose a la convention aeronautique du quaternion).
 */
#include <math.h>
#include <stdbool.h>
#include "ahrs.h"

static float inv_sqrt(float x) { return 1.0f / sqrtf(x); }

void ahrs_init(ahrs_t *a)
{
    a->q0 = 1.0f; a->q1 = a->q2 = a->q3 = 0.0f;
    a->ifb[0] = a->ifb[1] = a->ifb[2] = 0.0f;
    a->acc_mag_f = 1.0f;
}

void ahrs_params(ahrs_t *a, float decay, float kp, float ki, float beta)
{
    a->decay = decay;
    a->kp = kp;
    a->ki = ki;
    a->beta = beta;
}

/* confiance dans l'accelerometre : 1 a 1 g, decroit avec la racine de l'ecart */
static float acc_confidence(ahrs_t *a, float mag)
{
    a->acc_mag_f = a->acc_mag_f * 0.9f + mag * 0.1f;
    float c = 1.0f - a->decay * sqrtf(fabsf(a->acc_mag_f - 1.0f));
    return c < 0.0f ? 0.0f : (c > 1.0f ? 1.0f : c);
}

static void normalise(ahrs_t *a)
{
    float r = inv_sqrt(a->q0 * a->q0 + a->q1 * a->q1 + a->q2 * a->q2 + a->q3 * a->q3);
    a->q0 *= r; a->q1 *= r; a->q2 *= r; a->q3 *= r;
}

static void mahony(ahrs_t *a, const float g[3], const float acc[3], float dt, bool fusion)
{
    float gx = g[0], gy = g[1], gz = g[2];
    float ax = acc[0], ay = acc[1], az = acc[2];
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n > 0.01f) {
        float conf = acc_confidence(a, n);
        ax /= n; ay /= n; az /= n;
        /* demi-vecteur gravite estime (3e colonne de la matrice de rotation / 2) */
        float hvx = a->q1 * a->q3 - a->q0 * a->q2;
        float hvy = a->q0 * a->q1 + a->q2 * a->q3;
        float hvz = a->q0 * a->q0 - 0.5f + a->q3 * a->q3;
        float ex = ay * hvz - az * hvy, ey = az * hvx - ax * hvz, ez = ax * hvy - ay * hvx;
        if (fusion) {
            /* Fusion : retour proportionnel de gain 'beta' (demi-erreur x gain) */
            float k = 2.0f * a->beta * conf;
            gx += k * ex; gy += k * ey; gz += k * ez;
        } else {
            float tkp = 2.0f * a->kp * conf, tki = 2.0f * a->ki * conf;
            if (tki > 0.0f) {
                a->ifb[0] += tki * ex * dt; a->ifb[1] += tki * ey * dt; a->ifb[2] += tki * ez * dt;
                gx += a->ifb[0]; gy += a->ifb[1]; gz += a->ifb[2];
            } else {
                a->ifb[0] = a->ifb[1] = a->ifb[2] = 0.0f;
            }
            gx += tkp * ex; gy += tkp * ey; gz += tkp * ez;
        }
    }
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    float qa = a->q0, qb = a->q1, qc = a->q2;
    a->q0 += -qb * gx - qc * gy - a->q3 * gz;
    a->q1 += qa * gx + qc * gz - a->q3 * gy;
    a->q2 += qa * gy - qb * gz + a->q3 * gx;
    a->q3 += qa * gz + qb * gy - qc * gx;
    normalise(a);
}

static void madgwick(ahrs_t *a, const float g[3], const float acc[3], float dt)
{
    float q0 = a->q0, q1 = a->q1, q2 = a->q2, q3 = a->q3;
    float d1 = 0.5f * (-q1 * g[0] - q2 * g[1] - q3 * g[2]);
    float d2 = 0.5f * (q0 * g[0] + q2 * g[2] - q3 * g[1]);
    float d3 = 0.5f * (q0 * g[1] - q1 * g[2] + q3 * g[0]);
    float d4 = 0.5f * (q0 * g[2] + q1 * g[1] - q2 * g[0]);
    float ax = acc[0], ay = acc[1], az = acc[2];
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n > 0.01f) {
        ax /= n; ay /= n; az /= n;
        /* descente de gradient sur l'ecart entre gravite mesuree et estimee */
        float s0 = 4.0f * q0 * q2 * q2 + 2.0f * q2 * ax + 4.0f * q0 * q1 * q1 - 2.0f * q1 * ay;
        float s1 = 4.0f * q1 * q3 * q3 - 2.0f * q3 * ax + 4.0f * q0 * q0 * q1 - 2.0f * q0 * ay - 4.0f * q1 +
                   8.0f * q1 * q1 * q1 + 8.0f * q1 * q2 * q2 + 4.0f * q1 * az;
        float s2 = 4.0f * q0 * q0 * q2 + 2.0f * q0 * ax + 4.0f * q2 * q3 * q3 - 2.0f * q3 * ay - 4.0f * q2 +
                   8.0f * q2 * q1 * q1 + 8.0f * q2 * q2 * q2 + 4.0f * q2 * az;
        float s3 = 4.0f * q1 * q1 * q3 - 2.0f * q1 * ax + 4.0f * q2 * q2 * q3 - 2.0f * q2 * ay;
        float sn = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (sn > 1e-12f) {
            float r = inv_sqrt(sn);
            float k = a->beta * acc_confidence(a, n) * r;
            d1 -= k * s0; d2 -= k * s1; d3 -= k * s2; d4 -= k * s3;
        }
    }
    a->q0 = q0 + d1 * dt;
    a->q1 = q1 + d2 * dt;
    a->q2 = q2 + d3 * dt;
    a->q3 = q3 + d4 * dt;
    normalise(a);
}

void ahrs_update(ahrs_t *a, ahrs_mode_t mode, const float g[3], const float acc[3], float dt)
{
    if (!(dt > 0.0f) || dt > 0.5f) return;
    switch (mode) {
    case AHRS_MAHONY: mahony(a, g, acc, dt, false); break;
    case AHRS_FUSION: mahony(a, g, acc, dt, true); break;
    default:          madgwick(a, g, acc, dt); break;
    }
}

void ahrs_rpy(const ahrs_t *a, float rpy[3])
{
    float q0 = a->q0, q1 = a->q1, q2 = a->q2, q3 = a->q3;
    rpy[0] = -atan2f(q0 * q1 + q2 * q3, 0.5f - (q1 * q1 + q2 * q2));
    float s = -2.0f * (q1 * q3 - q0 * q2);
    rpy[1] = asinf(s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s));
    rpy[2] = -atan2f(q0 * q3 + q1 * q2, 0.5f - (q2 * q2 + q3 * q3));
}

void biquad_lowpass(biquad_t *f, float fc)
{
    if (fc > 0.49f) fc = 0.49f;
    float k = tanf(3.14159265f * fc), q = 0.707f;
    float n = 1.0f / (1.0f + k / q + k * k);
    f->a0 = k * k * n;
    f->a1 = 2.0f * f->a0;
    f->a2 = f->a0;
    f->b1 = 2.0f * (k * k - 1.0f) * n;
    f->b2 = (1.0f - k / q + k * k) * n;
    f->z1 = f->z2 = 0.0f;
}

float biquad_run(biquad_t *f, float x)
{
    float y = x * f->a0 + f->z1;
    f->z1 = x * f->a1 + f->z2 - f->b1 * y;
    f->z2 = x * f->a2 - f->b2 * y;
    return y;
}
