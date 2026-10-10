/*
 * fmath.h — Mathematiques rapides en simple precision pour la boucle FOC.
 * Toutes les fonctions sont inline, sans appel a la libm (pas de double,
 * pas d'errno), et executables depuis la RAM.
 */
#ifndef FMATH_H
#define FMATH_H

#include <stdint.h>
#include <stdbool.h>

#define FM_PI        3.14159265358979f
#define FM_2PI       6.28318530717959f
#define FM_PI_2      1.57079632679490f
#define FM_SQRT3     1.73205080756888f
#define FM_1_SQRT3   0.57735026918963f
#define FM_2_SQRT3   1.15470053837925f
#define FM_SQRT3_2   0.86602540378444f

static inline float fm_sat(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float fm_sat_sym(float x, float lim) { return fm_sat(x, -lim, lim); }
static inline float fm_abs(float x) { return x < 0.0f ? -x : x; }
static inline float fm_sign(float x) { return x < 0.0f ? -1.0f : 1.0f; }
static inline float fm_max(float a, float b) { return a > b ? a : b; }
static inline float fm_min(float a, float b) { return a < b ? a : b; }
static inline float fm_sqrt(float x) { return __builtin_sqrtf(x > 0.0f ? x : 0.0f); }
static inline float fm_sq(float x) { return x * x; }

/* Ramene un angle dans [-pi, pi). Valide pour |x| < 1e6. */
static inline float fm_wrap_pi(float x)
{
    if (x >= FM_PI || x < -FM_PI) {
        float k = (float)(int32_t)(x * (1.0f / FM_2PI) + (x >= 0.0f ? 0.5f : -0.5f));
        x -= k * FM_2PI;
        if (x >= FM_PI) x -= FM_2PI;
        if (x < -FM_PI) x += FM_2PI;
    }
    return x;
}

/* sin(x) pour x dans [-pi/2, pi/2] : Taylor ordre 9, erreur < 4e-6 */
static inline float fm_sin_core(float x)
{
    float x2 = x * x;
    return x * (1.0f + x2 * (-1.6666667e-1f + x2 * (8.3333333e-3f + x2 * (-1.9841270e-4f + x2 * 2.7557319e-6f))));
}

/* sin et cos simultanes, x quelconque (|x| < 1e6) */
static inline void fm_sincos(float x, float *s, float *c)
{
    x = fm_wrap_pi(x);
    /* sin : repli sur [-pi/2, pi/2] */
    float xs = x;
    if (xs > FM_PI_2) xs = FM_PI - xs;
    else if (xs < -FM_PI_2) xs = -FM_PI - xs;
    *s = fm_sin_core(xs);
    /* cos(x) = sin(x + pi/2) */
    float xc = x + FM_PI_2;
    if (xc >= FM_PI) xc -= FM_2PI;
    if (xc > FM_PI_2) xc = FM_PI - xc;
    else if (xc < -FM_PI_2) xc = -FM_PI - xc;
    *c = fm_sin_core(xc);
}

/* atan2 rapide : polynome minimax sur [0,1], erreur < 1.2e-5 rad */
static inline float fm_atan2(float y, float x)
{
    float ax = fm_abs(x), ay = fm_abs(y);
    float mx = fm_max(ax, ay), mn = fm_min(ax, ay);
    if (mx < 1e-20f) return 0.0f;
    float z = mn / mx;
    float z2 = z * z;
    float a = z * (0.99997726f + z2 * (-0.33262347f + z2 * (0.19354346f + z2 * (-0.11643287f + z2 * (0.05265332f + z2 * -0.01172120f)))));
    if (ay > ax) a = FM_PI_2 - a;
    if (x < 0.0f) a = FM_PI - a;
    if (y < 0.0f) a = -a;
    return a;
}

/* Filtre passe-bas du premier ordre : y += k (x - y), k dans ]0,1] */
#define FM_LPF(y, x, k) do { (y) += (k) * ((x) - (y)); } while (0)

#endif /* FMATH_H */
