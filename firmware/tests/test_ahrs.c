/*
 * test_ahrs.c — Estimation d'attitude de l'IMU :
 *  - inclinaison statique (roulis 25 deg, puis tangage 15 deg) retrouvee par les trois filtres
 *  - rotation en lacet a 90 deg/s pendant 1 s integree (+/-2 deg)
 *  - acceleration hors 1 g : confiance reduite, convergence plus lente
 *  - filtre passe-bas : gain 1 en continu, attenuation au-dessus de la coupure
 */
#include <math.h>
#include <stdio.h>
#include "ahrs.h"

static int fails;
static void check(const char *w, int ok) { printf("  [%s] %s\n", ok ? " OK " : "ECHEC", w); if (!ok) fails++; }

#define D2R (3.14159265f / 180.0f)

static void run(ahrs_t *a, ahrs_mode_t m, const float g[3], const float acc[3], float t, float fs)
{
    int n = (int)(t * fs);
    for (int k = 0; k < n; k++) ahrs_update(a, m, g, acc, 1.0f / fs);
}

int main(void)
{
    static const char *mn[3] = { "Madgwick", "Mahony", "Madgwick Fusion" };
    const float g0[3] = { 0, 0, 0 };
    for (int m = 0; m < 3; m++) {
        printf("== %s\n", mn[m]);
        ahrs_t a;
        ahrs_init(&a);
        ahrs_params(&a, 1.0f, 2.0f, 0.0f, 0.5f);
        float acc[3] = { 0.0f, sinf(25 * D2R), cosf(25 * D2R) };
        run(&a, (ahrs_mode_t)m, g0, acc, 20.0f, 200.0f);
        float rpy[3];
        ahrs_rpy(&a, rpy);
        char t[96];
        snprintf(t, sizeof t, "roulis 25 deg retrouve (%.1f / %.1f / %.1f deg)", rpy[0] / D2R, rpy[1] / D2R, rpy[2] / D2R);
        check(t, fabsf(fabsf(rpy[0] / D2R) - 25.0f) < 1.0f && fabsf(rpy[1] / D2R) < 1.0f);
        ahrs_init(&a);
        float acc2[3] = { sinf(15 * D2R), 0.0f, cosf(15 * D2R) };
        run(&a, (ahrs_mode_t)m, g0, acc2, 20.0f, 200.0f);
        ahrs_rpy(&a, rpy);
        snprintf(t, sizeof t, "tangage 15 deg retrouve (%.1f / %.1f deg)", rpy[0] / D2R, rpy[1] / D2R);
        check(t, fabsf(fabsf(rpy[1] / D2R) - 15.0f) < 1.0f && fabsf(rpy[0] / D2R) < 1.0f);

        ahrs_init(&a);
        float flat[3] = { 0, 0, 1 }, gz[3] = { 0, 0, 90 * D2R };
        run(&a, (ahrs_mode_t)m, gz, flat, 1.0f, 1000.0f);
        ahrs_rpy(&a, rpy);
        snprintf(t, sizeof t, "lacet 90 deg/s pendant 1 s : %.1f deg", rpy[2] / D2R);
        check(t, fabsf(fabsf(rpy[2] / D2R) - 90.0f) < 2.0f);
    }

    printf("== Confiance dans l'accelerometre\n");
    {
        ahrs_t a, b;
        ahrs_init(&a); ahrs_init(&b);
        ahrs_params(&a, 1.0f, 2.0f, 0.0f, 0.5f);
        ahrs_params(&b, 1.0f, 2.0f, 0.0f, 0.5f);
        float acc1[3] = { 0.0f, sinf(25 * D2R), cosf(25 * D2R) };
        float acc2[3] = { 0.0f, 1.6f * sinf(25 * D2R), 1.6f * cosf(25 * D2R) };
        run(&a, AHRS_MADGWICK, g0, acc1, 0.5f, 200.0f);
        run(&b, AHRS_MADGWICK, g0, acc2, 0.5f, 200.0f);
        float ra[3], rb[3];
        ahrs_rpy(&a, ra); ahrs_rpy(&b, rb);
        check("1.6 g : convergence plus lente qu'a 1 g", fabsf(rb[0]) < fabsf(ra[0]));
    }

    printf("== Filtre passe-bas (coupure 10 Hz a 200 Hz)\n");
    {
        biquad_t f;
        biquad_lowpass(&f, 10.0f / 200.0f);
        float y = 0;
        for (int k = 0; k < 400; k++) y = biquad_run(&f, 1.0f);
        check("gain continu 1", fabsf(y - 1.0f) < 1e-3f);
        biquad_lowpass(&f, 10.0f / 200.0f);
        float mx = 0;
        for (int k = 0; k < 800; k++) {
            y = biquad_run(&f, sinf(2 * 3.14159265f * 60.0f * (float)k / 200.0f));
            if (k > 400 && fabsf(y) > mx) mx = fabsf(y);
        }
        char t[64];
        snprintf(t, sizeof t, "60 Hz attenue (amplitude %.3f)", mx);
        check(t, mx < 0.05f);
    }

    printf("\n%s (%d echec(s))\n", fails ? "ECHEC" : "TOUS LES TESTS PASSENT", fails);
    return fails ? 1 : 0;
}
