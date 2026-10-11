/*
 * stats.c — Statistiques de trajet et odometre.
 */
#include <math.h>
#include <string.h>
#include "stats.h"
#include "ctrl.h"
#include "resc_conf.h"

static struct {
    double speed_sum, power_sum, current_sum, tmos_sum, tmot_sum;
    uint32_t n;
    float speed_max, power_max, current_max, tmos_max, tmot_max;
    uint32_t t0;
} S;

static uint32_t s_odo_base;          /* metres, odometre a la mise sous tension (ou fixe par l'outil) */
static float s_odo_ref;              /* distance absolue au moment ou la base a ete fixee */

static float pole_pairs(void) { return g_mcconf.si_motor_poles > 1.0f ? g_mcconf.si_motor_poles * 0.5f : 1.0f; }
static float gear(void) { return g_mcconf.si_gear_ratio > 0.0f ? g_mcconf.si_gear_ratio : 1.0f; }

float stats_speed_ms(void)
{
    float rpm = ctrl_stats()->erpm / pole_pairs() / gear();
    return rpm * 3.14159265f * g_mcconf.si_wheel_diameter / 60.0f;
}

/* tachymetre : 6 pas par tour electrique */
float stats_distance_m(void)
{
    return (float)ctrl_stats()->tacho / (6.0f * pole_pairs()) / gear() * 3.14159265f * g_mcconf.si_wheel_diameter;
}

float stats_distance_abs_m(void)
{
    return (float)ctrl_stats()->tacho_abs / (6.0f * pole_pairs()) / gear() * 3.14159265f * g_mcconf.si_wheel_diameter;
}

uint32_t stats_odometer_m(void)
{
    float d = stats_distance_abs_m() - s_odo_ref;
    return s_odo_base + (uint32_t)(d > 0.0f ? d : 0.0f);
}

void stats_set_odometer_m(uint32_t m)
{
    s_odo_base = m;
    s_odo_ref = stats_distance_abs_m();
}

void stats_odometer_restore(uint32_t base_m)
{
    s_odo_base = base_m == 0xFFFFFFFFu ? 0u : base_m;
    s_odo_ref = stats_distance_abs_m();
}

uint32_t stats_odometer_to_store(void) { return stats_odometer_m(); }

void stats_reset(uint32_t now_ms)
{
    memset(&S, 0, sizeof(S));
    S.t0 = now_ms;
}

void stats_update(uint32_t now_ms)
{
    (void)now_ms;
    const ctrl_stats_t *c = ctrl_stats();
    double power = (double)c->vbus * fabs((double)c->i_in);
    double speed = fabs((double)stats_speed_ms());
    double cur = fabs((double)c->i_motor);
    S.power_sum += power;
    S.speed_sum += speed;
    S.current_sum += cur;
    S.tmos_sum += (double)c->tfet;
    S.tmot_sum += (double)c->tmot;
    S.n++;
    if (power > (double)S.power_max) S.power_max = (float)power;
    if (speed > (double)S.speed_max) S.speed_max = (float)speed;
    if (cur > (double)S.current_max) S.current_max = (float)cur;
    if (c->tfet > S.tmos_max) S.tmos_max = c->tfet;
    if (c->tmot > S.tmot_max) S.tmot_max = c->tmot;
}

float stats_get(int field, uint32_t now_ms)
{
    double n = (double)(S.n > 0u ? S.n : 1u);
    switch (field) {
    case 0:  return (float)(S.speed_sum / n);
    case 1:  return S.speed_max;
    case 2:  return (float)(S.power_sum / n);
    case 3:  return S.power_max;
    case 4:  return (float)(S.current_sum / n);
    case 5:  return S.current_max;
    case 6:  return (float)(S.tmos_sum / n);
    case 7:  return S.tmos_max;
    case 8:  return (float)(S.tmot_sum / n);
    case 9:  return S.tmot_max;
    case 10: return (float)(now_ms - S.t0) * 1e-3f;
    default: return 0.0f;
    }
}
