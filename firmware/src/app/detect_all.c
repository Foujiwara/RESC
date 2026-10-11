/*
 * detect_all.c — Detection automatique FOC (boucle principale, sans blocage).
 */
#include <math.h>
#include <string.h>
#include "detect_all.h"
#include "detect_app.h"
#include "commands.h"
#include "can_vesc.h"
#include "foc.h"
#include "ctrl.h"
#include "resc_conf.h"
#include "systime.h"
#include "vesc_buffer.h"
#include "vesc_defaults.h"
#include "hw_config.h"
#include "fmath.h"

typedef enum {
    WZ_IDLE = 0, WZ_WAIT_READY, WZ_RL, WZ_LAMBDA, WZ_HALL, WZ_ENC, WZ_WAIT_CAN
} wz_t;

static struct {
    wz_t st;
    const comm_iface_t *reply;      /* outil (NULL : demande d'un maitre CAN) */
    int can_master;                 /* >= 0 : reponse CAN a ce controleur */
    bool activate_status;
    float p_loss, i_max;
    uint32_t t0;
    int result;
    int can_devs, can_res_n, can_fail;
    uint8_t id_new;
    vesc_mcconf_t old;
} W = { .can_master = -1 };

bool detect_all_busy(void) { return W.st != WZ_IDLE; }

static void begin(float p_loss)
{
    W.old = g_mcconf;
    W.p_loss = p_loss > 1.0f ? p_loss : 50.0f;
    W.result = -1;
    W.t0 = systime_ms();
    /* mesures depuis l'etat de reference : sans capteur, courants max par defaut de la carte */
    vesc_mcconf_t def;
    resc_conf_defaults_mc(&def);
    g_mcconf.foc_sensor_mode = 0;
    g_mcconf.l_current_max = def.l_current_max;
    g_mcconf.l_current_min = def.l_current_min;
    g_mcconf.l_abs_current_max = def.l_abs_current_max;
    g_mcconf.l_current_max_scale = 1.0f;
    g_mcconf.l_current_min_scale = 1.0f;
    resc_conf_clamp_mc(&g_mcconf);
    resc_conf_apply();
    W.st = WZ_WAIT_READY;
    commands_printf("Detection automatique : R/L, flux, capteurs (puissance de pertes %.0f W)", (double)W.p_loss);
}

static void reply_result(int res)
{
    if (W.can_master >= 0) {
        can_vesc_detect_result((uint8_t)W.can_master, (int8_t)(res < -128 ? -128 : res));
        return;
    }
    if (!W.reply) return;
    static uint8_t b[PACKET_MAX_PL];
    int32_t o = 0;
    if (res >= 0) {
        /* configurations a jour vers l'outil, comme VESC */
        b[o++] = COMM_GET_MCCONF;
        o += vesc_mcconf_serialize(b + o, &g_mcconf);
        packet_send(W.reply, b, (uint32_t)o);
        o = 0;
        b[o++] = COMM_GET_APPCONF;
        o += vesc_appconf_serialize(b + o, &g_appconf);
        packet_send(W.reply, b, (uint32_t)o);
        o = 0;
    }
    b[o++] = COMM_DETECT_APPLY_ALL_FOC;
    vb_put_i16(b, (int16_t)res, &o);
    packet_send(W.reply, b, (uint32_t)o);
}

static void finish(int res)
{
    if (res < 0) {
        g_mcconf = W.old;                    /* echec : configuration precedente */
        resc_conf_apply();
        resc_conf_store_mc();
        commands_printf("Detection automatique : echec (%d), configuration precedente remise", res);
    } else {
        resc_conf_apply();
        resc_conf_store_mc();
        if (W.can_master >= 0 && W.activate_status && g_appconf.can_status_msgs_r1 != 0x0F) {
            g_appconf.can_status_msgs_r1 = 0x0F;
            resc_conf_store_app();
        }
        if (W.can_master < 0 && (g_appconf.controller_id != W.id_new || g_appconf.can_status_msgs_r1 != 0x0F) &&
            W.can_devs > 0) {
            g_appconf.controller_id = W.id_new;
            g_appconf.can_status_msgs_r1 = 0x0F;
            resc_conf_store_app();
        }
        static const char *src[3] = { "sans capteur", "sondes Hall", "codeur" };
        commands_printf("Detection automatique terminee : %s, courant max %.1f A (enregistre)",
                        src[res <= 2 ? res : 0], (double)W.i_max);
    }
    reply_result(res);
    W.st = WZ_IDLE;
    W.can_master = -1;
}

void detect_all_request(const comm_iface_t *reply, const uint8_t *d, uint32_t n)
{
    if (W.st != WZ_IDLE || detect_app_busy() || n < 21) {
        uint8_t b[3];
        int32_t o = 0;
        b[o++] = COMM_DETECT_APPLY_ALL_FOC;
        vb_put_i16(b, -1, &o);
        packet_send(reply, b, (uint32_t)o);
        return;
    }
    int32_t i = 0;
    bool can = d[i++] != 0;
    float p = vb_get_f32(d, 1e3f, &i);
    float imin = vb_get_f32(d, 1e3f, &i), imax = vb_get_f32(d, 1e3f, &i);
    float olrpm = vb_get_f32(d, 1e3f, &i), slerpm = vb_get_f32(d, 1e3f, &i);
    vesc_mcconf_t def;
    resc_conf_defaults_mc(&def);
    /* reglages generaux demandes par l'assistant (0 : valeur par defaut) */
    g_mcconf.l_in_current_min = fabsf(imin) > 0.001f ? imin : def.l_in_current_min;
    g_mcconf.l_in_current_max = fabsf(imax) > 0.001f ? imax : def.l_in_current_max;
    g_mcconf.foc_openloop_rpm = fabsf(olrpm) > 0.001f ? olrpm : def.foc_openloop_rpm;
    g_mcconf.foc_sl_erpm = fabsf(slerpm) > 0.001f ? slerpm : def.foc_sl_erpm;
    W.reply = reply;
    commands_set_print_iface(reply);              /* messages de progression dans la console */
    W.can_master = -1;
    W.can_devs = W.can_res_n = W.can_fail = 0;
    W.id_new = resc_controller_id();
    if (can) {
        /* autres controleurs actifs (statut recu depuis moins de 100 ms) : memes reglages, detection
         * en parallele ; si l'un d'eux a notre identifiant, le notre change */
        int np;
        const can_peer_t *pe = can_vesc_peers(&np);
        uint32_t now = systime_ms();
        for (int k = 0; k < np; k++) {
            if (now - pe[k].ms > 100u) continue;
            uint8_t id = pe[k].id;
            can_vesc_conf_in_limits(id, g_mcconf.l_in_current_min, g_mcconf.l_in_current_max);
            can_vesc_conf_foc_erpms(id, g_mcconf.foc_openloop_rpm, g_mcconf.foc_sl_erpm);
            can_vesc_detect_apply_all_foc(id, true, p);
            W.can_devs++;
            if (id == W.id_new) W.id_new = (uint8_t)((W.id_new + 2u) % 255u);
        }
    }
    begin(p);
}

void detect_all_can_request(uint8_t master, bool activate_status, float max_power_loss)
{
    if (W.st != WZ_IDLE || detect_app_busy()) {
        can_vesc_detect_result(master, -1);
        return;
    }
    W.reply = 0;
    W.can_master = master;
    W.activate_status = activate_status;
    W.can_devs = W.can_res_n = W.can_fail = 0;
    W.id_new = resc_controller_id();
    begin(max_power_loss);
}

void detect_all_can_result(int8_t res)
{
    W.can_res_n++;
    if (res < 0) W.can_fail++;
}

void detect_all_poll(uint32_t now)
{
    if (W.st == WZ_IDLE || detect_app_busy()) return;
    int err = detect_app_last_error();
    switch (W.st) {
    case WZ_WAIT_READY:
        /* jusqu'a 5 s pour que les defauts disparaissent, puis 1 s de repos */
        if (foc_state() == FOC_ST_FAULT) {
            if (now - W.t0 > 5000u) finish((int)g_foc_telem.fault - 100);
            break;
        }
        if (now - W.t0 < 1000u || !foc_calibrated() || foc_state() != FOC_ST_IDLE) break;
        if (!detect_app_start_rl(0, 0.0f)) { finish(-1); break; }
        W.st = WZ_RL;
        break;

    case WZ_RL:
        if (err) { finish(err > 0 ? err - 100 : err); break; }
        {
            /* courant max d'apres la puissance de pertes : 1.5 R i^2 = P */
            float r = g_mcconf.foc_motor_r;
            float cap = (float)HWLIM_I_MOTOR_A * resc_current_cap_frac();
            W.i_max = fm_min(sqrtf(W.p_loss / fm_max(r, 1e-5f) / 1.5f), cap);
            commands_printf("Courant max d'apres %.0f W de pertes : %.1f A", (double)W.p_loss, (double)W.i_max);
        }
        if (!detect_app_start_lambda(0, W.i_max / 2.5f, 1800.0f, 0.3f, 0.0f, 0.0f)) { finish(-10); break; }
        W.st = WZ_LAMBDA;
        break;

    case WZ_LAMBDA: {
        if (err) { finish(err > 0 ? err - 100 : -10); break; }
        /* resultats : courants max, gains (deja calcules a chaque mesure), temperature de reference */
        float abs_max = fm_min(1.5f * W.i_max, (float)HWLIM_I_ABS_TRIP_A * resc_current_cap_frac());
        W.old.l_current_max = W.i_max;            /* conserves meme si les capteurs echouent */
        g_mcconf.l_current_max = W.i_max;
        g_mcconf.l_current_min = -W.i_max;
        g_mcconf.l_abs_current_max = abs_max;
        float tm = ctrl_stats()->tmot;
        if (ctrl_limits()->tmot_enabled && tm > -10.0f) {
            g_mcconf.foc_temp_comp_base_temp = tm;
            if (BOARD_PHASE_FILTER_PIN != HW_PIN_NONE) g_mcconf.foc_temp_comp = 1;
        }
        resc_conf_clamp_mc(&g_mcconf);
        resc_conf_apply();
        W.old = g_mcconf;                         /* moteur identifie : base des etapes suivantes */
        W.old.foc_sensor_mode = 0;
        if (!detect_app_start_hall(0, W.i_max / 3.0f)) { finish(0); break; }
        W.t0 = now;
        W.st = WZ_HALL;
        break;
    }

    case WZ_HALL:
        if (!err) {
            g_mcconf.foc_sensor_mode = 2;
            W.result = 1;
        } else {
            /* pas de sondes Hall : codeur absolu s'il est configure, sinon sans capteur */
            uint8_t port = g_mcconf.m_sensor_port_mode;
            if (port >= 2 && port != 4 && port <= 16) {
                g_mcconf.foc_sensor_mode = 1;
                resc_conf_apply();
                if (detect_app_start_encoder(0, W.i_max / 3.0f)) { W.st = WZ_ENC; break; }
            }
            g_mcconf.foc_sensor_mode = 0;
            W.result = 0;
        }
        W.st = WZ_WAIT_CAN;
        W.t0 = now;
        break;

    case WZ_ENC:
        if (!err) { g_mcconf.foc_sensor_mode = 1; W.result = 2; }
        else { g_mcconf.foc_sensor_mode = 0; W.result = 0; }
        W.st = WZ_WAIT_CAN;
        W.t0 = now;
        break;

    case WZ_WAIT_CAN:
        /* attente des autres controleurs (180 s au plus) */
        if (W.can_res_n >= W.can_devs) finish(W.can_fail ? -51 : W.result);
        else if (now - W.t0 > 180000u) finish(-50);
        break;

    default:
        W.st = WZ_IDLE;
        break;
    }
}
