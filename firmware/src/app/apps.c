/*
 * apps.c — Applications de commande RESC (1 kHz, contexte SysTick).
 *
 * Chaine commune a chaque application :
 *   entree -> decodage (-1..1 ou 0..1) -> zone morte (hyst) -> courbe -> rampe
 *   -> type de commande (courant, frein, duty, vitesse, position) -> couche ctrl
 *   -> eventuellement les autres controleurs par CAN (multi_esc), avec antipatinage (tc).
 * Une application ne commande que si son entree est valide : sinon elle se tait et
 * le timeout de commande (timeout_msec, timeout_brake_current) relache ou freine.
 *
 * Courbes (RESC) : a = |x|, k = throttle_exp (accel.) ou throttle_exp_brake (frein)
 *   exponentielle : (e^(k a) - 1) / (e^k - 1)
 *   naturelle     : k >= 0 : a^(1+k) ; k < 0 : 1 - (1-a)^(1-k)
 *   polynomiale   : a + k (a^3 - a), k borne a [-0.5, 1]
 * k > 0 adoucit le debut de course, k < 0 le rend plus vif.
 */
#include <math.h>
#include "apps.h"
#include "inputs.h"
#include "ctrl.h"
#include "foc.h"
#include "fault_codes.h"
#include "resc_conf.h"
#include "can_vesc.h"
#include "vesc_defaults.h"
#include "fmath.h"
#include "uart.h"
#include "hw_config.h"

#define RAD_TO_ERPM (60.0f / FM_2PI)

static uint32_t s_now;

typedef enum { OUT_NONE = 0, OUT_CUR_REL, OUT_BRAKE_REL, OUT_DUTY, OUT_RPM, OUT_POS } out_kind_t;

typedef struct {
    float val;             /* sortie rampee (-1..1) */
    bool  safe_ok;         /* demarrage securise satisfait */
    float safe_t;
} chain_t;

static struct {
    uint8_t app;
    bool ppm_on, adc_on, chuk_on, pas_on;
    in_uart_use_t uart_use;
    bool killed;
    bool disable_forever;
    uint32_t disable_until;
    uint32_t last_activity;
    uint32_t last_fault_seen;
    bool out_idle;          /* derniere sortie nulle deja envoyee : l'app laisse la main */
    /* PPM */
    float ppm_ms, ppm_dec;
    float med[5];
    uint8_t med_n, med_i;
    chain_t ch_ppm;
    bool hyst_rev, hyst_armed;
    float smart_duty;
    /* ADC */
    float adc_v[3], adc_f[3], adc_d[3];
    uint32_t adc_next;
    chain_t ch_adc;
    bool cc_on;
    float cc_erpm;
    float adc_sduty;        /* marche arriere intelligente : rapport cyclique rampe */
    /* manette */
    uint8_t js_x, js_y;
    bool bt_c, bt_z, rev_state, has_rev_state;
    uint32_t chuk_ms;
    float chuk_dec;
    chain_t ch_chuk;
    bool chuk_cc;
    float chuk_cc_erpm;
    float coast;
    /* PAS */
    float pas_inst;
    uint32_t pas_next;
    float pas_rpm, pas_out;
    uint32_t pas_last_edge;
} A;

/* --------------------------------------------------------------- outils */
static float curve(float x, float k_acc, float k_brk, uint8_t mode)
{
    float a = fm_min(fm_abs(x), 1.0f);
    float k = x >= 0.0f ? k_acc : k_brk;
    float r;
    switch (mode) {
    case 0:
        r = fm_abs(k) > 1e-3f ? (expf(k * a) - 1.0f) / (expf(k) - 1.0f) : a;
        break;
    case 1:
        r = k >= 0.0f ? powf(a, 1.0f + k) : 1.0f - powf(1.0f - a, 1.0f - k);
        break;
    default:
        k = fm_sat(k, -0.5f, 1.0f);
        r = a + k * (a * a * a - a);
        break;
    }
    return x >= 0.0f ? r : -r;
}

static float deadband(float x, float h)
{
    h = fm_sat(h, 0.0f, 0.9f);
    float a = fm_abs(x);
    if (a <= h) return 0.0f;
    return (x > 0.0f ? 1.0f : -1.0f) * (a - h) / (1.0f - h);
}

/* rampe : t_pos = temps pour 0 -> 1 en module, t_neg = temps pour 1 -> 0 */
static void ramp(chain_t *c, float target, float t_pos, float t_neg, float dt)
{
    bool up = fm_abs(target) > fm_abs(c->val) && target * c->val >= 0.0f;
    float t = up ? t_pos : t_neg;
    float step = t > 0.001f ? dt / t : 2.0f;
    c->val += fm_sat_sym(target - c->val, step);
}

/* demarrage securise : 1 regular (apres demarrage et perte d'entree), 2 aussi apres un defaut */
static bool safe(chain_t *c, uint8_t mode, float raw, bool lost, float dt)
{
    if (mode == 0) return true;
    bool fault = g_foc_telem.state == FOC_ST_FAULT && mode == 2;
    if (lost || fault) { c->safe_ok = false; c->safe_t = 0.0f; }
    if (!c->safe_ok) {
        if (fm_abs(raw) < 0.05f) c->safe_t += dt; else c->safe_t = 0.0f;
        if (c->safe_t > 0.2f) c->safe_ok = true;
    }
    return c->safe_ok;
}

static float erpm_now(void) { return g_foc_telem.omega * RAD_TO_ERPM; }

/* Envoie la commande et la recopie vers les autres controleurs (multi_esc), avec antipatinage */
static void output(out_kind_t k, float v, bool multi, bool tc, float tc_max)
{
    const ctrl_limits_t *l = ctrl_limits();
    float self_scale = 1.0f;
    int np = 0;
    const can_peer_t *peers = can_vesc_peers(&np);
    uint32_t now = s_now;
    float wmin = fm_abs(erpm_now());
    /* Sortie nulle (ni gaz ni frein) : envoyee une seule fois, puis plus rien, pour que les commandes
     * de l'outil ou du CAN (courant, frein a main, ...) ne soient pas ecrasees a chaque cycle */
    if ((k == OUT_CUR_REL || k == OUT_BRAKE_REL || k == OUT_NONE) && fm_abs(v) < 0.001f) {
        if (A.out_idle) return;
        A.out_idle = true;
    } else {
        A.out_idle = false;
    }
    if (multi && tc && k == OUT_CUR_REL && v > 0.0f) {
        for (int i = 0; i < np; i++)
            if (now - peers[i].ms < 100u && fm_abs(peers[i].erpm) < wmin) wmin = fm_abs(peers[i].erpm);
        if (tc_max > 1.0f) self_scale = fm_sat(1.0f - (fm_abs(erpm_now()) - wmin - tc_max) / tc_max, 0.0f, 1.0f);
    }
    switch (k) {
    case OUT_CUR_REL:   ctrl_set_current_rel(v * self_scale); break;
    case OUT_BRAKE_REL: ctrl_set_brake_rel(fm_abs(v)); break;
    case OUT_DUTY:      ctrl_set_duty(v * l->duty_max); break;
    case OUT_RPM:       ctrl_set_rpm(v); break;
    case OUT_POS:       ctrl_set_pos(v); break;
    default:            break;
    }
    if (!multi || k == OUT_POS || k == OUT_NONE) return;
    for (int i = 0; i < np; i++) {
        if (now - peers[i].ms >= 100u) continue;
        uint8_t id = peers[i].id;
        switch (k) {
        case OUT_CUR_REL: {
            float s = 1.0f;
            if (tc && v > 0.0f && tc_max > 1.0f)
                s = fm_sat(1.0f - (fm_abs(peers[i].erpm) - wmin - tc_max) / tc_max, 0.0f, 1.0f);
            can_vesc_send_current_rel(id, v * s);
            break;
        }
        case OUT_BRAKE_REL: can_vesc_send_brake(id, fm_abs(v) * fm_abs(l->i_min)); break;
        case OUT_DUTY:      can_vesc_send_duty(id, v * l->duty_max); break;
        case OUT_RPM:       can_vesc_send_rpm(id, v); break;
        default:            break;
        }
    }
}

/* ------------------------------------------------------------- configuration */
void apps_configure(void)
{
    const vesc_appconf_t *c = &g_appconf;
    A.app = c->app_to_use;
    A.ppm_on = A.app == APP_PPM || A.app == APP_PPM_UART;
    A.adc_on = A.app == APP_ADC || A.app == APP_ADC_UART || A.app == APP_ADC_PAS;
    /* manette (VESC Remote) : donnees recues par paquets (module radio sur l'UART permanent, BLE...) */
    A.chuk_on = A.app == APP_NUNCHUK || A.app == APP_NRF || A.app == APP_UART || A.app == APP_PPM_UART || A.app == APP_ADC_UART;
    A.pas_on = A.app == APP_PAS || A.app == APP_ADC_PAS;
    in_uart_use_t u = IN_UART_COMM;
    if (A.app == APP_ADC) u = IN_UART_BUTTONS;
    if (A.pas_on) u = IN_UART_PAS;
    bool servo = A.ppm_on;
    /* codeur PWM sur l'entree servo (m_sensor_port_mode) : la broche lui appartient */
    bool enc_pwm = g_mcconf.foc_sensor_mode == 1 && (g_mcconf.m_sensor_port_mode == 13 || g_mcconf.m_sensor_port_mode == 14);
    /* sortie servo (servo_out_enable) : sans application PPM et broche libre */
    bool servo_out = g_appconf.servo_out_enable && !servo && !enc_pwm;
    inputs_init(servo && !enc_pwm, u, enc_pwm || servo_out);
    if (servo_out) inputs_servo_out_init();
    if (u == IN_UART_COMM && A.uart_use != IN_UART_COMM) uart_attach_pins(BOARD_UART_INST, BOARD_UART_TX_PIN, BOARD_UART_RX_PIN);
    A.uart_use = u;
}

void apps_disable_output(int32_t ms, uint32_t now)
{
    if (ms == 0) { A.disable_forever = false; A.disable_until = now; return; }
    if (ms < 0) { A.disable_forever = true; ctrl_release(); return; }
    A.disable_until = now + (uint32_t)ms;
    ctrl_release();
}

uint32_t apps_last_activity_ms(void) { return A.last_activity; }
void apps_note_activity(uint32_t now) { A.last_activity = now; }

float apps_ppm_decoded(void) { return A.ppm_dec; }
float apps_ppm_pulse_ms(void) { return A.ppm_ms; }
float apps_adc_decoded(int n) { return A.adc_d[n == 2 ? 2 : 1]; }
float apps_adc_voltage(int n) { return A.adc_v[n == 2 ? 2 : 1]; }
float apps_chuk_decoded(void) { return A.chuk_dec; }

void apps_set_chuck(const uint8_t *d, uint32_t n, uint32_t now)
{
    if (n < 4) return;
    A.js_x = d[0];
    A.js_y = d[1];
    A.bt_c = d[2] != 0;
    A.bt_z = d[3] != 0;
    A.has_rev_state = n >= 12;
    if (A.has_rev_state) A.rev_state = d[11] != 0;
    A.chuk_ms = now ? now : 1;
}

/* ------------------------------------------------------------------- PPM */
static void app_ppm(uint32_t now, float dt)
{
    const vesc_appconf_t *c = &g_appconf;
    float ms;
    bool ok = inputs_servo_ms(&ms, now);
    if (ok && ms != A.med[(A.med_i + 4) % 5]) {
        A.med[A.med_i] = ms;
        A.med_i = (uint8_t)((A.med_i + 1) % 5);
        if (A.med_n < 5) A.med_n++;
    }
    if (ok && c->app_ppm_conf_median_filter && A.med_n == 5) {
        float s[5];
        for (int i = 0; i < 5; i++) s[i] = A.med[i];
        for (int i = 1; i < 5; i++) for (int j = i; j > 0 && s[j] < s[j - 1]; j--) { float t = s[j]; s[j] = s[j - 1]; s[j - 1] = t; }
        ms = s[2];
    }
    A.ppm_ms = ms;
    uint8_t type = c->app_ppm_conf_ctrl_type;
    bool norev = type == 2 || type == 5 || type == 7 || type == 11;
    float st = c->app_ppm_conf_pulse_start, en = c->app_ppm_conf_pulse_end, ce = c->app_ppm_conf_pulse_center;
    float dec;
    if (norev) dec = en > st ? fm_sat((ms - st) / (en - st), 0.0f, 1.0f) : 0.0f;
    else dec = ms >= ce ? fm_sat((ms - ce) / fm_max(en - ce, 1e-3f), 0.0f, 1.0f)
                        : -fm_sat((ce - ms) / fm_max(ce - st, 1e-3f), 0.0f, 1.0f);
    A.ppm_dec = ok ? dec : 0.0f;
    if (!ok || type == 0) { A.ch_ppm.val = 0.0f; safe(&A.ch_ppm, c->app_ppm_conf_safe_start, 1.0f, true, dt); return; }

    float x = deadband(dec, c->app_ppm_conf_hyst);
    if (!safe(&A.ch_ppm, c->app_ppm_conf_safe_start, x, false, dt)) return;
    x = curve(x, c->app_ppm_conf_throttle_exp, c->app_ppm_conf_throttle_exp_brake, c->app_ppm_conf_throttle_exp_mode);
    ramp(&A.ch_ppm, x, c->app_ppm_conf_ramp_time_pos, c->app_ppm_conf_ramp_time_neg, dt);
    float v = A.ch_ppm.val;
    bool multi = c->app_ppm_conf_multi_esc, tc = c->app_ppm_conf_tc;
    float tcm = c->app_ppm_conf_tc_max_diff;
    float erpm = erpm_now();
    float dir_lim = c->app_ppm_conf_max_erpm_for_dir;

    switch (type) {
    case 1: case 2: output(OUT_CUR_REL, v, multi, tc, tcm); break;
    case 3: if (v >= 0.0f) output(OUT_CUR_REL, v, multi, tc, tcm); else output(OUT_BRAKE_REL, v, multi, tc, tcm); break;
    case 4: case 5: output(OUT_DUTY, v, multi, tc, tcm); break;
    case 6: case 7: output(OUT_RPM, v * c->app_ppm_conf_pid_max_erpm, multi, tc, tcm); break;
    case 8: {
        /* marche arriere a hysteresis : frein a fond moteur arrete, puis neutre, inverse le sens */
        if (v < -0.9f && fm_abs(erpm) < dir_lim) A.hyst_armed = true;
        if (A.hyst_armed && fm_abs(v) < 0.05f) { A.hyst_rev = !A.hyst_rev; A.hyst_armed = false; }
        if (v >= 0.0f) output(OUT_CUR_REL, A.hyst_rev ? -v : v, multi, tc, tcm);
        else output(OUT_BRAKE_REL, v, multi, tc, tcm);
        break;
    }
    case 9: {
        /* marche arriere intelligente : frein en roulant, puis recul a duty limite et rampe */
        if (v >= 0.0f) { A.smart_duty = 0.0f; output(OUT_CUR_REL, v, multi, tc, tcm); }
        else if (erpm > dir_lim) output(OUT_BRAKE_REL, v, multi, tc, tcm);
        else {
            float tgt = v * fm_sat(c->app_ppm_conf_smart_rev_max_duty, 0.0f, 1.0f);
            float tr = fm_max(c->app_ppm_conf_smart_rev_ramp_time, 0.01f);
            A.smart_duty += fm_sat_sym(tgt - A.smart_duty, dt / tr);
            A.out_idle = false;
            ctrl_set_duty(A.smart_duty);
        }
        break;
    }
    case 10: output(OUT_POS, v * 180.0f < 0.0f ? v * 180.0f + 360.0f : v * 180.0f, false, false, 0.0f); break;
    case 11: output(OUT_POS, v * 360.0f, false, false, 0.0f); break;
    default: break;
    }
    A.last_activity = now;
}

/* ------------------------------------------------------------------- ADC */
/* Courant relatif c (signe = sens) des types courant de l'ADC, avec les options RESC :
 * marche arriere intelligente (types 2 a 5) et commande hybride courant / rapport cyclique (1 a 8). */
static void adc_drive(float c, uint8_t type, bool multi, bool tc, float tcm, float step)
{
    const vesc_appconf_t *a = &g_appconf;
    const ctrl_limits_t *l = ctrl_limits();
    if (a->app_adc_conf_resc_smart_rev && !l->x.brk_enable && type >= 2 && type <= 5 && c < 0.0f) {   /* Control Settings prioritaire */
        if (erpm_now() > a->app_adc_conf_resc_smart_rev_erpm) {
            /* encore en marche avant : frein proportionnel a la demande */
            A.adc_sduty = 0.0f;
            output(OUT_BRAKE_REL, c, multi, tc, tcm);
        } else {
            /* recul a rapport cyclique limite, atteint par une rampe */
            float dmax = fm_sat(a->app_adc_conf_resc_smart_rev_max_duty, 0.0f, 1.0f);
            float tr = fm_max(a->app_adc_conf_resc_smart_rev_ramp_time, 0.01f);
            A.adc_sduty += fm_sat_sym(c * dmax - A.adc_sduty, dmax * step / tr);   /* 0 -> dmax en tr */
            A.out_idle = false;
            ctrl_set_duty(A.adc_sduty);
        }
        return;
    }
    A.adc_sduty = 0.0f;
    if (a->app_adc_conf_resc_hybrid && type >= 1 && type <= 8 && fm_abs(c) > 0.001f) {
        /* limite de rapport cyclique proportionnelle aux gaz ; le courant decroit lineairement sur
         * la bande sous la limite et s'annule a la limite (pas de coupure brutale). Regulation dans
         * ctrl (1 kHz, pont maintenu) ; les autres controleurs recoivent le courant deja reduit. */
        float lim = fm_abs(c) * fm_sat(a->app_adc_conf_resc_hybrid_duty, 0.0f, 1.0f) * l->duty_max;
        float bf = fm_sat(a->app_adc_conf_resc_hybrid_band, 0.01f, 1.0f);
        A.out_idle = false;
        ctrl_set_current_hybrid(c >= 0.0f ? c * l->i_max : -c * l->i_min, lim, bf);
        if (multi) {
            float d = g_foc_telem.duty * (c > 0.0f ? 1.0f : -1.0f);
            float cr = c * fm_sat((lim - d) / fm_max(bf * lim, 0.002f), 0.0f, 1.0f);
            int np = 0;
            const can_peer_t *peers = can_vesc_peers(&np);
            for (int i = 0; i < np; i++)
                if (s_now - peers[i].ms < 100u) can_vesc_send_current_rel(peers[i].id, cr);
        }
        return;
    }
    output(OUT_CUR_REL, c, multi, tc, tcm);
}

static void app_adc(uint32_t now, float dt)
{
    const vesc_appconf_t *c = &g_appconf;
    uint16_t hz = c->app_adc_conf_update_rate_hz ? c->app_adc_conf_update_rate_hz : 500;
    if ((int32_t)(now - A.adc_next) < 0) return;
    float step = 1.0f / (float)(hz > 1000 ? 1000 : hz);
    A.adc_next = now + (uint32_t)(1000u / (hz > 1000 ? 1000 : hz));
    (void)dt;

    for (int n = 1; n <= 2; n++) {
        float v = inputs_adc_v(n);
        A.adc_v[n] = v;
        if (c->app_adc_conf_use_filter) A.adc_f[n] += (v - A.adc_f[n]) * fm_min(step / 0.02f, 1.0f);
        else A.adc_f[n] = v;
    }
    float v1 = A.adc_f[1], v2 = A.adc_f[2];
    bool valid = v1 >= c->app_adc_conf_voltage_min - 0.15f && v1 <= c->app_adc_conf_voltage_max + 0.15f;
    float s1 = c->app_adc_conf_voltage_start, e1 = c->app_adc_conf_voltage_end, ce = c->app_adc_conf_voltage_center;
    float d1 = e1 != s1 ? fm_sat((v1 - s1) / (e1 - s1), 0.0f, 1.0f) : 0.0f;
    float dc = v1 >= ce ? fm_sat((v1 - ce) / fm_max(fm_abs(e1 - ce), 1e-3f), 0.0f, 1.0f)
                        : -fm_sat((ce - v1) / fm_max(fm_abs(ce - s1), 1e-3f), 0.0f, 1.0f);
    if (c->app_adc_conf_voltage_inverted) { d1 = 1.0f - d1; dc = -dc; }
    float s2 = c->app_adc_conf_voltage2_start, e2 = c->app_adc_conf_voltage2_end;
    float d2 = e2 != s2 ? fm_sat((v2 - s2) / (e2 - s2), 0.0f, 1.0f) : 0.0f;
    if (c->app_adc_conf_voltage2_inverted) d2 = 1.0f - d2;

    uint8_t type = c->app_adc_conf_ctrl_type;
    bool centered = type == 2 || type == 5 || type == 6 || type == 10 || type == 13;
    A.adc_d[1] = centered ? dc : d1;
    A.adc_d[2] = d2;
    if (!valid || type == 0) { safe(&A.ch_adc, c->app_adc_conf_safe_start, 1.0f, true, step); return; }

    /* boutons : bit0 regulateur de vitesse, bit1 inverse le bouton du regulateur, bit2 inverse la marche arriere */
    bool rev = inputs_btn_reverse() ^ ((c->app_adc_conf_buttons & 4u) != 0);
    bool ccb = (c->app_adc_conf_buttons & 1u) && (inputs_btn_cruise() ^ ((c->app_adc_conf_buttons & 2u) != 0));

    float h = c->app_adc_conf_hyst;
    float x = deadband(centered ? dc : d1, h);
    float br = deadband(d2, h);
    if (!safe(&A.ch_adc, c->app_adc_conf_safe_start, x, false, step)) return;
    x = curve(x, c->app_adc_conf_throttle_exp, c->app_adc_conf_throttle_exp_brake, c->app_adc_conf_throttle_exp_mode);
    ramp(&A.ch_adc, x, c->app_adc_conf_ramp_time_pos, c->app_adc_conf_ramp_time_neg, step);
    float v = A.ch_adc.val;
    float dir = rev ? -1.0f : 1.0f;
    bool multi = c->app_adc_conf_multi_esc, tc = c->app_adc_conf_tc;
    float tcm = c->app_adc_conf_tc_max_diff;
    const ctrl_limits_t *l = ctrl_limits();

    /* regulateur de vitesse : maintient la vitesse tant que le bouton est appuye, sans gaz ni frein */
    if (ccb && fm_abs(v) < 0.02f && br < 0.02f && fm_abs(erpm_now()) > 300.0f) {
        if (!A.cc_on) { A.cc_on = true; A.cc_erpm = erpm_now(); }
        output(OUT_RPM, A.cc_erpm, multi, false, 0.0f);
        A.last_activity = now;
        return;
    }
    A.cc_on = false;

    switch (type) {
    case 1:  adc_drive(v, type, multi, tc, tcm, step); break;
    case 2:  adc_drive(v, type, multi, tc, tcm, step); break;
    case 3:  adc_drive(v * dir, type, multi, tc, tcm, step); break;
    case 4:  if (br > 0.0f) output(OUT_BRAKE_REL, br, multi, tc, tcm); else adc_drive(v * dir, type, multi, tc, tcm, step); break;
    case 5:  if (v >= 0.0f) adc_drive(v * dir, type, multi, tc, tcm, step); else output(OUT_BRAKE_REL, v, multi, tc, tcm); break;
    case 6:  if (v >= 0.0f) adc_drive(v, type, multi, tc, tcm, step); else output(OUT_BRAKE_REL, v, multi, tc, tcm); break;
    case 7:  if (rev) output(OUT_BRAKE_REL, v, multi, tc, tcm); else adc_drive(v, type, multi, tc, tcm, step); break;
    case 8:  if (br > 0.0f) output(OUT_BRAKE_REL, br, multi, tc, tcm); else adc_drive(v, type, multi, tc, tcm, step); break;
    case 9:  output(OUT_DUTY, v, multi, tc, tcm); break;
    case 10: output(OUT_DUTY, v, multi, tc, tcm); break;
    case 11: output(OUT_DUTY, v * dir, multi, tc, tcm); break;
    case 12: output(OUT_RPM, v * l->erpm_max, multi, tc, tcm); break;
    case 13: output(OUT_RPM, v >= 0.0f ? v * l->erpm_max : -v * l->erpm_min, multi, tc, tcm); break;
    case 14: output(OUT_RPM, (rev ? -l->erpm_min : l->erpm_max) * v, multi, tc, tcm); break;
    case 15: {
        /* Current Reverse ADC2 : ADC1 avant, ADC2 arriere, courant relatif signe (traite par
         * App > Control Settings). Priorite au frein : ADC2 appuye coupe ADC1. */
        float r = c->app_adc_conf_resc_adc2_brake_prio ? (br > 0.0f ? -br : v) : v - br;
        output(OUT_CUR_REL, fm_sat_sym(r, 1.0f), multi, tc, tcm);
        break;
    }
    default: break;
    }
    A.last_activity = now;
}

/* ---------------------------------------------------- manette (VESC Remote) */
static void app_chuk(uint32_t now, float dt)
{
    const vesc_appconf_t *c = &g_appconf;
    uint32_t tmo = g_appconf.timeout_msec ? g_appconf.timeout_msec : 1000u;
    bool ok = A.chuk_ms && now - A.chuk_ms < tmo;
    float dec = ((float)A.js_y - 128.0f) / 127.0f;
    dec = fm_sat(dec, -1.0f, 1.0f);
    A.chuk_dec = ok ? dec : 0.0f;
    uint8_t type = c->app_chuk_conf_ctrl_type;
    if (!ok || type == 0) { A.ch_chuk.val = 0.0f; A.chuk_cc = false; return; }

    float x = deadband(dec, c->app_chuk_conf_hyst);
    x = curve(x, c->app_chuk_conf_throttle_exp, c->app_chuk_conf_throttle_exp_brake, c->app_chuk_conf_throttle_exp_mode);
    ramp(&A.ch_chuk, x, c->app_chuk_conf_ramp_time_pos, c->app_chuk_conf_ramp_time_neg, dt);
    float v = A.ch_chuk.val;
    bool multi = c->app_chuk_conf_multi_esc, tc = c->app_chuk_conf_tc;
    float tcm = c->app_chuk_conf_tc_max_diff;
    float erpm = erpm_now();
    A.last_activity = now;

    /* regulateur de vitesse (bouton C) : la manette fait varier la consigne */
    if (A.bt_c && fm_abs(erpm) > 300.0f) {
        if (!A.chuk_cc) { A.chuk_cc = true; A.chuk_cc_erpm = erpm; }
        A.chuk_cc_erpm += x * c->app_chuk_conf_stick_erpm_per_s_in_cc * dt;
        output(OUT_RPM, A.chuk_cc_erpm, multi, false, 0.0f);
        return;
    }
    A.chuk_cc = false;

    bool rev = A.has_rev_state ? A.rev_state : A.bt_z;
    if (type == 3) {                                        /* bidirectionnel */
        output(OUT_CUR_REL, v, multi, tc, tcm);
        return;
    }
    if (v > 0.0f) {
        A.coast = 0.0f;
        if (rev && type == 1 && erpm < 1000.0f) {
            if (c->app_chuk_conf_use_smart_rev) {
                float tgt = -v * fm_sat(c->app_chuk_conf_smart_rev_max_duty, 0.0f, 1.0f);
                float tr = fm_max(c->app_chuk_conf_smart_rev_ramp_time, 0.01f);
                A.smart_duty += fm_sat_sym(tgt - A.smart_duty, dt / tr);
                A.out_idle = false;
                ctrl_set_duty(A.smart_duty);
            } else {
                output(OUT_CUR_REL, -v, multi, tc, tcm);
            }
        } else {
            A.smart_duty = 0.0f;
            output(OUT_CUR_REL, v, multi, tc, tcm);
        }
    } else if (v < 0.0f) {
        A.coast = 0.0f;
        output(OUT_BRAKE_REL, v, multi, tc, tcm);
    } else {
        /* manette au neutre : frein de roue libre leger, en rampe */
        float lvl = fm_sat(c->app_chuk_conf_coast_brake_level, 0.0f, 1.0f);
        float tr = fm_max(c->app_chuk_conf_coast_brake_ramp_time, 0.01f);
        A.coast += fm_sat_sym(lvl - A.coast, dt / tr);
        if (A.coast > 0.001f && fm_abs(erpm) > 300.0f) output(OUT_BRAKE_REL, A.coast, multi, false, 0.0f);
        else output(OUT_CUR_REL, 0.0f, multi, false, 0.0f);
    }
}

/* ------------------------------------------------------------------- PAS */
static float app_pas(uint32_t now)
{
    const vesc_appconf_t *c = &g_appconf;
    uint16_t mag = c->app_pas_conf_magnets ? c->app_pas_conf_magnets : 12;
    int32_t d = inputs_pas_poll();
    if (d) {
        /* cadence par la duree entre deux fronts (un aimant = 4 fronts de quadrature) */
        if (c->app_pas_conf_invert_pedal_direction) d = -d;
        uint32_t per = now - A.pas_last_edge;
        if (A.pas_last_edge && per > 0u && per < 500u)
            A.pas_inst = (float)d * 60000.0f / (4.0f * (float)mag * (float)per);
        A.pas_last_edge = now;
    } else if (now - A.pas_last_edge > 500u) {
        A.pas_inst = 0.0f;
    } else if (A.pas_inst != 0.0f) {
        /* pas de nouveau front : la cadence ne peut pas etre superieure a celle du delai ecoule */
        float lim = 60000.0f / (4.0f * (float)mag * (float)(now - A.pas_last_edge));
        if (fm_abs(A.pas_inst) > lim) A.pas_inst = A.pas_inst > 0.0f ? lim : -lim;
    }
    uint16_t hz = c->app_pas_conf_update_rate_hz ? c->app_pas_conf_update_rate_hz : 50;
    if (hz > 1000) hz = 1000;
    if ((int32_t)(now - A.pas_next) >= 0) {
        float t = 1.0f / (float)hz;
        A.pas_next = now + 1000u / hz;
        if (c->app_pas_conf_use_filter) A.pas_rpm += (A.pas_inst - A.pas_rpm) * fm_min(t / 0.3f, 1.0f);
        else A.pas_rpm = A.pas_inst;
        float tgt = 0.0f;
        float st = c->app_pas_conf_pedal_rpm_start, en = c->app_pas_conf_pedal_rpm_end;
        float sc = fm_sat(c->app_pas_conf_current_scaling, 0.0f, 1.0f);
        if (c->app_pas_conf_ctrl_type == 1)
            tgt = en > st ? fm_sat((A.pas_rpm - st) / (en - st), 0.0f, 1.0f) * sc : 0.0f;
        else if (c->app_pas_conf_ctrl_type == 2)
            tgt = A.pas_rpm > st ? sc : 0.0f;
        float tr = tgt > A.pas_out ? c->app_pas_conf_ramp_time_pos : c->app_pas_conf_ramp_time_neg;
        float stp = tr > 0.001f ? t / tr : 1.0f;
        A.pas_out += fm_sat_sym(tgt - A.pas_out, stp);
    }
    return A.pas_out;
}

/* ------------------------------------------------------------------ pas */
static bool kill_switch(void)
{
    switch (g_appconf.kill_sw_mode) {
    case 1: return !inputs_servo_level();
    case 2: return inputs_servo_level();
    case 3: return inputs_adc_v(2) < 1.65f;
    case 4: return inputs_adc_v(2) >= 1.65f;
    default: return false;
    }
}

void apps_tick_1khz(uint32_t now)
{
    const float dt = 0.001f;
    s_now = now;
    if (fm_abs(erpm_now()) > 100.0f) A.last_activity = now;

    bool killed = kill_switch();
    if (killed && !A.killed) ctrl_release();
    A.killed = killed;
    bool disabled = A.disable_forever || (int32_t)(A.disable_until - now) > 0;
    if (killed || disabled) {
        A.ch_ppm.safe_ok = A.ch_adc.safe_ok = false;
        A.out_idle = true;
        return;
    }

    if (A.ppm_on) app_ppm(now, dt);
    if (A.adc_on && !A.pas_on) app_adc(now, dt);
    if (A.chuk_on) app_chuk(now, dt);
    if (A.pas_on) {
        float p = app_pas(now);
        if (A.adc_on) {
            /* ADC + PAS : la poignee prime sur l'assistance */
            app_adc(now, dt);
            if (A.ch_adc.val < 0.02f && p > 0.0f) { A.out_idle = false; ctrl_set_current_rel(p); A.last_activity = now; }
        } else if (p > 0.0f || fm_abs(A.pas_rpm) > 1.0f) {
            A.out_idle = false;
            ctrl_set_current_rel(p);
            A.last_activity = now;
        }
    }
}
