/*
 * ctrl.c — Applications de controle RESC (executees a 1 kHz).
 */
#include <string.h>
#include "ctrl.h"
#include "foc.h"
#include "fmath.h"
#include "fault_codes.h"

#define RAD_TO_ERPM  (60.0f / FM_2PI)

typedef struct {
    ctrl_limits_t l;
    ctrl_stats_t st;

    volatile ctrl_mode_t mode;
    volatile float value;            /* A, duty ou ERPM selon le mode */
    volatile float ol_erpm;          /* vitesse imposee des modes boucle ouverte */
    volatile uint32_t last_cmd_ms;
    volatile bool cmd_flag;          /* nouvelle commande recue */
    volatile float off_delay;        /* s : maintien apres consigne nulle */
    volatile bool estop;
    volatile bool power_ok;

    uint32_t now_ms;
    float iq_out;                    /* reference envoyee au coeur */
    float duty_int;                  /* integrateur du mode duty */
    float s_int, s_prev_err, s_d_filt, s_set;   /* PID vitesse */
    float off_t;
    float last_theta;
    float tacho_acc;
    float lim_in_f;                  /* facteur de limitation du courant d'entree (filtre) */
    float fault_clear_t;
    uint32_t oc_count;
    uint32_t oc_window_ms;
} ctrl_t;

static ctrl_t C;

/* ------------------------------------------------------------- API */
void ctrl_init(const ctrl_limits_t *l)
{
    memset(&C, 0, sizeof(C));
    C.l = *l;
    C.lim_in_f = 1.0f;
    C.st.tfet = C.st.tmot = 25.0f;
}

void ctrl_set_limits(const ctrl_limits_t *l) { C.l = *l; }
const ctrl_limits_t *ctrl_limits(void) { return &C.l; }
ctrl_mode_t ctrl_mode(void) { return C.mode; }
const ctrl_stats_t *ctrl_stats(void) { return &C.st; }

static void command(ctrl_mode_t m, float v)
{
    C.mode = m;
    C.value = v;
    C.last_cmd_ms = C.now_ms;
    C.cmd_flag = true;
    C.st.timeout = false;
}

void ctrl_set_current(float a)      { command(CTRL_MODE_CURRENT, a); }
void ctrl_set_current_rel(float r)
{
    r = fm_sat_sym(r, 1.0f);
    command(CTRL_MODE_CURRENT, r >= 0.0f ? r * C.l.i_max : -r * C.l.i_min);
}
void ctrl_set_brake(float a)        { command(CTRL_MODE_BRAKE, fm_abs(a)); }
void ctrl_set_brake_rel(float r)    { command(CTRL_MODE_BRAKE, fm_abs(fm_sat_sym(r, 1.0f)) * fm_abs(C.l.i_min)); }
void ctrl_set_handbrake(float a)    { command(CTRL_MODE_HANDBRAKE, fm_abs(a)); }
void ctrl_set_duty(float d)         { command(CTRL_MODE_DUTY, fm_sat_sym(d, C.l.duty_max)); }
void ctrl_set_rpm(float erpm)       { command(CTRL_MODE_RPM, erpm); }
void ctrl_release(void)             { command(CTRL_MODE_NONE, 0.0f); }
void ctrl_set_openloop(float a, float erpm)      { C.ol_erpm = erpm; command(CTRL_MODE_OPENLOOP, fm_abs(a)); }
void ctrl_set_openloop_duty(float d, float erpm) { C.ol_erpm = erpm; command(CTRL_MODE_OPENLOOP_DUTY, fm_abs(fm_sat_sym(d, C.l.duty_max))); }
void ctrl_alive(void)               { C.last_cmd_ms = C.now_ms; C.st.timeout = false; }
void ctrl_set_off_delay(float s)    { C.off_delay = fm_sat(s, 0.0f, 10.0f); }
void ctrl_estop(void)               { C.estop = true; command(CTRL_MODE_NONE, 0.0f); }
void ctrl_set_power_allowed(bool a) { C.power_ok = a; }

void ctrl_set_temps(float t1, float t2, float t3, float tm)
{
    C.st.tfet1 = t1;
    C.st.tfet2 = t2;
    C.st.tfet3 = t3;
    float t = fm_max(t1, fm_max(t2, t3));
    FM_LPF(C.st.tfet, t, 0.05f);
    FM_LPF(C.st.tmot, tm, 0.05f);
}

void ctrl_reset_counters(void)
{
    C.st.ah = C.st.ah_chg = C.st.wh = C.st.wh_chg = 0.0f;
    C.st.tacho = C.st.tacho_abs = 0;
}

/* ------------------------------------------------------- limitations */
/* Facteur lineaire 1 -> 0 entre start et end (end > start) */
static float derate_up(float x, float start, float end)
{
    if (end <= start) return x < end ? 1.0f : 0.0f;
    return fm_sat((end - x) / (end - start), 0.0f, 1.0f);
}
/* Facteur lineaire 1 -> 0 quand x descend de start a end (end < start) */
static float derate_down(float x, float start, float end)
{
    if (start <= end) return x > end ? 1.0f : 0.0f;
    return fm_sat((x - end) / (start - end), 0.0f, 1.0f);
}

/* Bornes de courant [lo, hi] apres toutes les limitations */
static void current_window(float erpm, float *lo, float *hi)
{
    const ctrl_limits_t *l = &C.l;
    float vb = C.st.vbus;
    float hi_ = l->i_max * l->i_max_scale;
    float lo_ = l->i_min * l->i_min_scale;

    /* temperatures (motorisation et freinage) */
    float kt = derate_up(C.st.tfet, l->tfet_start, l->tfet_end);
    if (l->tmot_enabled) kt = fm_min(kt, derate_up(C.st.tmot, l->tmot_start, l->tmot_end));
    hi_ *= kt;
    lo_ *= kt;

    /* batterie : motorisation reduite sous cut_start, regeneration au dessus de regen_cut_start */
    float kb = derate_down(vb, l->batt_cut_start, l->batt_cut_end);
    float kr = derate_up(vb, l->regen_cut_start, l->regen_cut_end);

    /* vitesse : la motorisation dans le sens de rotation s'annule a erpm_max */
    float kpos = 1.0f, kneg = 1.0f;
    if (erpm > 0.0f) kpos = derate_up(erpm, l->erpm_start * l->erpm_max, l->erpm_max);
    if (erpm < 0.0f) kneg = derate_up(-erpm, l->erpm_start * -l->erpm_min, -l->erpm_min);

    /* Le sens du courant qui motorise depend du sens de rotation */
    bool fwd = erpm >= 0.0f;
    float k_motor = kb * C.lim_in_f;
    float k_regen = kr;
    if (fwd) {
        hi_ *= k_motor * kpos;
        lo_ *= k_regen;
    } else {
        lo_ *= k_motor * kneg;
        hi_ *= k_regen;
    }
    *hi = hi_;
    *lo = lo_;
    C.st.i_scale_lim = fm_min(kt, fm_min(k_motor, k_regen));
}

/* --------------------------------------------------------- pas 1 kHz */
void ctrl_update(uint32_t now_ms, float dt)
{
    C.now_ms = now_ms;
    const ctrl_limits_t *l = &C.l;
    const foc_telem_t *t = &g_foc_telem;

    /* -- mesures */
    float erpm = t->omega * RAD_TO_ERPM;
    C.st.erpm = erpm;
    C.st.vbus = t->vbus;
    C.st.duty = t->duty;
    C.st.i_in = t->i_bus;
    float i_motor = (t->iq < 0.0f ? -1.0f : 1.0f) * t->i_abs;
    FM_LPF(C.st.i_motor, i_motor, 0.1f);
    C.st.fault = t->fault;

    /* -- compteurs */
    float p = t->i_bus * t->vbus;
    if (t->i_bus >= 0.0f) { C.st.ah += t->i_bus * dt / 3600.0f; C.st.wh += p * dt / 3600.0f; }
    else { C.st.ah_chg -= t->i_bus * dt / 3600.0f; C.st.wh_chg -= p * dt / 3600.0f; }
    {
        float dth = fm_wrap_pi(t->theta - C.last_theta);
        C.last_theta = t->theta;
        if (t->state == FOC_ST_RUN || t->state == FOC_ST_START || t->state == FOC_ST_IDLE) {
            C.tacho_acc += dth;
            const float step = FM_PI / 3.0f;   /* 6 pas par tour electrique */
            while (C.tacho_acc >= step) { C.tacho_acc -= step; C.st.tacho++; C.st.tacho_abs++; }
            while (C.tacho_acc <= -step) { C.tacho_acc += step; C.st.tacho--; C.st.tacho_abs++; }
        }
    }

    /* -- defauts thermiques */
    if (C.st.tfet > l->tfet_end + 5.0f) foc_raise_fault(RESC_FAULT_OVER_TEMP_FET);
    if (l->tmot_enabled && C.st.tmot > l->tmot_end + 5.0f) foc_raise_fault(RESC_FAULT_OVER_TEMP_MOTOR);

    /* -- timeout de commande */
    if (l->timeout_ms > 0 && C.mode != CTRL_MODE_NONE && (now_ms - C.last_cmd_ms) > l->timeout_ms) {
        C.st.timeout = true;
        if (l->timeout_brake > 0.0f) { C.mode = CTRL_MODE_BRAKE; C.value = l->timeout_brake; }
        else { C.mode = CTRL_MODE_NONE; C.value = 0.0f; }
    }

    /* -- limitation du courant d'entree et de la puissance (boucle lente) */
    {
        float k = 1.0f;
        if (l->i_in_max > 0.0f && t->i_bus > l->i_in_max) k = fm_min(k, l->i_in_max / t->i_bus);
        if (l->watt_max > 0.0f && p > l->watt_max) k = fm_min(k, l->watt_max / p);
        FM_LPF(C.lim_in_f, k < 1.0f ? C.lim_in_f * k : 1.0f, 0.05f);
        C.lim_in_f = fm_sat(C.lim_in_f, 0.0f, 1.0f);
    }

    float lo, hi;
    current_window(erpm, &lo, &hi);

    /* -- application du mode */
    float iq = 0.0f;
    bool run = false, allow_start = false;
    uint8_t openloop = 0;
    int8_t dir = 1;
    float v = C.value;

    if (C.estop) {
        C.mode = CTRL_MODE_NONE;
        C.estop = false;
    }

    switch (C.mode) {
    case CTRL_MODE_CURRENT:
        if (fm_abs(v) > l->cc_min_current) {
            iq = v;
            run = true;
            allow_start = true;
            dir = v >= 0.0f ? 1 : -1;
            C.off_t = C.off_delay;
        } else if (C.off_t > 0.0f) {
            /* maintien du pont pendant off_delay (reprise rapide) */
            C.off_t -= dt;
            run = t->state == FOC_ST_RUN;
        }
        break;

    case CTRL_MODE_BRAKE:
        if (fm_abs(erpm) > l->start_erpm_exit && v > l->cc_min_current) {
            iq = erpm > 0.0f ? -v : v;
            run = true;
        }
        break;

    case CTRL_MODE_DUTY:
        if (fm_abs(v) >= l->duty_min) {
            float e = v - t->duty;
            C.duty_int += l->duty_ki * e * dt;
            C.duty_int = fm_sat(C.duty_int, lo, hi);
            iq = l->duty_kp * e + C.duty_int;
            run = true;
            allow_start = true;
            dir = v >= 0.0f ? 1 : -1;
        } else {
            C.duty_int = 0.0f;
        }
        break;

    case CTRL_MODE_RPM: {
        float target = fm_sat(v, l->erpm_min, l->erpm_max);
        if (fm_abs(target) < l->s_min_erpm) {     /* consigne sous le minimum : roue libre */
            C.s_int = 0.0f;
            C.s_set = erpm;
            break;
        }
        /* la rampe de consigne part au moins de la vitesse minimale, dans le sens demande */
        if (fm_abs(C.s_set) < l->s_min_erpm || C.s_set * target < 0.0f) C.s_set = fm_sign(target) * fm_max(l->s_min_erpm, fm_abs(erpm) * (erpm * target > 0.0f ? 1.0f : 0.0f));
        float step = l->s_ramp_erpms > 0.0f ? l->s_ramp_erpms * dt : 1e9f;
        C.s_set += fm_sat_sym(target - C.s_set, step);
        float e = C.s_set - erpm;
        float pt = e * l->s_kp * (1.0f / 20.0f);
        C.s_int += e * l->s_ki * dt * (1.0f / 20.0f);
        float dterm = (e - C.s_prev_err) * (l->s_kd / dt) * (1.0f / 20.0f);
        C.s_prev_err = e;
        FM_LPF(C.s_d_filt, dterm, l->s_kd_filter > 0.0f ? l->s_kd_filter : 1.0f);
        C.s_int = fm_sat_sym(C.s_int, 1.0f);
        float out = fm_sat_sym(pt + C.s_int + C.s_d_filt, 1.0f);
        if (!l->s_allow_braking) {
            if (C.s_set > 0.0f && out < 0.0f) out = 0.0f;
            if (C.s_set < 0.0f && out > 0.0f) out = 0.0f;
        }
        iq = out * (out >= 0.0f ? l->i_max : -l->i_min);
        run = true;
        allow_start = true;
        dir = C.s_set >= 0.0f ? 1 : -1;
        break;
    }

    case CTRL_MODE_OPENLOOP:        /* courant impose, angle tournant a ol_erpm (sens = signe de l'ERPM) */
        if (v > l->cc_min_current) {
            iq = C.ol_erpm >= 0.0f ? v : -v;
            run = true;
            openloop = 1;
        }
        break;

    case CTRL_MODE_OPENLOOP_DUTY:   /* rapport cyclique impose, sans regulation de courant */
        if (v > 0.001f) {
            run = true;
            openloop = 2;
        }
        break;

    case CTRL_MODE_HANDBRAKE:   /* non supporte en sensorless : roue libre */
    case CTRL_MODE_NONE:
    default:
        C.s_int = C.duty_int = 0.0f;
        C.s_set = erpm;
        break;
    }

    if (C.mode != CTRL_MODE_RPM) C.s_set = erpm;

    iq = fm_sat(iq, lo, hi);

    /* -- rearmement automatique des defauts (consigne relachee, condition disparue) */
    if (t->state == FOC_ST_FAULT) {
        run = false;
        if (t->fault == RESC_FAULT_ABS_OVER_CURRENT && C.fault_clear_t == 0.0f) {
            if (now_ms - C.oc_window_ms > 10000) { C.oc_window_ms = now_ms; C.oc_count = 0; }
            C.oc_count++;
        }
        bool released = C.mode == CTRL_MODE_NONE || fm_abs(v) <= l->cc_min_current;
        /* Rearmement automatique seulement si la cause a disparu ; les defauts materiels
         * (driver, break, offsets, watchdog) restent verrouilles jusqu'a 'clear' ou redemarrage. */
        bool cause_gone;
        switch (t->fault) {
        case RESC_FAULT_ABS_OVER_CURRENT: cause_gone = C.oc_count < 3; break;
        case RESC_FAULT_UNDERSPEED:       cause_gone = true; break;
        case RESC_FAULT_OVER_VOLTAGE:     cause_gone = t->vbus < l->regen_cut_start; break;
        case RESC_FAULT_UNDER_VOLTAGE:    cause_gone = t->vbus > l->batt_cut_start; break;
        case RESC_FAULT_OVER_TEMP_FET:    cause_gone = C.st.tfet < l->tfet_start; break;
        case RESC_FAULT_OVER_TEMP_MOTOR:  cause_gone = C.st.tmot < l->tmot_start; break;
        default:                          cause_gone = false; break;
        }
        if (released && cause_gone) C.fault_clear_t += dt; else C.fault_clear_t = dt * 0.5f;
        if (C.fault_clear_t > 0.5f) {
            foc_clear_fault();
            C.fault_clear_t = 0.0f;
        }
    } else {
        C.fault_clear_t = 0.0f;
    }

    if (!C.power_ok) run = false;
    if (!run) openloop = 0;

    C.iq_out = iq;
    g_foc_cmd.id_ref = 0.0f;
    g_foc_cmd.iq_ref = iq;
    g_foc_cmd.dir = dir;
    g_foc_cmd.allow_start = allow_start ? 1 : 0;
    g_foc_cmd.ol_omega = C.ol_erpm * (FM_2PI / 60.0f);
    g_foc_cmd.ol_duty = C.ol_erpm >= 0.0f ? v : -v;
    g_foc_cmd.openloop = openloop;
    g_foc_cmd.run = run ? 1 : 0;
}
