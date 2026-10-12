/*
 * ctrl.c — Applications de controle RESC (executees a 1 kHz).
 */
#include <math.h>
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
    volatile float hyb_lim, hyb_band; /* mode hybride : limite de rapport cyclique et bande (duty) */
    volatile uint32_t last_cmd_ms;
    volatile bool cmd_flag;          /* nouvelle commande recue */
    volatile float off_delay;        /* s : maintien apres consigne nulle */
    volatile bool estop;
    volatile bool power_ok;

    uint32_t now_ms;
    float iq_out;                    /* reference envoyee au coeur */
    float s_int, s_prev_err, s_d_filt, s_set;   /* PID vitesse */
    float off_t;
    float last_theta;
    foc_state_t last_state;
    float tacho_acc;
    float lim_in_f;                  /* facteur de limitation du courant d'entree (filtre) */
    float lim_rg_f;                  /* idem en regeneration (courant batterie negatif, puissance) */
    float bms_lo, bms_hi;            /* limites du BMS */
    float fault_clear_t;
    uint32_t fault_ms;               /* debut du defaut en cours */
    float vbus_f;
    float pos_acc;                   /* degres mecaniques accumules */
    float p_int, p_prev_err, p_d_filt;
    float p_prev_proc, p_dproc_filt;
    float i_in_map_f;                /* courant d'entree filtre (cartographie) */
    uint8_t s_div;
    float s_iq_hold;                 /* sortie du PID vitesse entre deux pas */
    uint32_t oc_count;
    uint32_t oc_window_ms;

    /* Control Settings (consigne de courant relatif) */
    uint8_t x_phase;                 /* 0 avant / neutre, 1 freinage, 2 recul */
    float x_last_rel;                /* dernier courant relatif applique (fondu) */
    float x_fade_from, x_fade_left;
    float x_rev_level;               /* niveau de recul en rampe (courant, duty ou limite) */
    float x_speed_duty;              /* rapport cyclique de la vitesse (FEM), filtre */
    bool  x_armed, x_arm_press, x_press_ok, x_was_neutral;
    float x_neutral_t;
    bool  x_tr_active;
    float x_tr_anchor;               /* degres electriques */
    float x_tr_pos;                  /* position (degres electriques, tachymetre) */
} ctrl_t;

static ctrl_t C;

/* ------------------------------------------------------------- journal */
#define CLOG_N 32
static struct { uint32_t ms; uint8_t code; float val; } s_log[CLOG_N];
static volatile uint32_t s_log_n;

void ctrl_log(uint8_t code, float val)
{
    uint32_t k = s_log_n % CLOG_N;
    s_log[k].ms = C.now_ms;
    s_log[k].code = code;
    s_log[k].val = val;
    s_log_n++;
}
int ctrl_log_count(void) { return s_log_n > CLOG_N ? CLOG_N : (int)s_log_n; }
bool ctrl_log_get(int k, uint32_t *ms, uint8_t *code, float *val)
{
    int n = ctrl_log_count();
    if (k < 0 || k >= n) return false;
    uint32_t first = s_log_n > CLOG_N ? s_log_n - CLOG_N : 0u;
    uint32_t j = (first + (uint32_t)k) % CLOG_N;
    *ms = s_log[j].ms; *code = s_log[j].code; *val = s_log[j].val;
    return true;
}

/* ------------------------------------------------------------- API */
void ctrl_init(const ctrl_limits_t *l)
{
    memset(&C, 0, sizeof(C));
    C.l = *l;
    C.lim_in_f = 1.0f;
    C.lim_rg_f = 1.0f;
    C.bms_lo = -1e9f;
    C.bms_hi = 1e9f;
    C.st.tfet = C.st.tmot = 25.0f;
}

void ctrl_set_limits(const ctrl_limits_t *l) { C.l = *l; }
const ctrl_limits_t *ctrl_limits(void) { return &C.l; }
ctrl_mode_t ctrl_mode(void) { return C.mode; }
float ctrl_pos_set_deg(void) { return C.mode == CTRL_MODE_POS ? C.value + C.l.p_offset : C.pos_acc; }
const ctrl_stats_t *ctrl_stats(void) { return &C.st; }

static void command(ctrl_mode_t m, float v)
{
    if (m != C.mode) ctrl_log(CLOG_MODE, (float)m);
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
    if (C.l.x.brk_enable || C.l.x.tr_enable) {
        /* Control Settings : la consigne est gardee et traitee a chaque pas de 1 kHz */
        if (C.mode != CTRL_MODE_CURRENT_REL) {
            /* nouvelle sequence (apres neutre, timeout ou autre commande) ; l'armement du recul
             * n'est pas efface ici : il suit sa propre temporisation (Reverse Disarm Time) */
            C.x_phase = 0;
            C.x_last_rel = 0.0f;
            C.x_fade_left = 0.0f;
            C.x_rev_level = 0.0f;
            C.x_tr_active = false;
        }
        command(CTRL_MODE_CURRENT_REL, r);
        return;
    }
    command(CTRL_MODE_CURRENT, r >= 0.0f ? r * C.l.i_max : -r * C.l.i_min);
}
void ctrl_set_brake(float a)        { command(CTRL_MODE_BRAKE, fm_abs(a)); }
void ctrl_set_brake_rel(float r)    { command(CTRL_MODE_BRAKE, fm_abs(fm_sat_sym(r, 1.0f)) * fm_abs(C.l.i_min)); }
void ctrl_set_handbrake(float a)    { command(CTRL_MODE_HANDBRAKE, fm_abs(a)); }
void ctrl_set_current_hybrid(float a, float lim, float band)
{
    if (a > 0.0f && C.x_armed) { C.x_armed = C.x_arm_press = false; ctrl_log(CLOG_REV_DISARMED, 2.0f); }
    C.hyb_lim = fm_sat(lim, 0.0f, C.l.duty_max);
    C.hyb_band = fm_max(fm_sat(band, 0.0f, 1.0f) * C.hyb_lim, 0.002f);
    command(CTRL_MODE_CURRENT_HYB, a);
}
void ctrl_set_duty(float d)         { command(CTRL_MODE_DUTY, fm_sat_sym(d, C.l.duty_max)); }
void ctrl_set_rpm(float erpm)       { command(CTRL_MODE_RPM, erpm); }
void ctrl_set_pos(float deg)        { command(CTRL_MODE_POS, deg); }
void ctrl_release(void)             { command(CTRL_MODE_NONE, 0.0f); }
void ctrl_set_openloop(float a, float erpm)      { C.ol_erpm = erpm; command(CTRL_MODE_OPENLOOP, fm_abs(a)); }
void ctrl_set_openloop_duty(float d, float erpm) { C.ol_erpm = erpm; command(CTRL_MODE_OPENLOOP_DUTY, fm_abs(fm_sat_sym(d, C.l.duty_max))); }
void ctrl_alive(void)               { C.last_cmd_ms = C.now_ms; C.st.timeout = false; }
void ctrl_set_off_delay(float s)    { C.off_delay = fm_sat(s, 0.0f, 10.0f); }
void ctrl_estop(void)               { C.estop = true; command(CTRL_MODE_NONE, 0.0f); }
void ctrl_set_power_allowed(bool a) { C.power_ok = a; }
void ctrl_set_bms_limits(float lo, float hi) { C.bms_lo = lo; C.bms_hi = hi; }

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

/* Limites du courant batterie effectives (configuration et BMS) ; 0 dans la configuration : aucune */
static void in_limits(const ctrl_limits_t *l, float *in_min, float *in_max)
{
    *in_max = l->i_in_max > 0.0f ? fm_min(l->i_in_max, C.bms_hi) : C.bms_hi;
    *in_min = l->i_in_min < 0.0f ? fm_max(l->i_in_min, C.bms_lo) : C.bms_lo;
}

/* Bornes de courant [lo, hi] apres toutes les limitations */
static void current_window(float erpm, float *lo, float *hi)
{
    const ctrl_limits_t *l = &C.l;
    float vb = C.st.vbus;
    float hi_ = l->i_max * l->i_max_scale;
    float lo_ = l->i_min * l->i_min_scale;

    /* temperatures : le freinage garde les seuils, la motorisation commence a reduire plus tot
     * (l_temp_accel_dec, fraction de la plage start..end) */
    float kt = derate_up(C.st.tfet, l->tfet_start, l->tfet_end);
    if (l->tmot_enabled) kt = fm_min(kt, derate_up(C.st.tmot, l->tmot_start, l->tmot_end));
    float da = fm_sat(l->temp_accel_dec, 0.0f, 1.0f);
    float kta = derate_up(C.st.tfet, l->tfet_start - da * (l->tfet_end - l->tfet_start), l->tfet_end);
    if (l->tmot_enabled) kta = fm_min(kta, derate_up(C.st.tmot, l->tmot_start - da * (l->tmot_end - l->tmot_start), l->tmot_end));
    /* duty : motorisation reduite de duty_start a duty_max */
    float kd = 1.0f;
    if (l->duty_start < 0.99f && l->duty_max > l->duty_start)
        kd = fm_sat((l->duty_max - fm_abs(C.st.duty)) / (l->duty_max - l->duty_start), 0.0f, 1.0f);

    /* batterie : motorisation reduite sous cut_start, regeneration au dessus de regen_cut_start */
    float kb = derate_down(vb, l->batt_cut_start, l->batt_cut_end);
    float kr = derate_up(vb, l->regen_cut_start, l->regen_cut_end);

    /* vitesse : la motorisation dans le sens de rotation s'annule a erpm_max */
    float kpos = 1.0f, kneg = 1.0f;
    if (erpm > 0.0f) kpos = derate_up(erpm, l->erpm_start * l->erpm_max, l->erpm_max);
    if (erpm < 0.0f) kneg = derate_up(-erpm, l->erpm_start * -l->erpm_min, -l->erpm_min);

    /* Le sens du courant qui motorise depend du sens de rotation */
    bool fwd = erpm >= 0.0f;
    float k_motor = kb * C.lim_in_f * kta * kd;
    /* courant d'entree cartographie : au-dessus de in_map_start x i_in_max, le courant moteur
     * maximal baisse lineairement jusqu'a 0 a i_in_max (limitation plus douce que la regulation) */
    if (l->in_map_start < 0.98f && l->i_in_max > 0.0f && C.i_in_map_f > 0.0f) {
        float frac = C.i_in_map_f / l->i_in_max;
        if (frac > l->in_map_start) k_motor *= fm_sat((1.0f - frac) / (1.0f - l->in_map_start), 0.0f, 1.0f);
    }
    float k_regen = kr * kt * C.lim_rg_f;
    /* courant batterie -> courant moteur (puissance conservee : I_batterie ~ I_moteur x duty), comme
     * VESC ; la boucle lente (lim_in_f, lim_rg_f) corrige ensuite les pertes */
    float in_min, in_max;
    in_limits(l, &in_min, &in_max);
    float dabs = fm_max(fm_abs(C.st.duty), 0.02f);
    float cap_m = in_max < 1e8f ? fm_max(in_max, 0.0f) / dabs : 1e9f;
    float cap_r = in_min > -1e8f ? fm_max(-in_min, 0.0f) / dabs : 1e9f;
    if (fwd) {
        hi_ = fm_min(hi_ * k_motor * kpos, cap_m);
        lo_ = fm_max(lo_ * k_regen, -cap_r);
    } else {
        lo_ = fm_max(lo_ * k_motor * kneg, -cap_m);
        hi_ = fm_min(hi_ * k_regen, cap_r);
    }
    *hi = hi_;
    *lo = lo_;
    C.st.i_scale_lim = fm_min(k_motor, k_regen);
}

/* ------------------------------------------------ Control Settings */
/* Sortie d'un pas : courant (A), rapport cyclique ou frein */
typedef struct {
    uint8_t kind;        /* 0 courant, 1 duty, 2 frein (A, positif) */
    float val;
} x_out_t;

/* Courant relatif signe -> A (convention de ctrl_set_current_rel) */
static float x_amps(float r) { return r >= 0.0f ? r * C.l.i_max : -r * C.l.i_min; }

/* Fondu frein <-> recul */
static float x_blend(float target, float dt)
{
    if (C.x_fade_left <= 0.0f || C.l.x.fade_time <= 0.0f) return target;
    C.x_fade_left -= dt;
    float k = fm_sat(1.0f - C.x_fade_left / C.l.x.fade_time, 0.0f, 1.0f);
    return C.x_fade_from + (target - C.x_fade_from) * k;
}

static void x_set_phase(uint8_t ph, float duty_now)
{
    if (ph == C.x_phase) return;
    if ((C.x_phase == 1 && ph == 2) || (C.x_phase == 2 && ph == 1)) {
        C.x_fade_from = C.x_last_rel;
        C.x_fade_left = C.l.x.fade_time;
    } else {
        C.x_fade_left = 0.0f;
    }
    if (ph == 2) {
        /* rampe du recul : depart du rapport cyclique de recul present (pas d'a-coup en marche) */
        float rd = fm_max(-duty_now, 0.0f);
        C.x_rev_level = C.l.x.rev_mode >= 2 ? rd : 0.0f;
    }
    C.x_phase = ph;
}

/* Reverse Ramp Time : monte de 0 a full en rev_ramp, redescend tout de suite */
static float x_ramp(float goal, float full, float dt)
{
    if (C.l.x.rev_ramp <= 0.0f || C.x_rev_level >= goal) C.x_rev_level = goal;
    else C.x_rev_level = fm_min(C.x_rev_level + full * dt / C.l.x.rev_ramp, goal);
    return C.x_rev_level;
}

/* Re-press to Reverse : le frein ne devient jamais recul. A l'arret, un appui au-dessus de
 * Arm Level parti du neutre arme le recul au relacher ; l'appui suivant recule. Neutre plus long
 * que Disarm Time, ou accelerateur : desarme. */
static void x_repress(float v, float erpm, float dt)
{
    if (v > 0.001f) {
        if (C.x_armed) ctrl_log(CLOG_REV_DISARMED, 2.0f);
        C.x_armed = C.x_arm_press = C.x_press_ok = C.x_was_neutral = false;
        C.x_neutral_t = 0.0f;
        return;
    }
    if (v >= -0.001f) {
        if (C.x_arm_press) { C.x_arm_press = false; C.x_armed = true; C.x_neutral_t = 0.0f; ctrl_log(CLOG_REV_ARMED, 0.0f); }
        C.x_neutral_t += dt;
        if (C.x_armed && C.x_neutral_t > C.l.x.rev_disarm_time) { C.x_armed = false; ctrl_log(CLOG_REV_DISARMED, 1.0f); }
        C.x_was_neutral = true;
        C.x_press_ok = false;
        return;
    }
    if (C.x_was_neutral) {
        C.x_was_neutral = false;
        C.x_press_ok = !C.x_armed && erpm <= C.l.x.stop_erpm;
    }
    C.x_neutral_t = 0.0f;
    if (C.x_press_ok && -v >= C.l.x.rev_arm_level) C.x_arm_press = true;
}

/* Tracteur : ressort virtuel sur la position (tachymetre) autour d'une ancre mobile */
static x_out_t x_tractor(float v, float erpm, float dt)
{
    x_out_t o = { 0, 0.0f };
    const float pos = (float)C.st.tacho * 60.0f;    /* degres electriques */
    if (!C.x_tr_active) { C.x_tr_active = true; C.x_tr_anchor = pos; }
    float target = C.l.x.tr_speed_mode == 1 ? (fm_abs(v) > 0.05f ? fm_sign(v) * C.l.x.tr_max_erpm : 0.0f)
                                            : v * C.l.x.tr_max_erpm;
    C.x_tr_anchor += target / 60.0f * 360.0f * dt;
    C.x_tr_anchor = fm_sat(C.x_tr_anchor, pos - C.l.x.tr_max_anchor, pos + C.l.x.tr_max_anchor);
    float err = pos - C.x_tr_anchor;
    float beyond = fm_max(fm_abs(err) - C.l.x.tr_free_play, 0.0f);
    float sgn = err > 0.0f ? -1.0f : 1.0f;
    float sp = (erpm - target) / 1000.0f;
    if (C.l.x.tr_mode == 0) {
        float imax = fm_min(C.l.x.tr_max_current, C.l.i_max);
        o.val = fm_sat_sym(sgn * C.l.x.tr_stiffness * beyond - C.l.x.tr_damping * sp, imax);
    } else {
        o.kind = 1;
        o.val = fm_sat_sym(sgn * C.l.x.tr_duty_stiffness * beyond - C.l.x.tr_duty_damping * sp,
                           C.l.x.tr_max_duty * C.l.duty_max);
    }
    return o;
}

/* Un pas de 1 kHz de la consigne relative v (-1..1) ; sd : rapport cyclique de la vitesse */
static x_out_t x_step(float v, float erpm, float sd, float dt)
{
    const ctrl_limits_t *l = &C.l;
    x_out_t o = { 0, 0.0f };

    if (l->x.tr_enable) return x_tractor(v, erpm, dt);
    C.x_tr_active = false;

    if (l->x.rev_repress) x_repress(v, erpm, dt);

    if (v >= -0.001f || !l->x.brk_enable) {
        x_set_phase(0, sd);
        C.x_last_rel = v;
        o.val = x_amps(v);
        return o;
    }

    /* freinage ou recul, avec marge de reprise du freinage */
    if (C.x_phase == 2) {
        if (erpm > l->x.stop_erpm * (1.0f + l->x.rebrake_margin)) x_set_phase(1, sd);
    } else {
        x_set_phase(erpm > l->x.stop_erpm ? 1 : 2, sd);
    }

    if (C.x_phase == 1) {
        if (l->x.brk_mode == 1) {
            C.x_last_rel = 0.0f;
            o.kind = 2;
            o.val = -v * fm_abs(l->i_min);
            return o;
        }
        float out = v;
        if (l->x.brk_force_stop < 0.999f && erpm < l->x.soft_stop_erpm && l->x.soft_stop_erpm > l->x.stop_erpm) {
            float f = fm_sat((erpm - l->x.stop_erpm) / (l->x.soft_stop_erpm - l->x.stop_erpm), 0.0f, 1.0f);
            out = v * (l->x.brk_force_stop + (1.0f - l->x.brk_force_stop) * f);
        }
        C.x_last_rel = x_blend(out, dt);
        o.val = x_amps(C.x_last_rel);
        return o;
    }

    /* recul (arret ou deja en recul) ; Re-press : rien tant que non arme */
    if (l->x.rev_repress && l->x.rev_mode != 1 && !C.x_armed) {
        C.x_rev_level = 0.0f;
        C.x_last_rel = x_blend(0.0f, dt);
        o.val = x_amps(C.x_last_rel);
        return o;
    }
    const float lim_full = l->x.rev_duty_max * l->duty_max;
    switch (l->x.rev_mode) {
    case 1:     /* aucun */
        C.x_last_rel = x_blend(0.0f, dt);
        o.val = x_amps(C.x_last_rel);
        break;
    case 2:     /* duty */
        C.x_last_rel = 0.0f;
        o.kind = 1;
        o.val = -x_ramp(-v * lim_full, lim_full, dt);
        break;
    case 3: {   /* courant plein, limite de rapport cyclique fixee par la consigne */
        float lim = x_ramp(-v * lim_full, lim_full, dt);
        float band = fm_max(l->x.rev_soft_zone * lim_full, 0.01f);
        float f = fm_sat((lim + sd) / band, 0.0f, 1.0f);
        C.x_last_rel = x_blend(-l->x.rev_cur_max * f, dt);
        o.val = x_amps(C.x_last_rel);
        break;
    }
    default: {  /* courant */
        float out = -x_ramp(-v * l->x.rev_cur_max, l->x.rev_cur_max, dt);
        if (l->x.rev_duty_max < 0.999f) {
            float band = fm_max(l->x.rev_soft_zone * lim_full, 0.002f);
            out *= fm_sat((lim_full + sd) / band, 0.0f, 1.0f);
        }
        C.x_last_rel = x_blend(out, dt);
        o.val = x_amps(C.x_last_rel);
        break;
    }
    }
    return o;
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
    if (C.vbus_f <= 0.0f) C.vbus_f = t->vbus;
    C.vbus_f += (t->vbus - C.vbus_f) * fm_min(dt / fm_max(l->batt_tau, dt), 1.0f);
    C.st.vbus = C.vbus_f;
    C.st.duty = t->duty;
    C.st.i_in = t->i_bus;
    {
        /* rapport cyclique correspondant a la vitesse (FEM / tension), filtre ~5 ms : les limites de
         * rapport cyclique l'utilisent au lieu du rapport applique, qui contient la chute R x I et
         * suit instantanement le courant commande (oscillation audible) */
        const foc_params_t *fp = foc_params();
        float vs = fm_max(C.vbus_f * FM_1_SQRT3, 1.0f);
        float sd = fp->lambda > 1e-6f ? t->omega * fp->lambda / vs : t->duty;
        C.x_speed_duty += (sd - C.x_speed_duty) * fm_min(dt / 0.005f, 1.0f);
    }
    {
        /* filtre du courant d'entree de la cartographie, defini par periode PWM, ramene a 1 kHz */
        float k = fm_sat(l->in_map_filter, 0.0f, 1.0f);
        float n = l->f_isr > 0.0f ? l->f_isr * dt : 1.0f;
        float kk = k >= 1.0f ? 1.0f : 1.0f - powf(1.0f - k, n);
        C.i_in_map_f += kk * (t->i_bus - C.i_in_map_f);
    }
    /* courant affiche : signe de iq (negatif = frein) seulement si iq compte dans le module ; sinon positif.
     * Sans cela, le frein a main (courant sur l'axe d, iq ~ 0 bruite) change de signe a chaque echantillon */
    float i_motor = (t->iq < -0.25f * t->i_abs ? -1.0f : 1.0f) * t->i_abs;
    FM_LPF(C.st.i_motor, i_motor, 0.1f);
    C.st.fault = t->fault;

    /* -- compteurs */
    float p = t->i_bus * t->vbus;
    if (t->i_bus >= 0.0f) { C.st.ah += t->i_bus * dt / 3600.0f; C.st.wh += p * dt / 3600.0f; }
    else { C.st.ah_chg -= t->i_bus * dt / 3600.0f; C.st.wh_chg -= p * dt / 3600.0f; }
    {
        float dth = fm_wrap_pi(t->theta - C.last_theta);
        C.last_theta = t->theta;
        /* changement d'etat : la source d'angle change (observateur, capteur), saut ignore ;
         * a l'arret pont coupe, l'angle de l'observateur n'est que du bruit */
        bool same = t->state == C.last_state;
        C.last_state = t->state;
        bool moving = t->state != FOC_ST_IDLE || fm_abs(erpm) > 300.0f;
        if (same && moving && (t->state == FOC_ST_RUN || t->state == FOC_ST_START || t->state == FOC_ST_IDLE)) {
            /* position mecanique (degres) : angle electrique / paires de poles / reduction */
            float pp = l->pole_pairs > 0.5f ? l->pole_pairs : 1.0f;
            float div = l->p_ang_div > 0.01f ? l->p_ang_div : 1.0f;
            C.pos_acc += dth * (180.0f / FM_PI) / (pp * div);
            while (C.pos_acc >= 360.0f) C.pos_acc -= 360.0f;
            while (C.pos_acc < 0.0f) C.pos_acc += 360.0f;
            C.st.pos_deg = C.pos_acc;
            C.tacho_acc += dth;
            const float step = FM_PI / 3.0f;   /* 6 pas par tour electrique */
            while (C.tacho_acc >= step) { C.tacho_acc -= step; C.st.tacho++; C.st.tacho_abs++; }
            while (C.tacho_acc <= -step) { C.tacho_acc += step; C.st.tacho--; C.st.tacho_abs++; }
        }
    }

    /* -- defauts thermiques */
    if (C.st.tfet > l->tfet_end + 5.0f) foc_raise_fault(RESC_FAULT_OVER_TEMP_FET);
    if (l->tmot_enabled && C.st.tmot > l->tmot_end + 5.0f) foc_raise_fault(RESC_FAULT_OVER_TEMP_MOTOR);
    /* defauts supplementaires (l_additional_faults) */
    if (t->state == FOC_ST_RUN) {
        float lim = erpm >= 0.0f ? l->erpm_max : -l->erpm_min;
        if ((l->add_faults & 0x08u) && lim > 0.0f && fm_abs(erpm) > 1.3f * lim) foc_raise_fault(RESC_FAULT_ABS_OVERSPEED);
        else if ((l->add_faults & 0x02u) && lim > 0.0f && fm_abs(erpm) > 1.1f * lim) foc_raise_fault(RESC_FAULT_OVERSPEED);
    }

    /* -- re-press : hors consigne relative (neutre, timeout, autre commande), le temps de neutre continue */
    if (C.mode != CTRL_MODE_CURRENT_REL) {
        if (C.x_arm_press) { C.x_arm_press = false; C.x_armed = true; C.x_neutral_t = 0.0f; ctrl_log(CLOG_REV_ARMED, 0.0f); }
        C.x_was_neutral = true;
        C.x_press_ok = false;
        C.x_neutral_t += dt;
        if (C.x_armed && C.x_neutral_t > l->x.rev_disarm_time) { C.x_armed = false; ctrl_log(CLOG_REV_DISARMED, 1.0f); }
    }

    /* -- journal : etat du coeur */
    {
        static foc_state_t prev_st;
        if (t->state != prev_st) { ctrl_log(CLOG_FOC_STATE, (float)t->state); prev_st = t->state; }
    }

    /* -- timeout de commande */
    if (l->timeout_ms > 0 && C.mode != CTRL_MODE_NONE && (now_ms - C.last_cmd_ms) > l->timeout_ms) {
        if (!C.st.timeout) ctrl_log(CLOG_TIMEOUT, (float)(now_ms - C.last_cmd_ms));
        C.st.timeout = true;
        if (l->timeout_brake > 0.0f) { C.mode = CTRL_MODE_BRAKE; C.value = l->timeout_brake; }
        else { C.mode = CTRL_MODE_NONE; C.value = 0.0f; }
    }

    /* -- limitation du courant d'entree et de la puissance (boucle lente) */
    {
        float k = 1.0f;
        float in_min, in_max;
        in_limits(l, &in_min, &in_max);
        if (in_max <= 0.0f && t->i_bus > 0.05f) k = 0.0f;
        else if (t->i_bus > in_max) k = fm_min(k, in_max / t->i_bus);
        if (l->watt_max > 0.0f && p > l->watt_max) k = fm_min(k, l->watt_max / p);
        FM_LPF(C.lim_in_f, k < 1.0f ? C.lim_in_f * k : 1.0f, 0.05f);
        C.lim_in_f = fm_sat(C.lim_in_f, 0.0f, 1.0f);
        /* regeneration : courant batterie (l_in_current_min, BMS) et puissance (l_watt_min) */
        float kr = 1.0f;
        if (in_min >= 0.0f && t->i_bus < -0.05f) kr = 0.0f;
        else if (t->i_bus < in_min) kr = fm_min(kr, in_min / t->i_bus);
        if (l->watt_min < 0.0f && p < l->watt_min) kr = fm_min(kr, l->watt_min / p);
        FM_LPF(C.lim_rg_f, kr < 1.0f ? C.lim_rg_f * kr : 1.0f, 0.05f);
        C.lim_rg_f = fm_sat(C.lim_rg_f, 0.0f, 1.0f);
    }

    float lo, hi;
    current_window(erpm, &lo, &hi);

    /* -- application du mode */
    float iq = 0.0f;
    bool run = false, allow_start = false;
    uint8_t openloop = 0;
    bool duty_mode = false;
    bool brake = false;
    int8_t dir = 1;
    float v = C.value;
    float duty_cmd = v;              /* rapport cyclique envoye en mode duty */

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

    case CTRL_MODE_CURRENT_HYB:
        /* courant reduit lineairement a l'approche de la limite de rapport cyclique (mesure dans le sens
         * demande) ; pont maintenu meme a courant nul pour garder la mesure et eviter les relances */
        if (fm_abs(v) > l->cc_min_current) {
            float d = C.x_speed_duty * (v >= 0.0f ? 1.0f : -1.0f);   /* vitesse, pas le rapport applique */
            iq = v * fm_sat((C.hyb_lim - d) / C.hyb_band, 0.0f, 1.0f);
            run = t->state == FOC_ST_RUN || fm_abs(iq) > l->cc_min_current;
            allow_start = fm_abs(iq) > l->cc_min_current;
            dir = v >= 0.0f ? 1 : -1;
            C.off_t = C.off_delay;
        }
        break;

    case CTRL_MODE_CURRENT_REL: {
        x_out_t o = x_step(v, erpm, C.x_speed_duty, dt);
        if (o.kind == 1) {
            duty_cmd = fm_sat_sym(o.val, l->duty_max);
            if (fm_abs(duty_cmd) >= l->duty_min) {
                duty_mode = true;
                run = true;
                allow_start = true;
                dir = duty_cmd >= 0.0f ? 1 : -1;
            }
        } else if (o.kind == 2) {
            if (o.val > l->cc_min_current) {
                iq = erpm > 0.0f ? -o.val : o.val;
                run = true;
                brake = true;
            }
        } else {
            /* pont maintenu tant que la consigne n'est pas nulle (tenue, frein, reprise) */
            iq = o.val;
            if (fm_abs(iq) > l->cc_min_current || C.l.x.tr_enable) {
                run = true;
                allow_start = fm_abs(iq) > l->cc_min_current;
                dir = iq >= 0.0f ? 1 : -1;
                C.off_t = C.off_delay;
            } else if (fm_abs(v) > 0.001f) {
                run = t->state == FOC_ST_RUN;
            } else if (C.off_t > 0.0f) {
                C.off_t -= dt;
                run = t->state == FOC_ST_RUN;
            }
        }
        break;
    }

    case CTRL_MODE_BRAKE:
        /* le coeur tient le frein jusqu'a l'arret : courant oppose a la vitesse, puis phases en
         * court-circuit (low-sides passants avec foc_short_ls_on_zero_duty) quand il n'est plus atteignable */
        if (v > l->cc_min_current) {
            iq = erpm > 0.0f ? -v : v;
            run = true;
            brake = true;
        }
        break;

    case CTRL_MODE_DUTY:
        /* rapport cyclique applique par le coeur ; le courant n'est que borne */
        if (fm_abs(v) >= l->duty_min) {
            duty_mode = true;
            run = true;
            allow_start = true;
            dir = v >= 0.0f ? 1 : -1;
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
        uint8_t div = l->s_rate_div ? l->s_rate_div : 1;
        if (++C.s_div >= div) {
            C.s_div = 0;
            float sdt = dt * (float)div;
            float w = l->s_speed_src == 0 ? erpm : (l->s_speed_src == 1 ? t->omega_fast : t->omega_raw) * RAD_TO_ERPM;
            float e = C.s_set - w;
            float pt = e * l->s_kp * (1.0f / 20.0f);
            C.s_int += e * l->s_ki * sdt * (1.0f / 20.0f);
            float dterm = (e - C.s_prev_err) * (l->s_kd / sdt) * (1.0f / 20.0f);
            C.s_prev_err = e;
            FM_LPF(C.s_d_filt, dterm, l->s_kd_filter > 0.0f ? l->s_kd_filter : 1.0f);
            C.s_int = fm_sat_sym(C.s_int, 1.0f);
            float out = fm_sat_sym(pt + C.s_int + C.s_d_filt, 1.0f);
            if (!l->s_allow_braking) {
                if (C.s_set > 0.0f && out < 0.0f) out = 0.0f;
                if (C.s_set < 0.0f && out > 0.0f) out = 0.0f;
            }
            C.s_iq_hold = out * (out >= 0.0f ? l->i_max : -l->i_min);
        }
        iq = C.s_iq_hold;
        run = true;
        allow_start = true;
        dir = C.s_set >= 0.0f ? 1 : -1;
        break;
    }

    case CTRL_MODE_POS: {
        /* PID position (degres) -> fraction du courant max ; gains reduits pres de la consigne */
        float set = v + l->p_offset;
        float e = set - C.pos_acc;
        while (e > 180.0f) e -= 360.0f;
        while (e < -180.0f) e += 360.0f;
        float g = (l->p_gain_dec > 0.0f && fm_abs(e) < l->p_gain_dec) ? fm_abs(e) / l->p_gain_dec : 1.0f;
        C.p_int = fm_sat_sym(C.p_int + e * l->p_ki * dt, 1.0f);
        float dterm = (e - C.p_prev_err) * l->p_kd / dt;
        C.p_prev_err = e;
        FM_LPF(C.p_d_filt, dterm, l->p_kd_filter > 0.0f ? l->p_kd_filter : 1.0f);
        /* derivee sur la position mesuree (p_pid_kd_proc) */
        float dp = C.pos_acc - C.p_prev_proc;
        while (dp > 180.0f) dp -= 360.0f;
        while (dp < -180.0f) dp += 360.0f;
        C.p_prev_proc = C.pos_acc;
        FM_LPF(C.p_dproc_filt, -dp * l->p_kd_proc / dt, l->p_kd_filter > 0.0f ? l->p_kd_filter : 1.0f);
        float out = fm_sat_sym(g * (e * l->p_kp + C.p_d_filt + C.p_dproc_filt) + C.p_int, 1.0f);
        iq = out * (out >= 0.0f ? l->i_max : -l->i_min);
        run = true;
        allow_start = false;          /* pas de demarrage I/f : la position demande un angle reel */
        dir = out >= 0.0f ? 1 : -1;
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

    case CTRL_MODE_HANDBRAKE:
        /* frein a main : courant continu a angle fige, maintient le rotor a l'arret */
        if (v > l->cc_min_current) {
            iq = v;
            run = true;
            openloop = 3;
        }
        break;

    case CTRL_MODE_NONE:
    default:
        C.s_int = 0.0f;
        C.s_set = erpm;
        break;
    }

    if (C.mode != CTRL_MODE_RPM) { C.s_set = erpm; C.s_iq_hold = 0.0f; }
    if (C.mode != CTRL_MODE_POS) { C.p_int = 0.0f; C.p_prev_err = 0.0f; C.p_d_filt = 0.0f; C.p_prev_proc = C.pos_acc; C.p_dproc_filt = 0.0f; }

    if (openloop != 3) iq = fm_sat(iq, lo, hi);
    else iq = fm_min(iq, fm_max(hi, -lo));

    /* -- rearmement automatique des defauts (consigne relachee, condition disparue) */
    if (t->state == FOC_ST_FAULT) {
        run = false;
        if (C.fault_ms == 0) C.fault_ms = now_ms ? now_ms : 1;
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
        if (C.fault_clear_t > 0.5f && now_ms - C.fault_ms >= l->fault_stop_ms) {
            foc_clear_fault();
            C.fault_clear_t = 0.0f;
        }
    } else {
        C.fault_clear_t = 0.0f;
        C.fault_ms = 0;
    }

    if (!C.power_ok) run = false;
    if (!run) openloop = 0;
    {
        /* run demande mais le coeur ne demarre pas : signale une fois apres 50 ms */
        static float idle_t;
        static bool logged;
        if (run && t->state == FOC_ST_IDLE) {
            idle_t += dt;
            if (idle_t > 0.05f && !logged) { ctrl_log(CLOG_NO_START, (float)t->state); logged = true; }
        } else {
            idle_t = 0.0f;
            logged = false;
        }
    }

    C.iq_out = iq;
    g_foc_cmd.id_ref = 0.0f;
    g_foc_cmd.iq_ref = iq;
    g_foc_cmd.dir = dir;
    g_foc_cmd.allow_start = allow_start ? 1 : 0;
    g_foc_cmd.ol_omega = C.ol_erpm * (FM_2PI / 60.0f);
    g_foc_cmd.ol_duty = C.ol_erpm >= 0.0f ? v : -v;
    g_foc_cmd.duty = duty_cmd;
    g_foc_cmd.i_hi = hi;
    g_foc_cmd.i_lo = lo;
    g_foc_cmd.duty_mode = (run && duty_mode) ? 1 : 0;
    g_foc_cmd.brake = (run && brake) ? 1 : 0;
    g_foc_cmd.openloop = openloop;
    g_foc_cmd.run = run ? 1 : 0;
}
