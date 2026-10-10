/*
 * foc.c — Coeur FOC RESC.
 *
 * Une execution de foc_isr() par periode PWM, declenchee par la fin des
 * conversions ADC injectees (echantillon au sommet du compteur, low-sides
 * passants). Sequence :
 *   mesures -> protections -> reconstruction des courants -> observateur
 *   -> machine d'etats (demarrage, marche, identification) -> boucle de courant
 *   -> limitation de tension -> Park inverse avec angle predit -> SVPWM.
 */
#include <string.h>
#include "foc.h"
#include "observer.h"
#include "fmath.h"
#include "fault_codes.h"
#include "platform.h"

/* ------------------------------------------------------------------ etat */
typedef enum { ST_ALIGN = 0, ST_RAMP, ST_FORCED } start_sub_t;   /* ST_FORCED : boucle ouverte commandee */

typedef enum {
    DS_IDLE = 0,
    DS_R_SEARCH, DS_R_STEP,
    DS_L_D, DS_L_Q,
    DS_LAM_ALIGN, DS_LAM_RAMP, DS_LAM_HOLD, DS_LAM_COAST,
    DS_OFF_OFF, DS_OFF_ON,
    DS_SEN_ALIGN, DS_SEN_RUN,
    DS_END
} det_step_t;

typedef struct {
    foc_params_t p;
    foc_params_t p_next;
    volatile bool p_pending;
    obs_cfg_t oc;
    obs_t obs;
    float dt;

    foc_state_t st;
    float st_time;

    /* calibration */
    uint32_t cal_n;
    float cal_acc[3];
    float off[3];
    bool calibrated;

    /* mesures */
    float i_ph[3];
    float i_a, i_b;
    float id, iq;
    float vbus, vbus_f;

    /* regulation */
    float id_ref, iq_ref;
    float int_d, int_q;
    float vd, vq;
    float duty[3], duty_p1[3], duty_p2[3];
    bool  br, br_p1, br_p2;
    float theta, omega;
    float slew, slew_t;
    bool  duty_on;
    float vq_duty, int_hi, int_lo;

    /* demarrage */
    start_sub_t sub;
    float ol_theta, ol_omega, ol_iq;
    int8_t dir;
    float lock_t, low_t, flux_bad_t;

    /* identification */
    foc_det_t det;
    det_step_t ds;
    float dst;
    float acc_v, acc_i, acc_n;
    int r_step;
    float rv[3], ri[3];
    float vd_bias;
    float inj_v;
    int8_t inj_s0, inj_s1, inj_s2;
    float inj_acc, inj_n;
    float i_prev;
    uint8_t inj_phase;
    float lam_ed, lam_eq, lam_w;
    float lam_driven, lam_vacc;
    float r_ihi;

    /* capteurs d'angle */
    uint16_t raw_cs[3];
    float vph_raw[3];
    uint8_t hall_now;
    uint32_t enc_now;
    hall_t hall;
    enc_t enc;
    bool use_obs;
    float hall_bad_t;

    /* identification : offsets et capteurs */
    float acc3[3], accv[3], off_tmp[3];
    float hs[8], hc[8], hn[8];
    int32_t enc_acc;
    uint32_t enc_prev;
    enc_cfg_t ecfg;
    float sen_rot, off_sin, off_cos;
    int sen_phase;

    uint32_t fault;
    volatile bool clear_req;
} foc_t;

static CCMBSS foc_t S;
foc_cmd_t   g_foc_cmd;
foc_telem_t g_foc_telem;

/* --------------------------------------------------------- utilitaires */
static void set_state(foc_state_t s)
{
    S.st = s;
    S.st_time = 0.0f;
    g_foc_telem.state = s;
}

static void obs_cfg_from_params(void)
{
    S.oc.r = S.p.r;
    S.oc.ld = S.p.ld;
    S.oc.lq = S.p.lq;
    S.oc.lambda = S.p.lambda > 1e-6f ? S.p.lambda : 1e-6f;
    /* gamma * lambda^2 * dt <= 0.2 pour la stabilite en discret */
    float g = S.p.obs_gamma;
    float gmax = 0.2f / (S.oc.lambda * S.oc.lambda * S.dt);
    S.oc.gamma = g < gmax ? g : gmax;
    S.oc.pll_kp = S.p.pll_kp;
    S.oc.pll_ki = S.p.pll_ki;
}

static void apply_params(const foc_params_t *p)
{
    S.p = *p;
    S.dt = 1.0f / S.p.f_pwm;
    obs_cfg_from_params();
}

static bool motor_params_ok(void)
{
    return S.p.r > 1e-5f && S.p.ld > 1e-8f && S.p.lq > 1e-8f && S.p.lambda > 1e-6f;
}

RAMFUNC static void trip(uint32_t code)
{
    foc_hw_kill();
    S.br = false;
    if (S.fault == RESC_FAULT_NONE) {
        S.fault = code;
        g_foc_telem.fault = code;
    }
    if (S.det.busy) {
        S.det.error = (int)code;
        S.det.busy = false;
        S.det.done = true;
        S.ds = DS_IDLE;
    }
    set_state(FOC_ST_FAULT);
}

void foc_raise_fault(uint32_t code) { trip(code); }

bool foc_clear_fault(void)
{
    if (S.st != FOC_ST_FAULT) return true;
    S.clear_req = true;
    return true;
}

/* ------------------------------------------------------------- API */
void foc_init(const foc_params_t *p)
{
    memset(&S, 0, sizeof(S));
    memset(&g_foc_telem, 0, sizeof(g_foc_telem));
    apply_params(p);
    for (int k = 0; k < 3; k++) {
        S.duty[k] = S.duty_p1[k] = S.duty_p2[k] = 0.5f;
        S.off[k] = 2048.0f;
    }
    obs_reset(&S.obs, &S.oc, 0.0f, 0.0f);
    hall_reset(&S.hall);
    enc_reset(&S.enc);
    g_foc_cmd.run = 0;
    g_foc_cmd.dir = 1;
    set_state(FOC_ST_CALIB);
}

void foc_set_params(const foc_params_t *p)
{
    S.p_next = *p;
    S.p_pending = true;
}

const foc_params_t *foc_params(void) { return &S.p; }
foc_state_t foc_state(void) { return S.st; }
bool foc_calibrated(void) { return S.calibrated; }

bool foc_detect_start(const foc_det_t *req)
{
    if (S.st != FOC_ST_IDLE || !S.calibrated || S.det.busy || req->kind == FOC_DET_NONE) return false;
    if (req->kind == FOC_DET_LAMBDA && !(req->r_known > 1e-5f && req->l_known > 1e-8f)) return false;
    foc_det_t d = *req;
    d.busy = true;
    d.done = false;
    d.error = 0;
    d.r = d.ld = d.lq = d.lambda = 0.0f;
    d.enc_offset = d.enc_ratio = 0.0f;
    d.enc_inverted = false;
    d.r_i_low = d.r_i_high = d.r_v_high = d.r_v_dt = 0.0f;
    d.warn = 0;
    for (int k = 0; k < 8; k++) { d.hall_angle[k] = 0.0f; d.hall_valid[k] = false; }
    for (int k = 0; k < 3; k++) { d.off_cs[k] = 0.0f; d.off_vph[k] = 0.0f; d.off_vph_drv[k] = 0.0f; }
    if ((req->kind == FOC_DET_HALL || req->kind == FOC_DET_ENCODER) && d.omega_test <= 0.0f) d.omega_test = FM_2PI * 2.0f;
    if (req->kind == FOC_DET_ENCODER && S.p.enc.cpr < 4) return false;
    S.det = d;
    S.ds = DS_IDLE;     /* l'ISR demarre la sequence */
    return true;
}

const foc_det_t *foc_detect_status(void) { return &S.det; }

void foc_detect_abort(void)
{
    if (S.det.busy) {
        S.det.error = -1;
        S.det.busy = false;
        S.det.done = true;
    }
}

/* ----------------------------------------------------- briques ISR */

/* Regulateur PI d/q avec anticipation et anti-windup par poursuite.
 * Retourne true si la tension a sature. */
RAMFUNC static bool current_loop(float kp, float ki, float vd_ff, float vq_ff, float vmax)
{
    float ed = S.id_ref - S.id;
    float eq = S.iq_ref - S.iq;
    S.int_d += ki * S.dt * ed;
    S.int_q += ki * S.dt * eq;
    float vd = kp * ed + S.int_d + vd_ff;
    float vq = kp * eq + S.int_q + vq_ff;

    /* Priorite a l'axe d (defluxage, stabilite), puis q dans le reste */
    float vd_s = fm_sat_sym(vd, vmax);
    float vq_lim = fm_sqrt(vmax * vmax - vd_s * vd_s);
    float vq_s = fm_sat_sym(vq, vq_lim);
    bool sat = (vd_s != vd) || (vq_s != vq);
    if (vd_s != vd) S.int_d = vd_s - kp * ed - vd_ff;
    if (vq_s != vq) S.int_q = vq_s - kp * eq - vq_ff;
    S.vd = vd_s;
    S.vq = vq_s;
    return sat;
}

/* Mode duty : tension q imposee (rampe de 0.1 s pleine echelle), axe d regule.
 * Le courant n'intervient qu'aux bornes : deux integrateurs a sens unique
 * reduisent (ou augmentent) la tension tant que iq sort de [i_lo, i_hi],
 * puis se relachent vers zero. Pas de boucle sur le duty mesure. */
RAMFUNC static void duty_loop(float kp, float ki, float vmax)
{
    const float dt = S.dt;
    if (!S.duty_on) {
        S.duty_on = true;
        S.vq_duty = S.vq;
        S.int_hi = S.int_lo = 0.0f;
    }
    float tgt = fm_sat_sym(g_foc_cmd.duty * S.vbus_f * FM_1_SQRT3, vmax);
    S.vq_duty += fm_sat_sym(tgt - S.vq_duty, vmax * dt / 0.1f);

    float ehi = g_foc_cmd.i_hi - S.iq, elo = g_foc_cmd.i_lo - S.iq;
    S.int_hi = fm_min(S.int_hi + ki * dt * ehi, 0.0f);
    S.int_lo = fm_max(S.int_lo + ki * dt * elo, 0.0f);
    float corr = S.int_hi + S.int_lo + kp * (fm_min(ehi, 0.0f) + fm_max(elo, 0.0f));

    float ed = S.id_ref - S.id;
    float vd_ff = -S.omega * S.p.lq * S.iq;
    S.int_d += ki * dt * ed;
    float vd = kp * ed + S.int_d + vd_ff;
    float vd_s = fm_sat_sym(vd, vmax);
    if (vd_s != vd) S.int_d = vd_s - kp * ed - vd_ff;
    float vq_lim = fm_sqrt(vmax * vmax - vd_s * vd_s);
    S.vq_duty = fm_sat_sym(S.vq_duty, vq_lim);
    S.vd = vd_s;
    S.vq = fm_sat_sym(S.vq_duty + corr, vq_lim);
    S.iq_ref = S.iq;                       /* reprise sans a-coup si on quitte le mode */
}

/* Park inverse avec angle predit + SVPWM (injection min-max) */
RAMFUNC static void modulate(float theta_out)
{
    float s, c;
    fm_sincos(theta_out, &s, &c);
    float va = c * S.vd - s * S.vq;
    float vb = s * S.vd + c * S.vq;

    float pa = va;
    float pb = -0.5f * va + FM_SQRT3_2 * vb;
    float pc = -0.5f * va - FM_SQRT3_2 * vb;
    float mx = fm_max(pa, fm_max(pb, pc));
    float mn = fm_min(pa, fm_min(pb, pc));
    float off = -0.5f * (mx + mn);
    float inv = 1.0f / fm_max(S.vbus_f, 1.0f);
    const float dmax = 0.995f;
    S.duty[0] = fm_sat(0.5f + (pa + off) * inv, 0.0f, dmax);
    S.duty[1] = fm_sat(0.5f + (pb + off) * inv, 0.0f, dmax);
    S.duty[2] = fm_sat(0.5f + (pc + off) * inv, 0.0f, dmax);
}

RAMFUNC static void bridge_off(void)
{
    S.br = false;
    S.duty_on = false;
    S.int_d = S.int_q = 0.0f;
    S.vd = S.vq = 0.0f;
    S.duty[0] = S.duty[1] = S.duty[2] = 0.5f;
}

/* Reconstruit (i_a, i_b) a partir des phases valides. */
RAMFUNC static void reconstruct_currents(const bool valid[3])
{
    int n = (int)valid[0] + (int)valid[1] + (int)valid[2];
    float a = S.i_ph[0], b = S.i_ph[1], c = S.i_ph[2];
    if (S.p.cs_two_shunts) {
        if (valid[0] && valid[1]) {
            c = -(a + b);
            n = 3;
        } else {
            n = 0;
        }
    }
    if (n == 3 && !S.p.cs_two_shunts) {
        S.i_a = (2.0f * a - b - c) * (1.0f / 3.0f);
        S.i_b = (b - c) * FM_1_SQRT3;
    } else if (n == 3) {
        S.i_a = a;
        S.i_b = (a + 2.0f * b) * FM_1_SQRT3;
    } else if (n == 2) {
        if (!valid[0]) a = -(b + c);
        else if (!valid[1]) b = -(a + c);
        else c = -(a + b);
        S.i_a = a;
        S.i_b = (a + 2.0f * b) * FM_1_SQRT3;
    } else {
        /* Prediction : vecteur courant tourne a la vitesse estimee */
        float s, co;
        fm_sincos(S.omega * S.dt, &s, &co);
        float ia = co * S.i_a - s * S.i_b;
        float ib = s * S.i_a + co * S.i_b;
        S.i_a = ia;
        S.i_b = ib;
        g_foc_telem.n_predicted++;
    }
}

/* Tension reellement appliquee pendant le dernier intervalle (alpha, beta) */
RAMFUNC static void applied_voltage(const float vph_meas[3], float *va, float *vb)
{
    float v[3];
    if (S.br_p1 || S.br_p2) {
        float k = S.vbus_f;
        float tc = S.p.dt_comp * S.p.f_pwm * S.vbus_f;      /* volts perdus pendant le temps mort */
        for (int x = 0; x < 3; x++) {
            float d1 = S.br_p1 ? S.duty_p1[x] : 0.5f;
            float d2 = S.br_p2 ? S.duty_p2[x] : d1;
            v[x] = 0.5f * (d1 + d2) * k - tc * fm_sat_sym(S.i_ph[x], 1.0f);
        }
    } else {
        v[0] = vph_meas[0];
        v[1] = vph_meas[1];
        v[2] = vph_meas[2];
    }
    *va = (2.0f * v[0] - v[1] - v[2]) * (1.0f / 3.0f);
    *vb = (v[1] - v[2]) * FM_1_SQRT3;
}

/* ---------------------------------------------- demarrage I/f et relais */
RAMFUNC static void enter_start(bool from_run)
{
    if (from_run) {
        /* Reprise en I/f a partir de l'etat observe, courant sur l'axe q rotor */
        S.ol_theta = S.obs.theta;
        S.ol_omega = S.obs.omega;
        S.sub = ST_RAMP;
        S.ol_iq = (float)S.dir * fm_max(S.p.st_i, fm_abs(S.iq_ref));
        /* Le PI tourne sans anticipation en I/f : on y reporte la FEM */
        S.int_d += -S.ol_omega * S.p.lq * S.iq_ref;
        S.int_q += S.ol_omega * (S.p.ld * S.id_ref + S.p.lambda);
        S.id_ref = 0.0f;
        S.iq_ref = S.ol_iq;
    } else {
        S.ol_theta = 0.0f;
        S.ol_omega = 0.0f;
        S.sub = ST_ALIGN;
        S.ol_iq = 0.0f;
        S.id_ref = S.iq_ref = 0.0f;
        S.int_d = S.int_q = 0.0f;
    }
    S.lock_t = 0.0f;
    set_state(FOC_ST_START);
}

/* Boucle ouverte commandee (foc_openloop, foc_openloop_duty) : angle impose,
 * jamais de relais vers l'observateur. Reprend l'angle courant si le pont tourne. */
RAMFUNC static void enter_forced(void)
{
    if (S.st == FOC_ST_RUN || S.st == FOC_ST_START) {
        S.ol_theta = S.theta;
        S.ol_omega = S.omega;
        S.ol_iq = S.iq_ref;
    } else {
        S.ol_theta = S.obs.theta;
        S.ol_omega = 0.0f;
        S.ol_iq = 0.0f;
        S.int_d = S.int_q = 0.0f;
        S.vd = S.vq = 0.0f;
    }
    S.id_ref = 0.0f;
    S.sub = ST_FORCED;
    set_state(FOC_ST_START);
}

/* Relais I/f -> observateur. Le rotor oscille autour de l'angle impose (pendule non
 * amorti) : tourner les references dans le repere observe pourrait mettre le courant
 * sur l'axe d (couple nul) et faire decrocher. Le courant de demarrage est donc place
 * entierement sur l'axe q observe (couple maximal) ; le PI rattrape l'echelon en
 * quelques periodes. L'integrateur reprend la chute ohmique, l'anticipation porte la FEM. */
RAMFUNC static void handoff_to_run(void)
{
    float imag = fm_sqrt(S.id_ref * S.id_ref + S.iq_ref * S.iq_ref);
    S.id_ref = 0.0f;
    S.iq_ref = (float)S.dir * imag;
    S.int_d = 0.0f;
    S.int_q = S.p.r * S.iq_ref;
    S.slew = S.p.st_i * 5.0f;      /* rejoint doucement la consigne (200 ms) */
    S.slew_t = 0.3f;
    S.low_t = S.flux_bad_t = 0.0f;
    set_state(FOC_ST_RUN);
}

/* --------------------------------------------------- identification */
RAMFUNC static void det_finish(int err)
{
    S.det.error = err;
    S.det.busy = false;
    S.det.done = true;
    S.ds = DS_IDLE;
    bridge_off();
    set_state(FOC_ST_IDLE);
}

/* Gains prudents pour l'identification (parametres moteur inconnus) */
RAMFUNC static void det_gains(float *kp, float *ki)
{
    float l = S.det.l_known > 1e-8f ? S.det.l_known : 20e-6f;
    float r = S.det.r_known > 1e-5f ? S.det.r_known : 0.0f;
    *kp = l * 1000.0f;
    *ki = r > 0.0f ? r * 1000.0f : *kp * 200.0f;
}

RAMFUNC static void det_step(float vmax, const float vph[3])
{
    const float dt = S.dt;
    float kp, ki;
    det_gains(&kp, &ki);
    S.dst += dt;
    float I = S.det.i_test;

    switch (S.ds) {
    case DS_IDLE:
        S.br = true;
        S.int_d = S.int_q = 0.0f;
        S.id_ref = S.iq_ref = 0.0f;
        S.dst = 0.0f;
        S.acc_v = S.acc_i = S.acc_n = 0.0f;
        switch (S.det.kind) {
        case FOC_DET_RL:      S.ds = DS_R_SEARCH; break;
        case FOC_DET_LAMBDA:  S.ds = DS_LAM_ALIGN; break;
        case FOC_DET_OFFSETS:
            if (fm_abs(S.obs.omega) > 0.3f * S.p.st_omega_exit) { det_finish(-9); return; }   /* moteur en rotation */
            S.ds = DS_OFF_OFF;
            for (int x = 0; x < 3; x++) S.acc3[x] = S.accv[x] = 0.0f;
            break;
        default:              S.ds = DS_SEN_ALIGN; break;
        }
        /* R/L : vecteur sur l'axe beta -> phase A sans courant, B et C a +-0.87 I.
         * Une phase a courant nul a une erreur de temps mort nulle et constante. */
        S.ol_theta = (S.det.kind == FOC_DET_RL) ? 0.5f * FM_PI : 0.0f;
        S.ol_omega = 0.0f;
        break;

    /* --- Resistance, etape 1 : rampe (0.5 s) jusqu'au courant de mesure det.i_test,
     *     fixe par l'application (fraction du courant moteur max configure).
     *     Moteur tres resistant : la rampe s'arrete si |vd| depasse 30 % de la tension
     *     disponible ; le palier haut est alors le courant atteint. */
    case DS_R_SEARCH:
        S.id_ref = fm_min(S.id_ref + I * dt / 0.5f, I);
        if (fm_abs(S.vd) > 0.3f * vmax) S.id_ref = fm_max(S.id - 0.02f * I, 0.1f * I);
        if (S.id_ref >= I || (fm_abs(S.vd) > 0.3f * vmax && S.dst > 0.1f)) {
            S.r_ihi = S.id_ref;
            S.det.r_i_high = S.r_ihi;
            S.det.r_i_low = 0.5f * S.r_ihi;
            S.r_step = 0;
            S.ds = DS_R_STEP;
            S.dst = 0.0f;
            S.acc_v = S.acc_i = S.acc_n = 0.0f;
        }
        break;

    /* --- Resistance, etape 2 : trois paliers montants 0.5, 0.75 et 1 x I.
     *     R = pente dV/dI : l'erreur de temps mort, constante sur les paliers, s'annule.
     *     Si la pente du bas est nettement plus forte que celle du haut, le temps mort
     *     n'etait pas sature au palier bas : on ne garde que les deux paliers hauts. */
    case DS_R_STEP: {
        float target = S.r_ihi * (0.5f + 0.25f * (float)S.r_step);
        S.id_ref += fm_sat_sym(target - S.id_ref, I * dt / 0.1f);
        if (S.dst > 0.3f) { S.acc_v += S.vd; S.acc_i += S.id; S.acc_n += 1.0f; }
        if (S.dst >= 0.8f) {
            S.rv[S.r_step] = S.acc_v / S.acc_n;
            S.ri[S.r_step] = S.acc_i / S.acc_n;
            S.dst = 0.0f;
            S.acc_v = S.acc_i = S.acc_n = 0.0f;
            if (++S.r_step < 3) break;

            float d01 = S.ri[1] - S.ri[0], d12 = S.ri[2] - S.ri[1];
            if (!(d01 > 0.1f * S.r_ihi && d12 > 0.1f * S.r_ihi)) { det_finish(-2); return; }
            float r_lo = (S.rv[1] - S.rv[0]) / d01;
            float r_hi = (S.rv[2] - S.rv[1]) / d12;
            float r = (S.rv[2] - S.rv[0]) / (S.ri[2] - S.ri[0]);
            if (r_lo > 1.3f * r_hi && r_hi > 0.0f) { r = r_hi; S.det.warn = 2; }
            if (!(r > 1e-5f && r < 20.0f)) { det_finish(-2); return; }
            S.det.r = r;
            S.det.r_v_high = S.rv[2] - S.rv[0];            /* chute ohmique utile */
            S.det.r_v_dt = S.rv[2] - r * S.ri[2];          /* tension perdue (temps mort, diodes) */
            if (S.det.warn == 0 && fm_abs(S.det.r_v_high) < 0.2f) S.det.warn = 1;
            S.det.r_known = r;

            /* Inductances au courant du palier haut (rotor tenu par le courant d) */
            S.vd_bias = S.rv[2];
            S.inj_v = fm_max(0.02f * S.vbus_f, 0.2f);
            S.inj_acc = S.inj_n = 0.0f;
            S.inj_s0 = S.inj_s1 = S.inj_s2 = 0;
            S.inj_phase = 0;
            S.i_prev = S.id;
            S.ds = DS_L_D;
        }
        break;
    }

    /* --- Inductances : creneaux de tension ++-- sur l'axe d puis q, mesure de di/dt */
    case DS_L_D:
    case DS_L_Q: {
        float i_now = (S.ds == DS_L_D) ? S.id : S.iq;
        /* Intervalle precedent couvert par les deux derniers creneaux ecrits */
        if (S.inj_s1 != 0 && S.inj_s1 == S.inj_s2) {
            S.inj_acc += (i_now - S.i_prev) * (float)S.inj_s1;
            S.inj_n += 1.0f;
        }
        S.i_prev = i_now;
        int8_t sgn = ((S.inj_phase >> 1) & 1) ? -1 : 1;
        S.inj_phase++;
        S.inj_s2 = S.inj_s1;
        S.inj_s1 = sgn;
        /* Tension appliquee directement (sans PI) */
        if (S.ds == DS_L_D) { S.vd = S.vd_bias + sgn * S.inj_v; S.vq = 0.0f; }
        else                { S.vd = S.vd_bias; S.vq = sgn * S.inj_v; }
        S.vd = fm_sat_sym(S.vd, vmax);
        S.vq = fm_sat_sym(S.vq, vmax);

        if (S.dst >= 0.08f && S.inj_n > 10.0f) {
            float di = S.inj_acc / S.inj_n;
            float target = 0.1f * S.r_ihi;
            if (di < 0.3f * target && S.inj_v < 0.25f * S.vbus_f) {
                S.inj_v *= 1.6f;                   /* signal trop faible : on augmente */
                S.inj_acc = S.inj_n = 0.0f;
                S.dst = 0.0f;
            } else if (S.dst >= 0.5f) {
                float l = di > 1e-3f ? S.inj_v * dt / di : -1.0f;
                if (!(l > 1e-7f && l < 0.1f)) { det_finish(-3); return; }
                if (S.ds == DS_L_D) {
                    S.det.ld = l;
                    S.ds = DS_L_Q;
                    S.inj_acc = S.inj_n = 0.0f;
                    S.inj_s1 = S.inj_s2 = 0;
                    S.i_prev = S.iq;
                    S.dst = 0.0f;
                } else {
                    S.det.lq = l;
                    /* fin : le PI reprend au palier haut et ramene le courant a zero */
                    S.ds = DS_END;
                    S.dst = 0.0f;
                    S.int_d = S.vd_bias;
                    S.int_q = 0.0f;
                }
            }
        }
        return;   /* tension imposee : pas de regulation de courant */
    }

    /* --- Flux : rotation en boucle ouverte a courant impose, FEM en regime etabli */
    case DS_LAM_ALIGN:
        S.iq_ref = I * fm_min(S.dst / 0.3f, 1.0f);
        S.id_ref = 0.0f;
        if (S.dst >= 0.5f) { S.ds = DS_LAM_RAMP; S.dst = 0.0f; }
        break;
    case DS_LAM_RAMP:
        S.iq_ref = I;
        S.ol_omega += S.det.ramp_rate * dt;
        {
            float mod = fm_sqrt(S.vd * S.vd + S.vq * S.vq) / fm_max(S.vbus_f * FM_1_SQRT3, 1.0f);
            if (mod >= S.det.duty_target || S.ol_omega >= S.det.omega_test) S.det.omega_test = S.ol_omega;
        }
        if (S.ol_omega >= S.det.omega_test) {
            S.ol_omega = S.det.omega_test;
            S.ds = DS_LAM_HOLD;
            S.dst = 0.0f;
            S.lam_ed = S.lam_eq = S.lam_w = 0.0f;
            S.acc_n = 0.0f;
        }
        break;
    case DS_LAM_HOLD: {
        S.iq_ref = I;
        if (S.dst > 0.4f) {
            float w = S.ol_omega, r = S.det.r_known, l = S.det.l_known;
            S.lam_ed += S.vd - r * S.id + w * l * S.iq;
            S.lam_eq += S.vq - r * S.iq - w * l * S.id;
            S.lam_w += w;
            S.acc_n += 1.0f;
        }
        if (S.dst >= 1.4f) {
            float ed = S.lam_ed / S.acc_n, eq = S.lam_eq / S.acc_n, w = S.lam_w / S.acc_n;
            float lam = fm_sqrt(ed * ed + eq * eq) / fm_max(fm_abs(w), 1.0f);
            if (!(lam > 1e-5f && lam < 1.0f)) { det_finish(-4); return; }
            S.lam_driven = lam;
            S.det.lambda = lam;
            S.ds = DS_LAM_COAST;
            S.dst = 0.0f;
            S.lam_vacc = 0.0f;
            S.acc_n = 0.0f;
            S.lam_w = S.ol_omega;
        }
        break;
    }

    /* --- FEM a pont coupe (sans erreur de temps mort) sur les tensions de phase */
    case DS_LAM_COAST: {
        S.br = false;
        S.int_d = S.int_q = 0.0f;
        if (S.dst > 0.004f) {
            float va = (2.0f * vph[0] - vph[1] - vph[2]) * (1.0f / 3.0f);
            float vb = (vph[1] - vph[2]) * FM_1_SQRT3;
            S.lam_vacc += fm_sqrt(va * va + vb * vb);
            S.acc_n += 1.0f;
        }
        if (S.dst >= 0.03f) {
            float lam_u = S.acc_n > 0.0f ? (S.lam_vacc / S.acc_n) / fm_max(fm_abs(S.lam_w), 1.0f) : 0.0f;
            /* Les deux mesures doivent concorder, sinon le rotor n'a pas suivi la boucle ouverte */
            if (!(lam_u > 0.65f * S.lam_driven && lam_u < 1.35f * S.lam_driven)) { det_finish(-5); return; }
            S.det.lambda = lam_u;
            det_finish(0);
        }
        return;
    }

    /* --- Offsets : pont coupe (courants, tensions de phase), puis vecteur nul sous PWM */
    case DS_OFF_OFF:
        S.br = false;
        S.int_d = S.int_q = 0.0f;
        if (S.dst > 0.05f) {
            for (int x = 0; x < 3; x++) { S.acc3[x] += (float)S.raw_cs[x]; S.accv[x] += S.vph_raw[x]; }
            S.acc_n += 1.0f;
        }
        if (S.dst >= 0.35f) {
            /* pont coupe, moteur arrete : les phases sont reliees par les enroulements et
             * sont au meme potentiel ; l'ecart de chaque phase a la moyenne est son offset
             * (le mode commun n'intervient pas dans alpha/beta) */
            float vm = 0.0f;
            for (int x = 0; x < 3; x++) { S.off_tmp[x] = S.acc3[x] / S.acc_n; vm += S.accv[x] / S.acc_n; }
            vm *= (1.0f / 3.0f);
            for (int x = 0; x < 3; x++) S.det.off_vph[x] = S.accv[x] / S.acc_n - vm;
            for (int x = 0; x < 3; x++) S.acc3[x] = S.accv[x] = 0.0f;
            S.acc_n = 0.0f;
            S.ds = DS_OFF_ON;
            S.dst = 0.0f;
        }
        return;
    case DS_OFF_ON:
        S.br = true;
        S.vd = S.vq = 0.0f;
        S.int_d = S.int_q = 0.0f;
        if (S.dst > 0.05f) {
            for (int x = 0; x < 3; x++) { S.acc3[x] += (float)S.raw_cs[x]; S.accv[x] += S.vph_raw[x]; }
            S.acc_n += 1.0f;
        }
        if (S.dst >= 0.35f) {
            /* sous PWM, vecteur nul : les trois phases commutent ensemble et ont la meme
             * tension moyenne ; l'ecart de chaque phase a la moyenne est son offset */
            float vm = 0.0f;
            for (int x = 0; x < 3; x++) vm += S.accv[x] / S.acc_n;
            vm *= (1.0f / 3.0f);
            for (int x = 0; x < 3; x++) S.det.off_vph_drv[x] = S.accv[x] / S.acc_n - vm;
            for (int x = 0; x < 3; x++) {
                float on = S.acc3[x] / S.acc_n;
                if (!(S.p.cs_two_shunts && x == 2) && fm_abs(on - S.off_tmp[x]) > 80.0f) { det_finish(-6); return; }
                S.det.off_cs[x] = on;
            }
            for (int x = 0; x < 3; x++) { S.off[x] = S.det.off_cs[x]; g_foc_telem.off_cs[x] = S.off[x]; }
            det_finish(0);
        }
        return;

    /* --- Capteurs : rotation lente a courant d impose, 3 tours avant, 3 arriere, 3 avant */
    case DS_SEN_ALIGN:
        S.id_ref = I * fm_min(S.dst / 0.3f, 1.0f);
        S.iq_ref = 0.0f;
        S.ol_theta = 0.0f;
        S.ol_omega = 0.0f;
        if (S.dst >= 0.6f) {
            S.ds = DS_SEN_RUN;
            S.dst = 0.0f;
            S.sen_phase = 0;
            S.sen_rot = 0.0f;
            S.enc_prev = S.enc_now;
            S.enc_acc = 0;
            S.off_sin = S.off_cos = 0.0f;
            for (int k = 0; k < 8; k++) S.hs[k] = S.hc[k] = S.hn[k] = 0.0f;
            S.ecfg = S.p.enc;
        }
        break;
    case DS_SEN_RUN: {
        float target = S.det.omega_test * (S.sen_phase == 1 ? -1.0f : 1.0f);
        S.ol_omega += fm_sat_sym(target - S.ol_omega, S.det.omega_test / 0.3f * dt);
        S.id_ref = I;
        S.iq_ref = 0.0f;
        S.sen_rot += fm_abs(S.ol_omega) * dt;
        if (S.p.enc.cpr >= 4) {
            int32_t dc = (int32_t)(S.enc_now - S.enc_prev);
            int32_t cpr = (int32_t)S.p.enc.cpr;
            while (dc > cpr / 2) dc -= cpr;
            while (dc < -(cpr / 2)) dc += cpr;
            S.enc_acc += dc;
        }
        S.enc_prev = S.enc_now;
        if (fm_abs(S.ol_omega) > 0.9f * S.det.omega_test) {
            float sn, cs;
            if (S.det.kind == FOC_DET_HALL) {
                uint8_t h = S.hall_now & 7;
                fm_sincos(S.ol_theta, &sn, &cs);
                S.hs[h] += sn;
                S.hc[h] += cs;
                S.hn[h] += 1.0f;
            } else if (S.sen_phase > 0) {
                fm_sincos(fm_wrap_pi(enc_raw_angle(&S.ecfg, S.enc_now) - S.ol_theta), &sn, &cs);
                S.off_sin += sn;
                S.off_cos += cs;
            }
        }
        if (S.sen_rot >= 3.0f * FM_2PI) {
            if (S.det.kind == FOC_DET_ENCODER && S.sen_phase == 0) {
                float cpr = (float)S.p.enc.cpr;
                float mech = fm_abs((float)S.enc_acc) / cpr;
                if (mech < 3.0f / 40.0f || mech > 3.1f) { det_finish(-8); return; }
                float ratio = 3.0f / mech;
                float rr = (float)(int32_t)(ratio + 0.5f);
                if (fm_abs(ratio - rr) < 0.05f * rr) ratio = rr;
                S.ecfg.ratio = ratio;
                S.ecfg.inverted = S.enc_acc < 0;
                S.ecfg.offset = 0.0f;
                S.det.enc_ratio = ratio;
                S.det.enc_inverted = S.ecfg.inverted;
            }
            S.sen_phase++;
            S.sen_rot = 0.0f;
            if (S.sen_phase >= 3) {
                if (S.det.kind == FOC_DET_HALL) {
                    float tot = 0.0f;
                    for (int k = 0; k < 8; k++) tot += S.hn[k];
                    if (S.hn[0] > 0.005f * tot || S.hn[7] > 0.005f * tot) { det_finish(-7); return; }
                    for (int k = 1; k < 7; k++) {
                        if (S.hn[k] < 0.03f * tot) { det_finish(-7); return; }
                        S.det.hall_angle[k] = fm_atan2(S.hs[k], S.hc[k]);
                        S.det.hall_valid[k] = true;
                    }
                } else {
                    S.det.enc_offset = fm_atan2(S.off_sin, S.off_cos);
                }
                S.ds = DS_END;
                S.dst = 0.0f;
            }
        }
        break;
    }

    case DS_END:
        /* Retour du courant a zero avant de couper le pont */
        S.id_ref *= 0.995f;
        S.iq_ref *= 0.995f;
        if (S.det.kind != FOC_DET_RL) S.ol_omega *= 0.999f;
        if (S.dst >= 0.3f) { det_finish(0); return; }
        break;
    }

    /* Rotation boucle ouverte eventuelle */
    S.ol_theta = fm_wrap_pi(S.ol_theta + S.ol_omega * dt);
    current_loop(kp, ki, 0.0f, 0.0f, vmax);
}

/* ================================================================ ISR */
RAMFUNC void foc_isr(const foc_adc_t *adc, foc_pwm_t *out)
{
    if (S.p_pending) {
        apply_params(&S.p_next);
        S.p_pending = false;
    }
    const float dt = S.dt;
    g_foc_telem.ticks++;
    S.st_time += dt;

    /* ---- 1. Conversion des mesures */
    S.hall_now = adc->hall;
    S.enc_now = adc->enc;
    g_foc_telem.hall = adc->hall;
    uint16_t cs[3] = { adc->cs[0], adc->cs[1], adc->cs[2] };
    uint16_t vr[3] = { adc->vph[0], adc->vph[1], adc->vph[2] };
    if (S.p.invert_dir) {
        uint16_t t = cs[1]; cs[1] = cs[2]; cs[2] = t;
        t = vr[1]; vr[1] = vr[2]; vr[2] = t;
    }
    S.vbus = (float)adc->vbus * S.p.v_scale;
    if (g_foc_telem.ticks < 4) S.vbus_f = S.vbus;
    FM_LPF(S.vbus_f, S.vbus, 0.2f);
    float vph[3];
    for (int x = 0; x < 3; x++) {
        S.raw_cs[x] = cs[x];
        S.vph_raw[x] = (float)vr[x] * S.p.vph_scale;
        vph[x] = S.vph_raw[x] - S.p.off_vph[x];
    }

    /* ---- 2. Calibration des offsets (pont coupe) */
    if (S.st == FOC_ST_INIT || S.st == FOC_ST_CALIB) {
        bridge_off();
        if (S.st_time > 0.05f) {
            for (int x = 0; x < 3; x++) S.cal_acc[x] += (float)cs[x];
            S.cal_n++;
            if (S.cal_n >= 2048) {
                bool ok = true;
                for (int x = 0; x < 3; x++) {
                    S.off[x] = S.cal_acc[x] / (float)S.cal_n;
                    g_foc_telem.off_cs[x] = S.off[x];
                    if (S.p.cs_two_shunts && x == 2) continue;
                    if (fm_abs(S.off[x] - 2048.0f) > 600.0f) {
                        ok = false;
                        trip(RESC_FAULT_HIGH_OFFSET_CURR_1 + (uint32_t)x);
                    }
                }
                if (ok && S.p.use_stored_offsets) {
                    /* offsets de la calibration manuelle (mesures sous PWM), si coherents */
                    for (int x = 0; x < 3; x++) {
                        if (fm_abs(S.p.stored_off_cs[x] - S.off[x]) < 80.0f) S.off[x] = S.p.stored_off_cs[x];
                        g_foc_telem.off_cs[x] = S.off[x];
                    }
                }
                if (ok) {
                    S.calibrated = true;
                    set_state(FOC_ST_IDLE);
                }
            }
        }
        goto output;
    }

    for (int x = 0; x < 3; x++) S.i_ph[x] = ((float)cs[x] - S.off[x]) * S.p.i_scale;

    /* ---- 3. Validite des echantillons (fenetre low-side de la periode echantillonnee) */
    bool valid[3] = { true, true, true };
    if (S.p.cs_lowside && S.br_p1) {
        float wmin = S.p.t_win * S.p.f_pwm;
        for (int x = 0; x < 3; x++) valid[x] = (1.0f - S.duty_p1[x]) >= wmin;
    }
    if (S.p.cs_two_shunts) valid[2] = false;

    /* ---- 4. Protections rapides */
    for (int x = 0; x < 3; x++) {
        if (valid[x] && fm_abs(S.i_ph[x]) > S.p.i_abs_trip) {
            trip(RESC_FAULT_ABS_OVER_CURRENT);
            break;
        }
    }
    if (S.vbus > S.p.vbus_trip_max) trip(RESC_FAULT_OVER_VOLTAGE);
    if (S.br && S.vbus < S.p.vbus_trip_min) trip(RESC_FAULT_UNDER_VOLTAGE);

    reconstruct_currents(valid);
    {
        float imag = fm_sqrt(S.i_a * S.i_a + S.i_b * S.i_b);
        if (imag > S.p.i_abs_trip) trip(RESC_FAULT_ABS_OVER_CURRENT);
        if (imag > g_foc_telem.i_phase_max) g_foc_telem.i_phase_max = imag;
    }

    /* ---- 5. Observateur (toujours actif : suit aussi un moteur en roue libre) */
    {
        float va, vb;
        applied_voltage(vph, &va, &vb);
#ifdef RESC_HOST
        g_foc_telem.dbg_va = va; g_foc_telem.dbg_vb = vb; g_foc_telem.dbg_ia = S.i_a; g_foc_telem.dbg_ib = S.i_b;
#endif
        obs_update(&S.obs, &S.oc, va, vb, S.i_a, S.i_b, S.id, dt);
    }
    float flux_ratio = S.obs.flux / S.oc.lambda;

    /* ---- 5b. Capteurs d'angle */
    bool sensor_ok = false;
    if (S.p.sensor_mode == FOC_SENSOR_HALL) {
        hall_update(&S.hall, &S.p.hall, S.hall_now, dt);
        sensor_ok = S.hall.ok;
    } else if (S.p.sensor_mode == FOC_SENSOR_ENCODER && S.p.enc.cpr >= 4) {
        enc_update(&S.enc, &S.p.enc, S.enc_now, dt);
        sensor_ok = true;
    }

    /* ---- 6. Machine d'etats */
    float vmax = S.p.m_max * S.vbus_f * FM_1_SQRT3;
    float theta_out;

    switch (S.st) {
    case FOC_ST_FAULT:
        bridge_off();
        if (S.clear_req) {
            S.clear_req = false;
            S.fault = RESC_FAULT_NONE;
            g_foc_telem.fault = RESC_FAULT_NONE;
            set_state(FOC_ST_IDLE);
        }
        S.theta = S.obs.theta;
        S.omega = S.obs.omega;
        goto output;

    case FOC_ST_IDLE:
        bridge_off();
        S.theta = S.obs.theta;
        S.omega = S.obs.omega;
        if (S.det.busy) {
            set_state(FOC_ST_DETECT);
            S.ds = DS_IDLE;
            goto output;
        }
        if (g_foc_cmd.run && g_foc_cmd.openloop && S.vbus > S.p.vbus_trip_min) {
            enter_forced();
        } else if (g_foc_cmd.run && motor_params_ok() && S.vbus > S.p.vbus_trip_min) {
            S.dir = g_foc_cmd.dir >= 0 ? 1 : -1;
            bool obs_fast = fm_abs(S.obs.omega) > 0.8f * S.p.st_omega && flux_ratio > 0.7f && flux_ratio < 1.4f;
            if (sensor_ok && !(S.p.sensor_mode == FOC_SENSOR_HALL && obs_fast && fm_abs(S.obs.omega) > S.p.sl_omega)) {
                /* Capteur : marche directe depuis l'arret, sans boucle ouverte */
                S.id_ref = S.iq_ref = 0.0f;
                S.int_d = S.int_q = 0.0f;
                S.slew = S.p.i_slew;
                S.slew_t = 0.0f;
                S.low_t = S.flux_bad_t = S.hall_bad_t = 0.0f;
                S.use_obs = false;
                set_state(FOC_ST_RUN);
            } else if (obs_fast) {
                /* Reprise au vol : l'anticipation porte la FEM, integrateurs nuls */
                S.id_ref = S.iq_ref = 0.0f;
                S.int_d = S.int_q = 0.0f;
                S.slew = S.p.i_slew;
                S.slew_t = 0.0f;
                S.low_t = S.flux_bad_t = 0.0f;
                S.use_obs = true;
                set_state(FOC_ST_RUN);
            } else if (g_foc_cmd.allow_start && S.p.sensor_mode == FOC_SENSOR_SENSORLESS) {
                enter_start(false);
            }
        }
        if (S.st == FOC_ST_IDLE) goto output;
        break;

    default:
        break;
    }

    if (S.st == FOC_ST_DETECT) {
        S.theta = S.ol_theta;
        S.omega = S.ol_omega;
        float s, c;
        fm_sincos(S.theta, &s, &c);
        S.id = c * S.i_a + s * S.i_b;
        S.iq = -s * S.i_a + c * S.i_b;
        if (!S.det.busy) { bridge_off(); set_state(FOC_ST_IDLE); goto output; }
        S.br = true;
        det_step(vmax, vph);
        if (S.st != FOC_ST_DETECT) goto output;
        theta_out = S.theta + S.omega * S.p.delay_comp * dt;
        modulate(theta_out);
        goto output;
    }

    if (!g_foc_cmd.run) {
        bridge_off();
        set_state(FOC_ST_IDLE);
        goto output;
    }
    /* Passage en boucle ouverte commandee depuis le demarrage ou la marche */
    if (g_foc_cmd.openloop && !(S.st == FOC_ST_START && S.sub == ST_FORCED)) enter_forced();
    /* Fin de la boucle ouverte commandee : pont coupe, la commande suivante redemarre */
    if (!g_foc_cmd.openloop && S.st == FOC_ST_START && S.sub == ST_FORCED) {
        bridge_off();
        set_state(FOC_ST_IDLE);
        goto output;
    }

    if (S.st == FOC_ST_START && S.sub == ST_FORCED) {
        S.br = true;
        float target = g_foc_cmd.ol_omega;
        float acc = fm_max(S.p.st_omega / fm_max(S.p.st_t_ramp, 0.05f), 200.0f);
        S.ol_omega += fm_sat_sym(target - S.ol_omega, acc * dt);
        S.ol_theta = fm_wrap_pi(S.ol_theta + S.ol_omega * dt);
        S.theta = S.ol_theta;
        S.omega = S.ol_omega;
        float s, c;
        fm_sincos(S.theta, &s, &c);
        S.id = c * S.i_a + s * S.i_b;
        S.iq = -s * S.i_a + c * S.i_b;
        if (g_foc_cmd.openloop == 2) {
            /* rapport cyclique impose sur l'axe q, sans regulation de courant
             * (les protections de courant de l'ISR restent actives) */
            float v = fm_sat_sym(g_foc_cmd.ol_duty, S.p.m_max) * S.vbus_f * FM_1_SQRT3;
            S.vd = 0.0f;
            S.vq += fm_sat_sym(v - S.vq, vmax * dt / 0.3f);
            S.int_d = S.int_q = 0.0f;
            S.id_ref = 0.0f;
            S.iq_ref = 0.0f;
        } else {
            S.ol_iq += fm_sat_sym(g_foc_cmd.iq_ref - S.ol_iq, S.p.i_slew * dt);
            S.id_ref = 0.0f;
            S.iq_ref = S.ol_iq;
            current_loop(S.p.kp, S.p.ki, 0.0f, 0.0f, vmax);
        }
        theta_out = S.theta + S.omega * S.p.delay_comp * dt;
        modulate(theta_out);
        goto output;
    }

    if (S.st == FOC_ST_START) {
        S.br = true;
        if (S.sub == ST_ALIGN) {
            S.ol_iq = (float)S.dir * S.p.st_i * fm_min(S.st_time / fm_max(S.p.st_t_align * 0.5f, 1e-3f), 1.0f);
            if (S.st_time >= S.p.st_t_align) S.sub = ST_RAMP;
        } else {
            float acc = S.p.st_omega / fm_max(S.p.st_t_ramp, 0.05f);
            float target = (float)S.dir * S.p.st_omega;
            if (S.ol_omega < target) S.ol_omega = fm_min(S.ol_omega + acc * dt, target);
            else S.ol_omega = fm_max(S.ol_omega - acc * dt, target);
            S.ol_iq = (float)S.dir * fm_max(S.p.st_i, fm_abs(g_foc_cmd.iq_ref));
            /* Relais vers l'observateur quand il est accroche */
            float w = S.ol_omega;
            bool fast = fm_abs(w) >= 0.9f * S.p.st_omega;
            bool lock = flux_ratio > 0.75f && flux_ratio < 1.3f &&
                        fm_abs(S.obs.omega - w) < 0.25f * fm_abs(w) &&
                        S.obs.omega * w > 0.0f;
            S.lock_t = (fast && lock) ? S.lock_t + dt : 0.0f;
        }
        S.ol_theta = fm_wrap_pi(S.ol_theta + S.ol_omega * dt);
        S.theta = S.ol_theta;
        S.omega = S.ol_omega;
        S.id_ref = 0.0f;
        S.iq_ref = S.ol_iq;
        if (S.lock_t >= 0.05f) {
            /* calcul des courants dans le repere boucle ouverte, puis relais */
            float s, c;
            fm_sincos(S.theta, &s, &c);
            S.id = c * S.i_a + s * S.i_b;
            S.iq = -s * S.i_a + c * S.i_b;
            handoff_to_run();
        } else if (S.st_time > S.p.st_timeout) {
            trip(RESC_FAULT_UNDERSPEED);
            goto output;
        }
    }

    float vd_ff = 0.0f, vq_ff = 0.0f;
    if (S.st == FOC_ST_RUN) {
        /* Source d'angle : capteur a basse vitesse, observateur au-dessus (Hall) */
        if (S.p.sensor_mode == FOC_SENSOR_HALL) {
            float w = fm_abs(S.obs.omega);
            bool obs_ok = flux_ratio > 0.7f && flux_ratio < 1.4f;
            if (!S.use_obs && w > 1.1f * S.p.sl_omega && obs_ok) S.use_obs = true;
            else if (S.use_obs && w < 0.9f * S.p.sl_omega) S.use_obs = false;
            if (!sensor_ok) {
                S.hall_bad_t += dt;
                if (!S.use_obs && S.hall_bad_t > 0.05f) { trip(RESC_FAULT_UNDERSPEED); goto output; }
            } else {
                S.hall_bad_t = 0.0f;
            }
        } else {
            S.use_obs = S.p.sensor_mode == FOC_SENSOR_SENSORLESS;
        }
        if (S.use_obs) {
            S.theta = S.obs.theta;
            S.omega = S.obs.omega;
        } else if (S.p.sensor_mode == FOC_SENSOR_HALL) {
            S.theta = S.hall.theta;
            S.omega = (fm_abs(S.obs.omega) > fm_abs(S.hall.omega) && flux_ratio > 0.7f && flux_ratio < 1.4f) ? S.obs.omega : S.hall.omega;
        } else {
            S.theta = S.enc.theta;
            S.omega = S.enc.omega;
        }
        g_foc_telem.angle_from_sensor = !S.use_obs;

        /* references internes : fluage vers la consigne */
        if (S.slew_t > 0.0f) S.slew_t -= dt; else S.slew = S.p.i_slew;
        float step = S.slew * dt;
        S.id_ref += fm_sat_sym(g_foc_cmd.id_ref - S.id_ref, step);
        S.iq_ref += fm_sat_sym(g_foc_cmd.iq_ref - S.iq_ref, step);

        vd_ff = -S.omega * S.p.lq * S.iq_ref;
        vq_ff = S.omega * (S.p.ld * S.id_ref + S.p.lambda);

        /* Basse vitesse : retour en I/f si autorise, sinon roue libre */
        if (S.p.sensor_mode == FOC_SENSOR_SENSORLESS && fm_abs(S.omega) < S.p.st_omega_exit) S.low_t += dt; else S.low_t = 0.0f;
        if (S.low_t > 0.05f) {
            if (g_foc_cmd.allow_start) {
                S.dir = g_foc_cmd.dir >= 0 ? 1 : -1;
                enter_start(true);
                S.theta = S.ol_theta;
                S.omega = S.ol_omega;
                vd_ff = vq_ff = 0.0f;
            } else {
                bridge_off();
                set_state(FOC_ST_IDLE);
                goto output;
            }
        }
        /* Perte de synchronisation : flux estime incoherent */
        if (S.st == FOC_ST_RUN && S.use_obs) {
            if (flux_ratio < 0.5f || flux_ratio > 2.0f) S.flux_bad_t += dt; else S.flux_bad_t = 0.0f;
            if (S.flux_bad_t > 0.02f) {
                trip(RESC_FAULT_UNDERSPEED);
                goto output;
            }
        }
        S.br = true;
    }

    /* ---- 7. Park, boucle de courant, modulation */
    {
        float s, c;
        fm_sincos(S.theta, &s, &c);
        S.id = c * S.i_a + s * S.i_b;
        S.iq = -s * S.i_a + c * S.i_b;
    }
    if (S.st == FOC_ST_RUN && g_foc_cmd.duty_mode) {
        duty_loop(S.p.kp, S.p.ki, vmax);
    } else {
        if (S.duty_on) {
            /* sortie du mode duty : l'integrateur q reprend la tension appliquee */
            S.duty_on = false;
            S.int_q = S.vq - vq_ff;
        }
        current_loop(S.p.kp, S.p.ki, vd_ff, vq_ff, vmax);
    }
    theta_out = S.theta + S.omega * S.p.delay_comp * dt;
    modulate(theta_out);

output:
    if (!S.br) {
        S.duty[0] = S.duty[1] = S.duty[2] = 0.5f;
        if (S.st != FOC_ST_DETECT && S.st != FOC_ST_START && S.st != FOC_ST_RUN) {
            /* Hors pont actif, Park pour la telemetrie */
            float s, c;
            fm_sincos(S.obs.theta, &s, &c);
            S.id = c * S.i_a + s * S.i_b;
            S.iq = -s * S.i_a + c * S.i_b;
        }
    }

    /* ---- 8. Sorties et historique */
    for (int x = 0; x < 3; x++) {
        S.duty_p2[x] = S.duty_p1[x];
        S.duty_p1[x] = S.duty[x];
    }
    S.br_p2 = S.br_p1;
    S.br_p1 = S.br;

    out->enable = S.br;
    if (S.p.invert_dir) {
        out->duty[0] = S.duty[0];
        out->duty[1] = S.duty[2];
        out->duty[2] = S.duty[1];
    } else {
        out->duty[0] = S.duty[0];
        out->duty[1] = S.duty[1];
        out->duty[2] = S.duty[2];
    }

    /* ---- 9. Telemetrie (filtree) */
    {
        foc_telem_t *t = &g_foc_telem;
        const float k = 0.02f;
        FM_LPF(t->id, S.id, k);
        FM_LPF(t->iq, S.iq, k);
        FM_LPF(t->vd, S.vd, k);
        FM_LPF(t->vq, S.vq, k);
        float imag = fm_sqrt(S.id * S.id + S.iq * S.iq);
        FM_LPF(t->i_abs, imag, k);
        float p = 1.5f * (S.vd * S.id + S.vq * S.iq);
        FM_LPF(t->i_bus, S.br ? p / fm_max(S.vbus_f, 1.0f) : 0.0f, k);
        t->id_ref = S.id_ref;
        t->iq_ref = S.iq_ref;
        t->vbus = S.vbus_f;
        float vmag = fm_sqrt(S.vd * S.vd + S.vq * S.vq);
        float duty = S.br ? vmag / fm_max(S.vbus_f * FM_1_SQRT3, 1.0f) : 0.0f;
        t->duty = S.vq < 0.0f ? -duty : duty;
        /* vitesse et angle de COMMANDE (observateur, Hall, codeur ou boucle ouverte) */
        FM_LPF(t->omega, S.omega, 0.05f);
        t->theta = S.theta;
        t->obs_omega = S.obs.omega;
        t->obs_theta = S.obs.theta;
        t->flux_ratio = S.obs.flux / S.oc.lambda;
        t->state = S.st;
    }
}
