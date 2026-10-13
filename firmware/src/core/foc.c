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
#include <math.h>
#include <string.h>
#include "foc.h"
#include "observer.h"
#include "fmath.h"
#include "fault_codes.h"
#include "platform.h"

/* ------------------------------------------------------------------ etat */
typedef enum { ST_ALIGN = 0, ST_RAMP, ST_FORCED, ST_HFI, ST_MAG } start_sub_t;   /* ST_MAG : magnetisation (induction) */   /* ST_FORCED : boucle ouverte commandee */

typedef enum {
    DS_IDLE = 0,
    DS_R_SEARCH, DS_R_STEP,
    DS_L_D, DS_L_Q,
    DS_LAM_ALIGN, DS_LAM_RAMP, DS_LAM_HOLD, DS_LAM_COAST,
    DS_OFF_OFF, DS_OFF_ON,
    DS_SEN_ALIGN, DS_SEN_RUN,
    DS_AC_PRE, DS_AC_STEP,
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
    float cal_vacc[3], cal_vmin[3], cal_vmax[3];   /* tensions de phase pont coupe (calibration au demarrage) */
    float cal_dacc[3], cal_dsq[3];                 /* ecart a la moyenne des 3 phases : somme, somme des carres */
    float off_vph[3];            /* offsets de tension pont coupe utilises (demarrage ou configuration) */
    uint32_t meas_n;             /* echantillons pont coupe depuis le dernier pilotage */
    float va_prev, vb_prev;      /* pont coupe : tension mesuree a l'echantillon precedent */
    /* validation de la reprise au vol : la FEM mesuree (pont coupe) doit etre en avance de 90 deg sur le flux
     * observe ; l'observateur integre cette meme tension, un ecart signale qu'il ne la suit pas (derive) */
    float rv_s, rv_c;            /* ecart FEM mesuree / FEM attendue (sin, cos filtres) */
    float rv_mag;                /* V : module de la tension mesuree, filtre */
    float rv_e;                  /* rad : ecart instantane (le filtre seul laisserait passer un saut brusque) */
    float rv_th_prev, rv_w;      /* angle de la tension mesuree, vitesse qui s'en deduit (rad/s el.) */
    float rv_flux;               /* angle du flux deduit de la tension mesuree (recalage) */
    float resume_wait_t;         /* s : reprise demandee, refusee faute d'observateur coherent */
    /* perte de synchronisation : courant mesure compare a la prediction du modele d/q sur une periode */
    float pr_id, pr_iq;          /* courant d/q de la periode precedente */
    float pr_vd1, pr_vq1, pr_vd2, pr_vq2;   /* tensions commandees aux deux periodes precedentes */
    uint16_t pr_n;               /* periodes de marche consecutives enregistrees */
    bool pr_meas, pr_meas_prev;  /* courant mesure (et non predit) a cette periode, a la precedente */
    float sync_r;                /* residu filtre (1 = seuil) */
    uint8_t sync_bad;
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
    bool  dr_on;                 /* duty : regulateur de descente actif */
    bool  dc_on;                 /* duty simple : roue libre (courant nul) jusqu'a la consigne */
    float dr_int;
    float duty_abs;              /* modulation appliquee (0..1, Vbus/sqrt3 = 1) */
    float i_abs_f;               /* module du courant filtre (frein) */
    float slip_t;                /* duree d'ecart codeur / observateur */
    float fw_id;                 /* defluxage */
    float iq_lpf;                /* iq mesure filtre (MTPA) */
    float lam_eff;               /* lambda courant (adaptatif, temperature, saturation) */
    volatile float temp_mot;
    float w_pll_theta, w_pll_omega;   /* vitesse derivee de l'angle de commande */
    float omega_ctl;
    float pf_err, pf_bad_t, pf_t;   /* filtres de phase */
    bool  pf_on, pf_use;
    float hall_mix;              /* melange Hall -> observateur (0..1) */

    /* moteur a induction */
    float psir;                  /* flux rotorique (modele en courant, Wb) */
    float ws;                    /* glissement (rad/s el.) */
    float slip_acc;              /* angle de glissement accumule (codeur) */
    float ac_t_mag;              /* s : magnetisation avant couple */
    float ac_vinf, ac_i0;        /* identification : tension finale, courant du palier bas */
    uint16_t ac_n;

    /* injection haute frequence */
    bool  hfi_on;                /* injection active */
    bool  use_hfi;               /* angle de commande = HFI (marche) */
    uint8_t hfi_ph;              /* sequence ++-- */
    int8_t hfi_s1, hfi_s2;       /* signes des deux derniers creneaux ecrits */
    float hfi_ia, hfi_ib;        /* courant de l'echantillon precedent */
    float hfi_q_prev, hfi_d_prev;
    int8_t hfi_sig_prev;
    float hfi_acc, hfi_amp_acc;
    uint16_t hfi_n, hfi_amp_n;
    float hfi_theta, hfi_omega;
    float hfi_v;
    float hfi_amp0, hfi_amp_p, hfi_amp_m;
    uint16_t hfi_upd, hfi_good;
    uint8_t hfi_stage;           /* demarrage : 0 convergence, 1 impulsion +, 2 impulsion -, 3 termine */
    float hfi_t;

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

    /* demi-periode (ctl_mode 2) : rapports cycliques de la seconde moitie */
    float duty_h[3], duty_h_prev[3];
    float theta_h;
    bool  h_ok;
    /* frein : court-circuit des phases tant que le courant de frein n'est pas atteignable */
    bool  brk_short;
    uint8_t brk_n;
    float brk_w_prev, brk_vq_prev, duty_sf;
    float hfi_ovr_t;             /* HFI : angle recale sur l'observateur apres le retour a basse vitesse */
    float id_f, iq_f;            /* courants filtres (foc_current_filter_const) */

    foc_cmd_t cmd;               /* copie de g_foc_cmd prise au debut de chaque periode */
    uint32_t fault;
    volatile bool clear_req;
    volatile uint32_t fault_req;
    uint8_t ov_n, uv_n;          /* echantillons consecutifs hors plage de tension bus */ /* defaut signale hors ISR (pris en compte au debut de la periode suivante) */
} foc_t;

static CCMBSS foc_t S;
#define AC_N   600                 /* identification induction : vd toutes les 2 ms pendant 1.2 s */
static float s_acbuf[AC_N];
volatile foc_cmd_t g_foc_cmd;
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
    S.oc.kind = S.p.obs_type < OBS_COUNT ? (obs_kind_t)S.p.obs_type : OBS_RESC_ACTIVE_FLUX;
    S.oc.hyb_w1 = S.p.st_omega;
    S.oc.hyb_w2 = 3.0f * S.p.st_omega;
    S.oc.gain_slow = fm_sat(S.p.obs_gain_slow, 0.0f, 1.0f);
    S.oc.offset = S.p.obs_offset;
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
    /* valeurs neutres si non renseignees (0) */
    if (!(S.p.overmod >= 1.0f)) S.p.overmod = 1.0f;
    if (S.p.overmod > 1.5f) S.p.overmod = 1.5f;
    if (!(S.p.vd_max > 0.0f && S.p.vd_max <= 1.0f)) S.p.vd_max = 1.0f;
    if (!(S.p.i_filter > 0.0f)) S.p.i_filter = 0.1f;
    S.dt = 1.0f / S.p.f_pwm;
    obs_cfg_from_params();
    if (!(S.obs.lambda > 0.5f * S.oc.lambda && S.obs.lambda < 1.5f * S.oc.lambda)) S.obs.lambda = S.oc.lambda;
    if (!(S.obs.lambda_est > 0.3f * S.oc.lambda && S.obs.lambda_est < 2.5f * S.oc.lambda)) S.obs.lambda_est = S.oc.lambda;
    S.lam_eff = S.obs.lambda;
}

static bool acim(void) { return S.p.motor_kind == 1; }
/* parametres d'induction utilisables (evite 0/0 tant qu'ils ne sont pas saisis) */
static bool acim_ok(void)
{
    return S.p.acim_lm > 1e-7f && S.p.acim_lr > S.p.acim_lm && S.p.acim_rr > 1e-6f && S.p.acim_im > 0.1f;
}
static float acim_tr(void) { return S.p.acim_lr / fm_max(S.p.acim_rr, 1e-6f); }

static bool motor_params_ok(void)
{
    if (acim())
        return S.p.r > 1e-5f && S.p.ld > 1e-8f && S.p.acim_lm > 1e-7f && S.p.acim_lr > S.p.acim_lm &&
               S.p.acim_rr > 1e-5f && S.p.acim_im > 0.1f;
    return S.p.r > 1e-5f && S.p.ld > 1e-8f && S.p.lq > 1e-8f && S.p.lambda > 1e-6f;
}



/* Uniquement depuis l'ISR FOC (les autres contextes passent par foc_raise_fault) */
RAMFUNC static void trip_at(uint32_t code, uint32_t line)
{
    foc_hw_kill();
    S.br = false;
    if (S.fault == RESC_FAULT_NONE) {
        S.fault = code;
        g_foc_telem.fault = code;
        g_foc_telem.last_fault = code;          /* memorise : un defaut qui s'efface seul reste visible */
        g_foc_telem.last_fault_line = line;
        g_foc_telem.fault_count++;
        uint32_t k = g_foc_telem.fault_log_n++ & 7u;
        g_foc_telem.fault_log[k].code = code;
        g_foc_telem.fault_log[k].line = line;
        g_foc_telem.fault_log[k].ticks = g_foc_telem.ticks;
    }
    if (S.det.busy) {
        S.det.error = (int)code;
        S.det.busy = false;
        S.det.done = true;
        S.ds = DS_IDLE;
    }
    set_state(FOC_ST_FAULT);
}

__attribute__((weak)) void foc_hw_rearm(void) { }

/* Defaut detecte hors ISR (SysTick, break, EXTI, watchdog analogique) : le pont est coupe tout de suite,
 * l'etat du coeur n'est modifie que par l'ISR (pas d'ecriture concurrente de S.st / S.br). */
void foc_raise_fault(uint32_t code)
{
    foc_hw_kill();
    uint32_t none = RESC_FAULT_NONE;              /* premier defaut conserve ; LDREX/STREX sur cible */
    if (code != RESC_FAULT_NONE)
        __atomic_compare_exchange_n(&S.fault_req, &none, code, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}
/* Essais (simulation) : observateur force sur un etat donne, comme un decrochage en roue libre */
void foc_dbg_obs_reset(float theta, float omega) { obs_reset(&S.obs, &S.oc, theta, omega); }
#define trip(code) trip_at((code), __LINE__)
void foc_set_motor_temp(float t) { S.temp_mot = t; }

/* Parametres moteur effectifs : temperature (R cuivre +0.393 %/K, lambda NdFeB -0.11 %/K)
 * et saturation (baisse lineaire avec le courant, rapportee a i_sat_ref). */
RAMFUNC static void motor_comp(void)
{
    float kr = 1.0f, kl = 1.0f, ksl = 1.0f;
    if (S.p.temp_comp) {
        float dT = S.temp_mot - S.p.temp_comp_base;
        kr = 1.0f + 0.00393f * dT;
        kl = 1.0f - 0.0011f * dT;
    }
    if (S.p.sat_comp_mode && S.p.i_sat_ref > 0.0f) {
        float x = fm_min(fm_sqrt(S.i_a * S.i_a + S.i_b * S.i_b) / S.p.i_sat_ref, 1.5f);
        float k = fm_max(1.0f - S.p.sat_comp * x, 0.3f);
        if (S.p.sat_comp_mode & 1) ksl = k;
        if (S.p.sat_comp_mode & 2) kl *= k;
    }
    S.oc.r = S.p.r * fm_sat(kr, 0.5f, 1.6f);
    S.oc.ld = S.p.ld * ksl;
    S.oc.lq = S.p.lq * ksl;
    S.oc.lambda = fm_max(S.p.lambda * fm_sat(kl, 0.5f, 1.2f), 1e-6f);
    if (acim() && acim_ok()) {
        /* flux actif d'un moteur asynchrone : (Lm/Lr) psi_r, module donne par le modele en courant */
        float nom = S.p.acim_lm * S.p.acim_lm / S.p.acim_lr * S.p.acim_im;
        S.oc.lambda = fm_max(S.p.acim_lm / S.p.acim_lr * S.psir, 0.05f * nom);
        S.oc.kind = OBS_RESC_ACTIVE_FLUX;     /* flux actif a module variable (modele en courant) */
    }
    float gmax = 0.2f / (S.oc.lambda * S.oc.lambda * S.dt);
    S.oc.gamma = S.p.obs_gamma < gmax ? S.p.obs_gamma : gmax;
}

/* Anticipation de la boucle de courant selon foc_cc_decoupling */
RAMFUNC static void ff_terms(float w, float id, float iq, float *vd, float *vq)
{
    *vd = (S.p.decoupling & 1) ? -w * S.oc.lq * iq : 0.0f;
    *vq = ((S.p.decoupling & 1) ? w * S.oc.ld * id : 0.0f) + ((S.p.decoupling & 2) ? w * S.lam_eff : 0.0f);
}

/* Integrateurs prepositionnes sur la tension du modele en regime etabli pour les
 * references actuelles (reprise sans a-coup), anticipation deduite si utilisee */
RAMFUNC static void pi_preload(float w, bool with_ff)
{
    float vd = S.oc.r * S.id_ref - w * S.oc.lq * S.iq_ref;
    float vq = S.oc.r * S.iq_ref + w * (S.oc.ld * S.id_ref + S.lam_eff);
    float fd = 0.0f, fq = 0.0f;
    if (with_ff) ff_terms(w, S.id_ref, S.iq_ref, &fd, &fq);
    S.int_d = vd - fd;
    S.int_q = vq - fq;
}

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
        S.duty[k] = S.duty_p1[k] = S.duty_p2[k] = S.duty_h[k] = S.duty_h_prev[k] = 0.5f;
        S.off[k] = 2048.0f;
    }
    obs_reset(&S.obs, &S.oc, 0.0f, 0.0f);
    hall_reset(&S.hall);
    enc_reset(&S.enc);
    g_foc_cmd.run = 0;
    g_foc_cmd.dir = 1;
    S.cmd.dir = 1;
    for (int x = 0; x < 3; x++) S.off_vph[x] = S.p.off_vph[x];
    set_state(FOC_ST_CALIB);
}

void foc_cmd_publish(const foc_cmd_t *c)
{
    uint32_t q = g_foc_cmd.seq;
    g_foc_cmd.seq = q + 1u;                       /* impair : l'ISR ignore la commande en cours d'ecriture */
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    foc_cmd_t t = *c;
    t.seq = q + 1u;
    g_foc_cmd = t;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    g_foc_cmd.seq = q + 2u;
}

/* Copie de la commande pour la periode ; l'ISR interrompt l'ecrivain (moins prioritaire) : si une
 * publication est en cours, la commande precedente est conservee */
RAMFUNC static void cmd_fetch(void)
{
    uint32_t q = g_foc_cmd.seq;
    if (q & 1u) return;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    foc_cmd_t t = g_foc_cmd;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    if (g_foc_cmd.seq == q) S.cmd = t;
}

/* Hors ISR. p_pending est retire avant la copie : une copie inachevee n'est jamais appliquee. */
void foc_set_params(const foc_params_t *p)
{
    S.p_pending = false;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    S.p_next = *p;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    S.p_pending = true;
}

const foc_params_t *foc_params(void) { return &S.p; }
foc_state_t foc_state(void) { return S.st; }
bool foc_calibrated(void) { return S.calibrated; }

bool foc_detect_start(const foc_det_t *req)
{
    if (S.st != FOC_ST_IDLE || !S.calibrated || S.det.busy || req->kind == FOC_DET_NONE) return false;
    if ((req->kind == FOC_DET_LAMBDA || req->kind == FOC_DET_ACIM) && !(req->r_known > 1e-5f && req->l_known > 1e-8f)) return false;
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

    /* Priorite a l'axe d (defluxage, stabilite), dans la limite foc_mag_vd_max, puis q dans le reste */
    float vd_s = fm_sat_sym(vd, vmax * S.p.vd_max);
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
 * puis se relachent vers zero. Pas de boucle sur le duty mesure en montee.
 * Descente (consigne plus faible que le duty applique, meme sens) : regulateur
 * PI duty -> courant de frein (foc_duty_dowmramp_kp / ki), borne a [i_lo, i_hi],
 * jusqu'a atteindre la consigne ; la tension imposee reprend ensuite. */
RAMFUNC static void duty_loop(float kp, float ki, float vd_ff, float vq_ff, float vmax)
{
    const float dt = S.dt;
    float vref = fm_max(S.vbus_f * FM_1_SQRT3, 1.0f);
    if (!S.duty_on) {
        S.duty_on = true;
        S.dr_on = false;
        S.dc_on = false;
        S.vq_duty = S.vq;
        S.int_hi = S.int_lo = 0.0f;
    }
    float d_tgt = S.cmd.duty;
    float d_now = S.vq / vref;
    bool same = d_tgt * d_now >= 0.0f;
    if (S.p.duty_basic) {
        /* mode duty simple : consigne sous le duty present -> courant nul (ni frein ni couple), pont
         * actif ; reprise du suivi du duty quand il a rejoint la consigne (continu : vq_duty = vq) */
        if (!S.dc_on && same && fm_abs(d_tgt) < fm_abs(d_now) - 0.01f) S.dc_on = true;
        if (S.dc_on && (!same || fm_abs(d_now) <= fm_abs(d_tgt) + 0.003f)) S.dc_on = false;
        if (S.dc_on) {
            S.iq_ref += fm_sat_sym(0.0f - S.iq_ref, S.p.i_slew * dt);
            current_loop(kp, ki, vd_ff, vq_ff, vmax);
            S.vq_duty = S.vq;
            S.int_hi = S.int_lo = 0.0f;
            return;
        }
    }
    if (!S.dr_on && same && !S.p.duty_basic && fm_abs(d_tgt) < fm_abs(d_now) - 0.02f && S.p.duty_dr_kp + S.p.duty_dr_ki > 0.0f) {
        S.dr_on = true;
        S.dr_int = S.iq;
    }
    if (S.dr_on) {
        float e = d_tgt - d_now;
        S.dr_int = fm_sat(S.dr_int + S.p.duty_dr_ki * e * dt, S.cmd.i_lo, S.cmd.i_hi);
        S.iq_ref = fm_sat(S.p.duty_dr_kp * e + S.dr_int, S.cmd.i_lo, S.cmd.i_hi);
        current_loop(kp, ki, vd_ff, vq_ff, vmax);
        S.vq_duty = S.vq;
        S.int_hi = S.int_lo = 0.0f;
        if (!same || fm_abs(d_now) <= fm_abs(d_tgt) + 0.005f) S.dr_on = false;
        return;
    }

    float tgt = fm_sat_sym(d_tgt * S.vbus_f * FM_1_SQRT3, vmax);
    S.vq_duty += fm_sat_sym(tgt - S.vq_duty, vmax * dt / 0.1f);

    float ehi = S.cmd.i_hi - S.iq, elo = S.cmd.i_lo - S.iq;
    S.int_hi = fm_min(S.int_hi + ki * dt * ehi, 0.0f);
    S.int_lo = fm_max(S.int_lo + ki * dt * elo, 0.0f);
    float corr = S.int_hi + S.int_lo + kp * (fm_min(ehi, 0.0f) + fm_max(elo, 0.0f));

    float ed = S.id_ref - S.id;
    S.int_d += ki * dt * ed;
    float vd = kp * ed + S.int_d + vd_ff;
    float vd_s = fm_sat_sym(vd, vmax * S.p.vd_max);
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
    /* foc_short_ls_on_zero_duty : tensions nulles = phases en court-circuit ; low-sides passants en
     * continu plutot qu'un court-circuit module (moins de pertes, pas d'erreur de temps mort,
     * plus de couple de freinage a basse vitesse). Pas pendant l'injection HFI. */
    if (S.p.short_ls && !S.hfi_on && S.vd == 0.0f && S.vq == 0.0f)
        S.duty[0] = S.duty[1] = S.duty[2] = 0.0f;
}

/* Modulation avec l'angle predit au milieu de l'intervalle d'application. ctl_mode 2 : la commande
 * s'applique en deux moities (creux -> sommet, puis sommet -> creux avec l'angle interpole par
 * foc_isr_half) ; centres a delay_comp -/+ 1/4 de periode. */
RAMFUNC static void modulate_out(void)
{
    const float d = S.p.delay_comp;
    S.h_ok = S.p.ctl_mode == 2 && !S.hfi_on && (S.st == FOC_ST_RUN || S.st == FOC_ST_START);
    if (S.h_ok) {
        modulate(S.theta + S.omega * (d - 0.25f) * S.dt);
        S.theta_h = S.theta + S.omega * (d + 0.25f) * S.dt;
    } else {
        modulate(S.theta + S.omega * d * S.dt);
    }
}

RAMFUNC static void bridge_off(void)
{
    S.br = false;
    S.brk_short = false;
    S.brk_n = 100;              /* frein demarre moteur lance : regulation directe */
    S.duty_on = false;
    S.hfi_on = false;
    S.use_hfi = false;
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
            /* intervalle entre deux echantillons : seconde moitie de la commande precedente (identique a
             * celle-ci, ou interpolee en ctl_mode 2), puis premiere moitie de la derniere commande */
            float d1 = S.br_p1 ? S.duty_p1[x] : 0.5f;
            float d2 = S.br_p2 ? S.duty_h_prev[x] : d1;
            v[x] = 0.5f * (d1 + d2) * k - tc * fm_sat_sym(S.i_ph[x], 1.0f);
        }
        if (S.pf_use) {
            /* filtre de phase actif : tensions mesurees (sans erreur de temps mort) ;
             * l'ecart avec la tension commandee sert a la surveillance */
            float da = (2.0f * v[0] - v[1] - v[2]) * (1.0f / 3.0f), db = (v[1] - v[2]) * FM_1_SQRT3;
            float ma = (2.0f * vph_meas[0] - vph_meas[1] - vph_meas[2]) * (1.0f / 3.0f);
            float mb = (vph_meas[1] - vph_meas[2]) * FM_1_SQRT3;
            float e = fm_sqrt((ma - da) * (ma - da) + (mb - db) * (mb - db));
            FM_LPF(S.pf_err, e, 0.01f);
            *va = ma;
            *vb = mb;
            return;
        }
    } else {
        v[0] = vph_meas[0];
        v[1] = vph_meas[1];
        v[2] = vph_meas[2];
    }
    *va = (2.0f * v[0] - v[1] - v[2]) * (1.0f / 3.0f);
    *vb = (v[1] - v[2]) * FM_1_SQRT3;
}

/* ------------------------------------------------ injection haute frequence
 * Creneaux ++-- de tension sur l'axe d estime (periode 4 Tpwm). Un intervalle entre
 * deux echantillons n'est exploite que s'il est entierement sous un meme signe (les
 * deux derniers creneaux ecrits identiques). Deux intervalles purs de signes opposes
 * se soustraient : la pente due au fondamental s'elimine et il reste
 *   sigma * (dq_new - dq_prev) / 2 = -K sin(2 eps),  K = V T / 2 (1/Ld - 1/Lq)
 * sur l'axe q estime (eps = avance de l'estimation sur le rotor), et sur l'axe d
 *   sigma * (dd_new - dd_prev) / 2 ~ V T / Ld  (amplitude, pour la polarite).
 * Toutes les 'samples' demodulations : filtre alpha-beta sur l'angle. */
static bool hfi_possible(void) { return S.oc.lq > 1.05f * S.oc.ld && S.oc.ld > 1e-8f; }

RAMFUNC static void hfi_reset(float theta, float omega)
{
    S.hfi_theta = theta;
    S.hfi_omega = omega;
    S.hfi_ph = 0;
    S.hfi_s1 = S.hfi_s2 = 0;
    S.hfi_sig_prev = 0;
    S.hfi_acc = S.hfi_amp_acc = 0.0f;
    S.hfi_n = S.hfi_amp_n = 0;
    S.hfi_upd = 0;
    S.hfi_good = 0;
    S.hfi_ia = S.i_a;
    S.hfi_ib = S.i_b;
}

/* Demodulation : a appeler a chaque periode ou l'injection est active */
RAMFUNC static void hfi_measure(void)
{
    const float dt = S.dt;
    S.hfi_theta = fm_wrap_pi(S.hfi_theta + S.hfi_omega * dt);
    float dia = S.i_a - S.hfi_ia, dib = S.i_b - S.hfi_ib;
    S.hfi_ia = S.i_a;
    S.hfi_ib = S.i_b;
    if (S.hfi_s1 == 0 || S.hfi_s1 != S.hfi_s2) return;     /* intervalle mixte */
    float sn, cs;
    fm_sincos(S.hfi_theta, &sn, &cs);
    float dq = -sn * dia + cs * dib;
    float dd = cs * dia + sn * dib;
    int8_t sig = S.hfi_s1;
    if (S.hfi_sig_prev == -sig) {
        float x = (float)sig * (dq - S.hfi_q_prev) * 0.5f;
        float a = (float)sig * (dd - S.hfi_d_prev) * 0.5f;
        S.hfi_amp_acc += a;
        S.hfi_amp_n++;
        /* PLL du second ordre corrigee a chaque demodulation (toutes les 2 periodes) :
         * bande wb = 2000 x gain rad/s, amortissement 1 */
        float k = 0.5f * S.hfi_v * dt * (1.0f / S.oc.ld - 1.0f / S.oc.lq);
        float eps = k > 1e-6f ? -0.5f * x / k : 0.0f;
        eps = fm_sat_sym(eps, fm_max(S.p.hfi.max_err, 0.05f));
        float wb = 2000.0f * fm_sat(S.p.hfi.gain, 0.01f, 1.0f);
        float t2 = 2.0f * dt;
        S.hfi_theta = fm_wrap_pi(S.hfi_theta - 2.0f * wb * t2 * eps);
        S.hfi_omega -= wb * wb * t2 * eps;
        S.hfi_acc += eps;
        S.hfi_n++;
        /* bloc de 'samples' demodulations : erreur moyenne (convergence, telemetrie) */
        uint16_t n = S.p.hfi.samples ? S.p.hfi.samples : 16;
        if (S.hfi_n >= n) {
            float em = S.hfi_acc / (float)S.hfi_n;
            g_foc_telem.hfi_err = em;
            S.hfi_good = fm_abs(em) < 0.1f ? (uint16_t)(S.hfi_good + 1) : 0;
            S.hfi_acc = 0.0f;
            S.hfi_n = 0;
            if (S.hfi_upd < 0xFFFF) S.hfi_upd++;
        }
    }
    S.hfi_q_prev = dq;
    S.hfi_d_prev = dd;
    S.hfi_sig_prev = sig;
}

/* Ajoute le creneau a la tension d calculee par la boucle de courant */
RAMFUNC static void hfi_inject(float vmax)
{
    int8_t sig = ((S.hfi_ph >> 1) & 1) ? -1 : 1;
    S.hfi_ph++;
    S.hfi_s2 = S.hfi_s1;
    S.hfi_s1 = sig;
    float v = fm_min(S.hfi_v, 0.3f * vmax);
    S.vd = fm_sat_sym(S.vd + (float)sig * v, vmax);
    float vq_lim = fm_sqrt(fm_max(vmax * vmax - S.vd * S.vd, 0.0f));
    S.vq = fm_sat_sym(S.vq, vq_lim);
}

/* Demarrage HFI / VSS depuis l'arret : convergence, test de polarite, puis marche */
RAMFUNC static void enter_hfi_start(void)
{
    hfi_reset(S.obs.theta, 0.0f);
    S.hfi_on = true;
    S.hfi_stage = 0;
    S.hfi_t = 0.0f;
    S.hfi_v = S.p.hfi.v_start;
    S.id_ref = S.iq_ref = 0.0f;
    S.int_d = S.int_q = 0.0f;
    S.sub = ST_HFI;
    S.lock_t = 0.0f;
    set_state(FOC_ST_START);
}

/* ---------------------------------------------- demarrage I/f et relais */
/* Courant de demarrage : la consigne, bornee par st_i (boost) et st_i_max */
RAMFUNC static inline float st_cur(float cmd)
{
    return fm_min(fm_max(S.p.st_i, fm_abs(cmd)), S.p.st_i_max);
}

RAMFUNC static void enter_start(bool from_run)
{
    if (from_run) {
        /* Reprise en I/f a partir de l'etat observe, courant sur l'axe q rotor */
        S.ol_theta = S.obs.theta;
        S.ol_omega = S.obs.omega;
        S.sub = ST_RAMP;
        S.ol_iq = (float)S.dir * st_cur(S.iq_ref);
        /* Le PI tourne sans anticipation en I/f : il porte toute la tension */
        S.id_ref = acim() ? S.p.acim_im : 0.0f;
        S.iq_ref = S.ol_iq;
        pi_preload(S.ol_omega, false);
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

/* Induction : magnetisation (courant d jusqu'au courant magnetisant, ~3 Tr) avant tout couple.
 * Avec codeur, le champ suit le rotor (aucun couple de freinage) ; sans capteur, il reste fixe. */
RAMFUNC static void enter_acim_start(void)
{
    S.id_ref = S.iq_ref = 0.0f;
    S.int_d = S.int_q = 0.0f;
    S.ol_theta = S.theta;
    S.ol_omega = 0.0f;
    S.ol_iq = 0.0f;
    S.slip_acc = 0.0f;
    S.ac_t_mag = fm_sat(3.0f * acim_tr(), 0.05f, 3.0f);
    S.sub = ST_MAG;
    S.lock_t = 0.0f;
    set_state(FOC_ST_START);
}

/* Frein depuis l'arret ou a basse vitesse : pont actif, phases en court-circuit (tensions nulles ;
 * low-sides passants avec foc_short_ls_on_zero_duty). Le frein en courant reprend des que la
 * vitesse permet d'atteindre le courant demande. */
RAMFUNC static void enter_brake_short(void)
{
    S.id_ref = S.iq_ref = 0.0f;
    S.int_d = S.int_q = 0.0f;
    S.vd = S.vq = 0.0f;
    S.brk_short = true;
    S.brk_n = 0;
    S.slew = S.p.i_slew;
    S.slew_t = 0.0f;
    S.low_t = S.flux_bad_t = S.hall_bad_t = 0.0f;
    S.use_obs = S.p.sensor_mode != FOC_SENSOR_ENCODER && S.p.sensor_mode != FOC_SENSOR_HALL;
    S.use_hfi = false;
    S.hall_mix = 0.0f;
    S.fw_id = 0.0f;
    S.w_pll_theta = S.obs.theta;
    S.w_pll_omega = S.obs.omega;
    set_state(FOC_ST_RUN);
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
    if (acim()) {
        /* induction : le flux reste sur d, le couple sur q */
        S.id_ref = S.p.acim_im;
        S.iq_ref = S.ol_iq;
    }
    pi_preload(S.obs.omega, true);
    S.slew = fm_max(imag, 2.0f) * 5.0f;   /* rejoint doucement la consigne (200 ms) */
    S.w_pll_theta = S.obs.theta;
    S.w_pll_omega = S.obs.omega;
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
        case FOC_DET_ACIM:    S.ds = DS_AC_PRE; break;
        default:              S.ds = DS_SEN_ALIGN; break;
        }
        /* R/L : vecteur sur l'axe beta -> phase A sans courant, B et C a +-0.87 I.
         * Une phase a courant nul a une erreur de temps mort nulle et constante. */
        S.ol_theta = (S.det.kind == FOC_DET_RL || S.det.kind == FOC_DET_ACIM) ? 0.5f * FM_PI : 0.0f;
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
            if (r_lo > 1.1f * r_hi && r_hi > 0.0f) { r = r_hi; S.det.warn = 2; }   /* temps mort non sature en bas */
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

    /* --- Induction : echelon de courant continu (pas de couple, champ fixe).
     *     Palier I/2 pendant 2 s (flux rotorique etabli), echelon a I. Apres la pointe inductive
     *     (~ms), vd(t) = vinf + K dI / Tr e^(-t/Tr), K = Lm^2 / Lr (flux rotorique qui s'etablit).
     *     vd est enregistre toutes les 2 ms pendant 1.2 s, vinf moyenne entre 1.8 et 2.5 s.
     *     Tr : regression de ln(vd - vinf) entre 80 % et 10 % de l'amplitude a 6 ms ;
     *     K : amplitude extrapolee a t = 0. Lr = Lm + Llr, avec Llr ~ sigma Ls / 2. */
    case DS_AC_PRE:
        S.id_ref += fm_sat_sym(0.5f * I - S.id_ref, I * dt / 0.1f);
        if (S.dst >= 2.0f) {
            S.ac_i0 = S.id;
            S.ds = DS_AC_STEP;
            S.dst = 0.0f;
            S.ac_n = 0;
            S.acc_v = S.acc_i = S.acc_n = 0.0f;
            S.acc3[0] = S.acc3[1] = 0.0f;
        }
        break;
    case DS_AC_STEP: {
        S.id_ref = I;
        /* moyenne de vd sur chaque tranche de 2 ms */
        S.acc3[0] += S.vd;
        S.acc3[1] += 1.0f;
        if (S.dst >= 0.002f * (float)(S.ac_n + 1) && S.ac_n < AC_N) {
            s_acbuf[S.ac_n++] = S.acc3[0] / S.acc3[1];
            S.acc3[0] = S.acc3[1] = 0.0f;
        }
        if (S.dst > 1.8f) { S.acc_v += S.vd; S.acc_i += S.id; S.acc_n += 1.0f; }
        if (S.dst >= 2.5f) {
            float vinf = S.acc_v / S.acc_n, i1 = S.acc_i / S.acc_n;
            float di = i1 - S.ac_i0;
            int kref = 3;                                   /* tranche centree sur 7 ms */
            float aref = s_acbuf[kref] - vinf;
            float sx = 0, sy = 0, sxx = 0, sxy = 0, n = 0;
            for (int k = kref; k < S.ac_n; k++) {
                float a = s_acbuf[k] - vinf;
                if (a > 0.8f * aref) continue;
                if (a < 0.1f * aref) break;
                float t = 0.002f * ((float)k + 0.5f), y = logf(a);
                sx += t; sy += y; sxx += t * t; sxy += t * y; n += 1.0f;
            }
            float den = n * sxx - sx * sx;
            float slope = (n >= 4.0f && den > 0.0f) ? (n * sxy - sx * sy) / den : 0.0f;
            float tr = slope < 0.0f ? -1.0f / slope : -1.0f;
            float a0 = (n >= 4.0f && tr > 0.0f) ? expf((sy - slope * sx) / n) : 0.0f;   /* amplitude a t = 0 */
            float k = (di > 0.1f * I && tr > 0.0f) ? a0 * tr / di : -1.0f;
            if (!(aref > 0.0f && k > 1e-7f && tr > 2e-3f && tr < 2.0f)) { det_finish(-10); return; }
            float sls = S.det.l_known;
            float llr = 0.5f * sls;
            float lm = 0.5f * (k + fm_sqrt(k * k + 4.0f * k * llr));
            S.det.acim_k = k;
            S.det.acim_tr = tr;
            S.det.acim_lm = lm;
            S.det.acim_lr = lm + llr;
            S.det.acim_rr = (lm + llr) / tr;
            S.ds = DS_END;
            S.dst = 0.0f;
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
    cmd_fetch();
    {
        /* lecture et effacement atomiques : une protection de priorite superieure peut s'intercaler */
        uint32_t code = __atomic_exchange_n(&S.fault_req, RESC_FAULT_NONE, __ATOMIC_SEQ_CST);
        if (code != RESC_FAULT_NONE) trip_at(code, 0);
    }

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
        vph[x] = S.vph_raw[x] - S.off_vph[x];
    }

    /* ---- 2. Calibration des offsets (pont coupe) */
    if (S.st == FOC_ST_INIT || S.st == FOC_ST_CALIB) {
        bridge_off();
        if (S.st_time > 0.05f) {
            for (int x = 0; x < 3; x++) {
                S.cal_acc[x] += (float)cs[x];
                if (S.cal_n == 0) { S.cal_vacc[x] = 0.0f; S.cal_vmin[x] = S.cal_vmax[x] = S.vph_raw[x];
                                    S.cal_dacc[x] = S.cal_dsq[x] = 0.0f; }
                S.cal_vacc[x] += S.vph_raw[x];
                {
                    float dv = S.vph_raw[x] - (S.vph_raw[0] + S.vph_raw[1] + S.vph_raw[2]) * (1.0f / 3.0f);
                    S.cal_dacc[x] += dv;
                    S.cal_dsq[x] += dv * dv;
                }
                S.cal_vmin[x] = fm_min(S.cal_vmin[x], S.vph_raw[x]);
                S.cal_vmax[x] = fm_max(S.cal_vmax[x], S.vph_raw[x]);
            }
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
                /* Offsets de tension pont coupe, comme la calibration au demarrage de VESC : ecart de chaque
                 * phase a la moyenne des trois, moteur a l'arret (sinon : valeurs de la configuration). Ils ne
                 * servent qu'en roue libre (l'observateur suit alors les tensions mesurees). */
                {
                    float vm = (S.cal_vacc[0] + S.cal_vacc[1] + S.cal_vacc[2]) / (3.0f * (float)S.cal_n);
                    /* moteur a l'arret : l'ecart entre phases ne varie pas (un moteur qui tourne donne une
                     * FEM alternative) ; ecart-type, insensible aux pics de bruit isoles */
                    bool still = true;
                    float sd_max = 0.0f;
                    for (int x = 0; x < 3; x++) {
                        float mn = S.cal_dacc[x] / (float)S.cal_n;
                        float sd = fm_sqrt(fm_max(S.cal_dsq[x] / (float)S.cal_n - mn * mn, 0.0f));
                        sd_max = fm_max(sd_max, sd);
                    }
                    if (sd_max > 0.3f) still = false;
                    g_foc_telem.off_vph_sd = sd_max;
                    if (still) {
                        for (int x = 0; x < 3; x++) S.off_vph[x] = fm_sat_sym(S.cal_vacc[x] / (float)S.cal_n - vm, 1.0f);
                    }
                    for (int x = 0; x < 3; x++) g_foc_telem.off_vph_used[x] = S.off_vph[x];
                    g_foc_telem.off_vph_boot = still;
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
        /* echantillon au centre de la fenetre low-side (declenchement fixe au sommet) : il faut t_win AVANT lui,
         * soit une fenetre totale de 2 t_win (doc 04 §4.4) */
        float wmin = 2.0f * S.p.t_win * S.p.f_pwm;
        for (int x = 0; x < 3; x++) valid[x] = (1.0f - fm_max(S.duty_p1[x], S.duty_h[x])) >= wmin;
        /* foc_current_sample_mode 0 (plus longue fenetre) : la phase a la fenetre la plus courte est
         * reconstruite des deux autres des que sa fenetre approche la limite */
        if (S.p.cs_mode == 0 && valid[0] && valid[1] && valid[2] && !S.p.cs_two_shunts) {
            int w = S.duty_p1[0] >= S.duty_p1[1] ? (S.duty_p1[0] >= S.duty_p1[2] ? 0 : 2) : (S.duty_p1[1] >= S.duty_p1[2] ? 1 : 2);
            if ((1.0f - S.duty_p1[w]) < 2.0f * wmin) valid[w] = false;
        }
    }
    if (S.p.cs_two_shunts) valid[2] = false;
    /* foc_current_sample_mode 2 (fort courant) : la phase au plus fort courant est deduite des deux
     * autres (mesure au-dela de la plage d'un ampli) */
    if (S.p.cs_mode == 2 && valid[0] && valid[1] && valid[2]) {
        float a0 = fm_abs(S.i_ph[0]), a1 = fm_abs(S.i_ph[1]), a2 = fm_abs(S.i_ph[2]);
        valid[a0 >= a1 ? (a0 >= a2 ? 0 : 2) : (a1 >= a2 ? 1 : 2)] = false;
    }

    const bool br_was_on = S.br;                  /* une coupure a ce tour : sortie immediate (pont coupe) */
    /* ---- 4. Protections rapides : surintensite echantillon par echantillon (pas de filtre : la carte
     * n'a pas de protection materielle independante ; le watchdog analogique double cette coupure) */
    for (int x = 0; x < 3; x++) {
        if (valid[x] && fm_abs(S.i_ph[x]) > S.p.i_abs_trip) {
            trip(RESC_FAULT_ABS_OVER_CURRENT);
            break;
        }
    }
    /* tension bus : 4 echantillons consecutifs (0.27 ms a 15 kHz), un pic de bruit isole ne coupe pas */
    S.ov_n = S.vbus > S.p.vbus_trip_max ? (uint8_t)(S.ov_n < 255 ? S.ov_n + 1 : 255) : 0;
    S.uv_n = (S.br && S.vbus < S.p.vbus_trip_min) ? (uint8_t)(S.uv_n < 255 ? S.uv_n + 1 : 255) : 0;
    if (S.ov_n >= 4) trip(RESC_FAULT_OVER_VOLTAGE);
    if (S.uv_n >= 4) trip(RESC_FAULT_UNDER_VOLTAGE);

    {
        uint32_t np0 = g_foc_telem.n_predicted;
        reconstruct_currents(valid);
        S.pr_meas_prev = S.pr_meas;
        S.pr_meas = g_foc_telem.n_predicted == np0;
    }
    {
        float imag = fm_sqrt(S.i_a * S.i_a + S.i_b * S.i_b);
        FM_LPF(S.i_abs_f, imag, 0.2f);                /* ~0.3 ms a 15 kHz */
        if (imag > S.p.i_abs_trip) trip(RESC_FAULT_ABS_OVER_CURRENT);
        if (imag > g_foc_telem.i_phase_max) g_foc_telem.i_phase_max = imag;
    }
    if (br_was_on && !S.br) goto output;

    /* ---- 5. Observateur (toujours actif : suit aussi un moteur en roue libre) */
    {
        /* filtre de phase commute en dessous de pf_omega (hysteresis 10 %) */
        bool want = S.p.pf_enable && fm_abs(S.obs.omega) < (S.pf_on ? 1.1f : 0.9f) * S.p.pf_omega;
        if (want != S.pf_on) {
            S.pf_on = want;
            S.pf_t = 0.0f;
            S.pf_err = 0.0f;
            foc_hw_phase_filter(want);
        }
        S.pf_t += dt;
        S.pf_use = S.pf_on && S.pf_t > 0.003f && S.br_p1 && S.br_p2;
        if (S.pf_use && S.br && !S.p.pf_no_fault) {
            if (S.pf_err > fm_max(2.0f, 0.15f * S.vbus_f)) S.pf_bad_t += dt; else S.pf_bad_t = 0.0f;
            if (S.pf_bad_t > 0.1f) trip(RESC_FAULT_PHASE_FILTER);
        } else {
            S.pf_bad_t = 0.0f;
        }

        if (acim() && acim_ok()) {
            /* modele en courant : d(psi_r)/dt = (Lm id - psi_r) / Tr ; glissement Lm iq / (Tr psi_r) */
            float tr = acim_tr();
            float idm = S.br_p1 ? S.id : 0.0f;
            S.psir += (S.p.acim_lm * idm - S.psir) * fm_min(dt / tr, 1.0f);
            if (S.psir < 0.0f) S.psir = 0.0f;
            float pmin = 0.1f * S.p.acim_lm * S.p.acim_im;
            S.ws = S.br_p1 ? S.p.acim_lm * S.iq / (tr * fm_max(S.psir, pmin)) : 0.0f;
            g_foc_telem.acim_psir = S.psir;
            g_foc_telem.acim_slip = S.ws;
        }
        motor_comp();
        /* pont coupe (moteur non pilote) : courant nul pour l'observateur, comme VESC (« The current is 0
         * when the motor is undriven ») ; les shunts bas ne donnent alors pas de mesure exploitable et un
         * faux courant fait deriver le flux estime pendant la roue libre */
        const bool driven = S.br_p1 || S.br_p2;
        S.oc.id = driven ? S.id : 0.0f;
        S.oc.iq = driven ? S.iq : 0.0f;
        S.oc.adapt_on = S.st == FOC_ST_RUN && S.use_obs;   /* RESC _LAMBDA : seulement en marche observee */
        float va, vb;
        bool v_avg = false;                       /* tension moyennee : elle date du milieu de l'intervalle */
        applied_voltage(vph, &va, &vb);
        if (!driven) {
            /* pont coupe : la mesure est la tension a l'instant de l'echantillon ; l'observateur integre la
             * moyenne de l'intervalle ecoule -> moyenne des deux derniers echantillons (sinon retard d'une
             * demi-periode : 6.7 deg a 30000 ERPM, pointe de courant a la reprise) */
            float pa = S.meas_n ? S.va_prev : va, pb = S.meas_n ? S.vb_prev : vb;
            S.va_prev = va;
            S.vb_prev = vb;
            /* utile en vitesse seulement (retard negligeable a basse vitesse : demarrages inchanges) */
            v_avg = fm_abs(S.obs.omega) * dt > 0.05f;
            if (v_avg) {
                va = 0.5f * (va + pa);
                vb = 0.5f * (vb + pb);
            }
        }
#ifdef RESC_HOST
        g_foc_telem.dbg_va = va; g_foc_telem.dbg_vb = vb; g_foc_telem.dbg_ia = S.i_a; g_foc_telem.dbg_ib = S.i_b;
#endif
        /* niveau de tension pour le gain a basse vitesse : modulation appliquee ou FEM
         * (pont coupe, la modulation est nulle mais la FEM renseigne la vitesse) */
        /* pont coupe : modulation equivalente a la tension mesuree (comme VESC), pas a la vitesse estimee :
         * un observateur qui deriverait ne baisse pas lui-meme son gain */
        float m_obs = driven ? fm_max(S.duty_abs, fm_abs(S.obs.omega) * S.lam_eff / fm_max(S.vbus_f * FM_1_SQRT3, 1.0f))
                             : fm_sqrt(va * va + vb * vb) / fm_max(S.vbus_f * FM_1_SQRT3, 1.0f);
        obs_update(&S.obs, &S.oc, va, vb, driven ? S.i_a : 0.0f, driven ? S.i_b : 0.0f, m_obs, dt);
        if (!driven) {
            /* FEM mesuree : angle de la tension (milieu de l'intervalle si moyennee) ; attendue : flux observe
             * + 90 deg dans le sens de rotation, ramene au meme instant */
            float th_v = fm_atan2(vb, va);
            float sg = S.obs.omega >= 0.0f ? 1.0f : -1.0f;
            float th_e = S.obs.theta - (v_avg ? 0.5f * S.obs.omega * dt : 0.0f) + sg * FM_PI_2;
            float se, ce;
            fm_sincos(th_v - th_e, &se, &ce);
            S.rv_e = fm_wrap_pi(th_v - th_e);
            float vm = fm_sqrt(va * va + vb * vb);
            if (S.meas_n == 0u) {
                S.rv_s = se; S.rv_c = ce; S.rv_mag = vm;
                S.rv_th_prev = th_v;
                S.rv_w = S.obs.omega;
            } else {
                FM_LPF(S.rv_s, se, 0.0625f);
                FM_LPF(S.rv_c, ce, 0.0625f);
                FM_LPF(S.rv_mag, vm, 0.0625f);
                FM_LPF(S.rv_w, fm_wrap_pi(th_v - S.rv_th_prev) / dt, 0.1f);
                S.rv_th_prev = th_v;
            }
            float sw = S.rv_w >= 0.0f ? 1.0f : -1.0f;
            S.rv_flux = fm_wrap_pi(th_v - sw * FM_PI_2 + (v_avg ? 0.5f * S.rv_w * dt : 0.0f));
            g_foc_telem.resume_err = fm_atan2(S.rv_s, S.rv_c);
            if (S.meas_n < 0xFFFFFFFFu) S.meas_n++;
        } else {
            S.meas_n = 0u;
        }
        S.lam_eff = S.obs.lambda;
    }
    float flux_ratio = S.obs.flux / S.lam_eff;

    if (S.hfi_on) hfi_measure();
    g_foc_telem.hfi_active = S.hfi_on;

    /* ---- 5b. Capteurs d'angle */
    bool sensor_ok = false;
    if (S.p.sensor_mode == FOC_SENSOR_HALL) {
        hall_update(&S.hall, &S.p.hall, S.hall_now, dt);
        sensor_ok = S.hall.ok;
        g_foc_telem.sens_theta = S.hall.theta;
    } else if (S.p.sensor_mode == FOC_SENSOR_ENCODER && S.p.enc.cpr >= 4) {
        enc_update(&S.enc, &S.p.enc, S.enc_now, dt);
        sensor_ok = true;
        g_foc_telem.sens_theta = S.enc.theta;
    }

    /* ---- 6. Machine d'etats */
    /* cercle de tension : modulation lineaire max x foc_overmod_factor (au-dela de 1 : surmodulation,
     * les rapports cycliques sont ecretes par phase) */
    float vmax = S.p.m_max * S.vbus_f * FM_1_SQRT3 * S.p.overmod;

    switch (S.st) {
    case FOC_ST_FAULT:
        bridge_off();
        if (S.clear_req) {
            S.clear_req = false;
            S.fault = RESC_FAULT_NONE;
            g_foc_telem.fault = RESC_FAULT_NONE;
            foc_hw_rearm();
            if (S.calibrated) {
                set_state(FOC_ST_IDLE);
            } else {
                /* defaut pendant la calibration (offsets, watchdog au demarrage...) : on la refait,
                 * jamais de pont actif avec des offsets de courant non mesures */
                S.cal_n = 0;
                for (int x = 0; x < 3; x++) S.cal_acc[x] = 0.0f;
                set_state(FOC_ST_CALIB);
                goto output;
            }
        }
        S.theta = S.obs.theta;
        S.omega = S.obs.omega;
        goto output;

    case FOC_ST_IDLE:
        bridge_off();
        S.theta = S.obs.theta;
        S.omega = S.obs.omega;
        if (!S.calibrated) goto output;           /* jamais de pont sans offsets mesures */
        if (S.det.busy) {
            set_state(FOC_ST_DETECT);
            S.ds = DS_IDLE;
            goto output;
        }
        if (S.cmd.run && S.cmd.openloop && S.vbus > S.p.vbus_trip_min) {
            enter_forced();
        } else if (S.cmd.run && acim() && motor_params_ok() && S.vbus > S.p.vbus_trip_min) {
            S.dir = S.cmd.dir >= 0 ? 1 : -1;
            if (S.cmd.allow_start || S.p.sensor_mode == FOC_SENSOR_ENCODER) enter_acim_start();
            else if (S.cmd.brake) enter_brake_short();
        } else if (S.cmd.run && motor_params_ok() && S.vbus > S.p.vbus_trip_min) {
            S.dir = S.cmd.dir >= 0 ? 1 : -1;
            bool obs_ok0 = flux_ratio > 0.7f && flux_ratio < 1.4f;
            bool obs_fast = !acim() && fm_abs(S.obs.omega) > 0.8f * S.p.st_omega && obs_ok0;
            /* HFI / VSS : moteur encore lance au-dessus de foc_hfi_reset_erpm, reprise sur l'observateur
             * sans nouvelle recherche de position (en dessous : recherche et polarite) */
            if ((S.p.sensor_mode == FOC_SENSOR_HFI || S.p.sensor_mode == FOC_SENSOR_VSS) && S.p.hfi.reset_omega > 0.0f &&
                fm_abs(S.obs.omega) > S.p.hfi.reset_omega && obs_ok0)
                obs_fast = true;
            /* Hall : l'observateur a pu decrocher pendant la roue libre (vitesse fausse) ; s'il s'ecarte de
             * plus de 20 % de la vitesse des sondes, il est recale sur elles avant la reprise, qui part
             * alors de la FEM a la vitesse des sondes (sinon pointe de courant a la reprise) */
            if (S.p.sensor_mode == FOC_SENSOR_HALL && sensor_ok && fm_abs(S.hall.omega) > 0.5f * S.p.sl_omega &&
                (!obs_ok0 || fm_abs(S.obs.omega - S.hall.omega) > 0.2f * fm_abs(S.hall.omega))) {
                obs_reset(&S.obs, &S.oc, S.hall.theta, S.hall.omega);
                obs_fast = false;
                g_foc_telem.obs_resync++;
            }
            /* Moteur lance (plus de 2 V de FEM mesuree pont coupe) : sans capteur, la seule facon de reprendre est
             * la reprise au vol, et seulement si l'observateur suit la tension mesuree (ecart < 15 deg filtre,
             * < 30 deg instantane, 32 periodes pont coupe au moins) ; elle est alors permise meme sous le seuil
             * de vitesse habituel. Sinon : pas de pont (ni demarrage I/f depuis l'arret, ni frein en
             * court-circuit, sur un moteur qui tourne) ; avec un capteur valide, demarrage sur le capteur.
             * Apres 0.5 s de refus, l'observateur est recale sur la tension mesuree. */
            bool hold = false;
            bool spinning = !acim() && S.meas_n > 0u && S.rv_mag > 2.0f;
            if (spinning) {
                bool coherent = S.meas_n >= 32u && fm_abs(fm_atan2(S.rv_s, S.rv_c)) < 0.26f && fm_abs(S.rv_e) < 0.52f;
                if (coherent && obs_ok0) {
                    obs_fast = true;                  /* observateur verifie : reprise, meme sous le seuil de vitesse */
                    S.resume_wait_t = 0.0f;
                } else {
                    obs_fast = false;
                    hold = !sensor_ok;                /* capteur valide (Hall, codeur) : demarrage sur le capteur */
                    S.resume_wait_t += dt;
                    g_foc_telem.resume_refused++;
                    if (S.resume_wait_t > 0.5f && S.meas_n >= 32u) {
                        obs_reset(&S.obs, &S.oc, S.rv_flux, S.rv_w);
                        S.resume_wait_t = 0.0f;
                        g_foc_telem.obs_resync++;
                    }
                }
            } else {
                S.resume_wait_t = 0.0f;
            }
            if (hold) {
                /* attente : l'observateur se recale (pont coupe) */
            } else if (sensor_ok && !(S.p.sensor_mode == FOC_SENSOR_HALL && obs_fast && fm_abs(S.obs.omega) > S.p.sl_omega)) {
                /* Capteur : marche directe depuis l'arret, sans boucle ouverte */
                S.id_ref = S.iq_ref = 0.0f;
                pi_preload(S.p.sensor_mode == FOC_SENSOR_HALL ? S.hall.omega : S.enc.omega, true);
                {
                    float w0 = S.p.sensor_mode == FOC_SENSOR_HALL ? S.hall.omega : S.enc.omega;
                    S.duty_sf = w0 * S.lam_eff / fm_max(S.vbus_f * FM_1_SQRT3, 1.0f);
                    S.brk_w_prev = w0;
                    S.brk_vq_prev = S.duty_sf;
                    S.vq = S.duty_sf * fm_max(S.vbus_f * FM_1_SQRT3, 1.0f);   /* mode duty : part de la FEM, pas de 0 */
                }
                S.slew = S.p.i_slew;
                S.slew_t = 0.0f;
                S.low_t = S.flux_bad_t = S.hall_bad_t = 0.0f;
                S.use_obs = false;
                S.hall_mix = 0.0f;
                S.fw_id = 0.0f;
                set_state(FOC_ST_RUN);
            } else if (obs_fast) {
                /* Reprise au vol : integrateurs sur la FEM observee (selon l'anticipation) */
                S.id_ref = S.iq_ref = 0.0f;
                pi_preload(S.obs.omega, true);
                /* frein au vol : modulation et signes courants (pas de court-circuit a pleine vitesse) */
                S.duty_sf = (S.oc.r * S.iq_ref + S.obs.omega * S.lam_eff) / fm_max(S.vbus_f * FM_1_SQRT3, 1.0f);
                S.brk_w_prev = S.obs.omega;
                S.brk_vq_prev = S.duty_sf;
                S.vq = S.duty_sf * fm_max(S.vbus_f * FM_1_SQRT3, 1.0f);       /* mode duty : part de la FEM, pas de 0 */
                S.slew = S.p.i_slew;
                S.slew_t = 0.0f;
                S.low_t = S.flux_bad_t = 0.0f;
                S.use_obs = true;
                S.hall_mix = 1.0f;
                S.fw_id = 0.0f;
                S.w_pll_theta = S.obs.theta;
                S.w_pll_omega = S.obs.omega;
                set_state(FOC_ST_RUN);
            } else if (S.cmd.allow_start && S.p.sensor_mode == FOC_SENSOR_SENSORLESS) {
                enter_start(false);
            } else if (S.cmd.allow_start && (S.p.sensor_mode == FOC_SENSOR_HFI || S.p.sensor_mode == FOC_SENSOR_VSS)) {
                if (hfi_possible()) enter_hfi_start();
                else enter_start(false);                       /* pas de saillance : I/f */
            } else if (S.cmd.brake) {
                enter_brake_short();
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
        modulate_out();
        goto output;
    }

    if (!S.cmd.run) {
        bridge_off();
        set_state(FOC_ST_IDLE);
        goto output;
    }
    /* Passage en boucle ouverte commandee depuis le demarrage ou la marche */
    if (S.cmd.openloop && !(S.st == FOC_ST_START && S.sub == ST_FORCED)) enter_forced();
    /* Fin de la boucle ouverte commandee : pont coupe, la commande suivante redemarre */
    if (!S.cmd.openloop && S.st == FOC_ST_START && S.sub == ST_FORCED) {
        bridge_off();
        set_state(FOC_ST_IDLE);
        goto output;
    }

    if (S.st == FOC_ST_START && S.sub == ST_FORCED) {
        S.br = true;
        float target = S.cmd.openloop == 3 ? 0.0f : S.cmd.ol_omega;
        float acc = fm_max(S.p.st_omega / fm_max(S.p.st_t_ramp, 0.05f), 200.0f);
        if (S.cmd.openloop == 3) S.ol_omega = 0.0f;      /* frein a main : angle fige */
        S.ol_omega += fm_sat_sym(target - S.ol_omega, acc * dt);
        S.ol_theta = fm_wrap_pi(S.ol_theta + S.ol_omega * dt);
        S.theta = S.ol_theta;
        S.omega = S.ol_omega;
        float s, c;
        fm_sincos(S.theta, &s, &c);
        S.id = c * S.i_a + s * S.i_b;
        S.iq = -s * S.i_a + c * S.i_b;
        if (S.cmd.openloop == 2) {
            /* rapport cyclique impose sur l'axe q, sans regulation de courant
             * (les protections de courant de l'ISR restent actives) */
            float v = fm_sat_sym(S.cmd.ol_duty, S.p.m_max) * S.vbus_f * FM_1_SQRT3;
            S.vd = 0.0f;
            S.vq += fm_sat_sym(v - S.vq, vmax * dt / 0.3f);
            S.int_d = S.int_q = 0.0f;
            S.id_ref = 0.0f;
            S.iq_ref = 0.0f;
        } else if (S.cmd.openloop == 3) {
            /* frein a main : courant continu sur l'axe d a l'angle fige, le rotor s'y aligne et y reste */
            S.ol_iq += fm_sat_sym(fm_abs(S.cmd.iq_ref) - S.ol_iq, S.p.i_slew * dt);
            S.id_ref = S.ol_iq;
            S.iq_ref = 0.0f;
            current_loop(S.p.kp, S.p.ki, 0.0f, 0.0f, vmax);
        } else {
            S.ol_iq += fm_sat_sym(S.cmd.iq_ref - S.ol_iq, S.p.i_slew * dt);
            S.id_ref = 0.0f;
            S.iq_ref = S.ol_iq;
            current_loop(S.p.kp, S.p.ki, 0.0f, 0.0f, vmax);
        }
        modulate_out();
        goto output;
    }

    if (S.st == FOC_ST_START && S.sub == ST_HFI) {
        /* convergence (start_updates), puis impulsions d pour la polarite */
        S.br = true;
        S.hfi_t += dt;
        S.theta = S.hfi_theta;
        S.omega = S.hfi_omega;
        float sn, cs;
        fm_sincos(S.theta, &sn, &cs);
        S.id = cs * S.i_a + sn * S.i_b;
        S.iq = -sn * S.i_a + cs * S.i_b;
        float ia = fm_max(S.p.hfi.amb_current, 0.5f);
        float tp = fm_max(S.p.hfi.amb_time, 0.004f);
        bool dbl = S.p.hfi.amb_mode != 1;
        if (S.hfi_stage == 0) {
            S.id_ref = 0.0f;
            /* convergence : start_updates mises a jour au moins, et 5 de suite a moins de 0.1 rad */
            if (S.hfi_upd >= S.p.hfi.start_updates && S.hfi_good >= 5) {
                S.hfi_amp0 = S.hfi_amp_n ? S.hfi_amp_acc / (float)S.hfi_amp_n : 0.0f;
                S.hfi_stage = 1;
                S.hfi_t = 0.0f;
            }
        } else if (S.hfi_stage <= 2) {
            float target = S.hfi_stage == 1 ? ia : -ia;
            S.id_ref = target * fm_min(S.hfi_t / (0.3f * tp), 1.0f);
            if (S.hfi_t < 0.5f * tp) { S.hfi_amp_acc = 0.0f; S.hfi_amp_n = 0; }
            if (S.hfi_t >= tp) {
                float a = S.hfi_amp_n ? S.hfi_amp_acc / (float)S.hfi_amp_n : 0.0f;
                if (S.hfi_stage == 1) S.hfi_amp_p = a; else S.hfi_amp_m = a;
                S.hfi_stage = (S.hfi_stage == 1 && dbl) ? 2 : 3;
                S.hfi_t = 0.0f;
            }
        } else {
            /* l'axe d sature (Ld plus faible, reponse plus forte) du cote de l'aimant */
            float ref = dbl ? S.hfi_amp_m : S.hfi_amp0;
#ifdef RESC_HOST
            g_foc_telem.dbg_amp0 = S.hfi_amp0; g_foc_telem.dbg_amp_p = S.hfi_amp_p; g_foc_telem.dbg_amp_m = S.hfi_amp_m;
            g_foc_telem.dbg_hfi_th = S.hfi_theta;
#endif
            if (S.hfi_amp_p < ref) S.hfi_theta = fm_wrap_pi(S.hfi_theta + FM_PI);
            S.id_ref *= 0.0f;
            S.dir = S.cmd.dir >= 0 ? 1 : -1;
            if (S.p.sensor_mode == FOC_SENSOR_VSS) {
                /* VSS : I/f sans calage, a partir de l'angle trouve */
                S.hfi_on = false;
                S.ol_theta = S.hfi_theta;
                S.ol_omega = 0.0f;
                S.ol_iq = (float)S.dir * st_cur(S.cmd.iq_ref);
                S.sub = ST_RAMP;
                S.lock_t = 0.0f;
                S.int_d = S.int_q = 0.0f;
                S.st_time = 0.0f;
            } else {
                S.use_hfi = true;
                S.iq_ref = 0.0f;
                S.slew = S.p.i_slew;
                S.slew_t = 0.0f;
                S.low_t = S.flux_bad_t = 0.0f;
                S.fw_id = 0.0f;
                S.w_pll_theta = S.hfi_theta;
                S.w_pll_omega = 0.0f;
                set_state(FOC_ST_RUN);
            }
        }
        if (S.st == FOC_ST_START && S.sub == ST_HFI) {
            if (S.st_time > 2.0f) { trip(RESC_FAULT_UNDERSPEED); goto output; }   /* pas de convergence */
            S.iq_ref = 0.0f;
            current_loop(S.p.kp, S.p.ki, 0.0f, 0.0f, vmax);
            hfi_inject(vmax);
            modulate_out();
            goto output;
        }
    }

    if (S.st == FOC_ST_START && S.sub == ST_MAG) {
        S.br = true;
        S.id_ref = S.p.acim_im * fm_min(S.st_time / (0.3f * S.ac_t_mag), 1.0f);
        S.iq_ref = 0.0f;
        if (S.p.sensor_mode == FOC_SENSOR_ENCODER && S.p.enc.cpr >= 4) {
            S.theta = S.enc.theta + S.slip_acc;
            S.omega = S.enc.omega;
        } else {
            S.theta = S.ol_theta;
            S.omega = 0.0f;
        }
        if (S.st_time >= S.ac_t_mag) {
            S.dir = S.cmd.dir >= 0 ? 1 : -1;
            if (S.p.sensor_mode == FOC_SENSOR_ENCODER && S.p.enc.cpr >= 4) {
                S.use_obs = false;
                S.slew = S.p.i_slew;
                S.slew_t = 0.0f;
                S.low_t = S.flux_bad_t = 0.0f;
                set_state(FOC_ST_RUN);
            } else {
                /* sans capteur : rampe de frequence a courant impose, puis observateur */
                S.ol_theta = S.theta;
                S.ol_omega = 0.0f;
                S.ol_iq = (float)S.dir * st_cur(S.cmd.iq_ref);
                S.sub = ST_RAMP;
                S.lock_t = 0.0f;
                S.st_time = 0.0f;
            }
        }
        float sn, cs;
        fm_sincos(S.theta, &sn, &cs);
        S.id = cs * S.i_a + sn * S.i_b;
        S.iq = -sn * S.i_a + cs * S.i_b;
        if (S.st == FOC_ST_START && S.sub == ST_MAG) {
            current_loop(S.p.kp, S.p.ki, 0.0f, 0.0f, vmax);
            modulate_out();
            goto output;
        }
    }

    if (S.st == FOC_ST_START && S.sub != ST_HFI) {
        S.br = true;
        if (S.sub == ST_ALIGN) {
            S.ol_iq = (float)S.dir * st_cur(S.cmd.iq_ref) * fm_min(S.st_time / fm_max(S.p.st_t_align * 0.5f, 1e-3f), 1.0f);
            if (S.st_time >= S.p.st_t_align) S.sub = ST_RAMP;
        } else {
            float acc = S.p.st_omega / fm_max(S.p.st_t_ramp, 0.05f);
            float w_ho = S.p.st_omega;
            if (S.p.st_low > 0.05f && S.p.st_i_ref > 0.0f)
                w_ho *= S.p.st_low + (1.0f - S.p.st_low) * fm_min(fm_abs(S.cmd.iq_ref) / S.p.st_i_ref, 1.0f);
            float target = (float)S.dir * w_ho;
            if (S.ol_omega < target) S.ol_omega = fm_min(S.ol_omega + acc * dt, target);
            else S.ol_omega = fm_max(S.ol_omega - acc * dt, target);
            /* amortissement : si l'observateur (plausible) voit le rotor plus rapide que la rampe,
             * l'angle impose le rattrape au lieu de le laisser depasser puis recaler en arriere */
            if (S.oc.kind != OBS_RESC_BEMF && S.oc.kind != OBS_RESC_BEMF_LAMBDA && flux_ratio > 0.6f && flux_ratio < 1.5f &&
                S.obs.omega * (float)S.dir > S.ol_omega * (float)S.dir)
                S.ol_omega += (S.obs.omega - S.ol_omega) * fm_min(200.0f * dt, 1.0f);
            S.ol_iq = (float)S.dir * st_cur(S.cmd.iq_ref);
            /* Relais vers l'observateur quand il est accroche */
            float w = S.ol_omega;
            bool fast = fm_abs(w) >= 0.9f * w_ho;
            bool lock = flux_ratio > 0.75f && flux_ratio < 1.3f &&
                        fm_abs(S.obs.omega - w) < 0.25f * fm_abs(w) &&
                        S.obs.omega * w > 0.0f;
            S.lock_t = (fast && lock) ? S.lock_t + dt : 0.0f;
        }
        S.ol_theta = fm_wrap_pi(S.ol_theta + S.ol_omega * dt);
        S.theta = S.ol_theta;
        S.omega = S.ol_omega;
        S.id_ref = acim() ? S.p.acim_im : 0.0f;
        S.iq_ref = S.ol_iq;
        if (S.lock_t >= fm_max(S.p.st_t_hold, 0.05f)) {
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
            if (S.p.sl_omega_start < 0.95f * S.p.sl_omega) {
                /* melange lineaire entre sl_omega_start et sl_omega */
                float m = obs_ok ? fm_sat((w - S.p.sl_omega_start) / (S.p.sl_omega - S.p.sl_omega_start), 0.0f, 1.0f) : 0.0f;
                S.hall_mix = m;
                S.use_obs = m >= 1.0f;
            } else {
                if (!S.use_obs && w > 1.1f * S.p.sl_omega && obs_ok) S.use_obs = true;
                else if (S.use_obs && w < 0.9f * S.p.sl_omega) S.use_obs = false;
                S.hall_mix = S.use_obs ? 1.0f : 0.0f;
            }
            if (!sensor_ok) {
                S.hall_bad_t += dt;
                /* etat invalide (parasite) : l'angle Hall reste le dernier valide, comme VESC ;
                 * defaut si les capteurs restent invalides 0,1 s */
                if (!S.use_obs && S.hall_bad_t > 0.1f) { trip(RESC_FAULT_UNDERSPEED); goto output; }
            } else {
                S.hall_bad_t = 0.0f;
            }
        } else if (S.p.sensor_mode == FOC_SENSOR_HFI) {
            float w = fm_abs(S.obs.omega);
            float h = fm_max(S.p.hfi.hyst, 0.05f);
            bool obs_ok = flux_ratio > 0.7f && flux_ratio < 1.4f;
            if (S.use_hfi && ((w > (1.0f + h) * S.p.hfi.sl_omega && obs_ok) || S.cmd.brake)) {
                S.use_hfi = false;
                S.hfi_on = false;
            } else if (!S.use_hfi && w < (1.0f - h) * S.p.hfi.sl_omega && hfi_possible() && !S.cmd.brake) {
                /* retour a basse vitesse : l'injection repart de l'angle observe */
                hfi_reset(S.obs.theta, S.obs.omega);
                S.use_hfi = true;
                S.hfi_on = true;
                S.hfi_ovr_t = 0.0f;
            }
            /* l'observateur reste juste a la vitesse ou l'on repasse en injection : l'angle HFI y est
             * recale pendant foc_hfi_obs_ovr_sec */
            if (S.use_hfi && S.hfi_ovr_t < S.p.hfi.obs_ovr && obs_ok) {
                S.hfi_ovr_t += dt;
                S.hfi_theta = S.obs.theta;
                S.hfi_omega = S.obs.omega;
            }
            S.use_obs = !S.use_hfi;
        } else {
            S.use_obs = S.p.sensor_mode != FOC_SENSOR_ENCODER;
        }
        if (acim() && S.p.sensor_mode == FOC_SENSOR_ENCODER && S.p.enc.cpr >= 4) {
            /* commande vectorielle indirecte : angle rotor (codeur) + glissement integre */
            S.use_obs = false;
            S.slip_acc = fm_wrap_pi(S.slip_acc + S.ws * dt);
            S.theta = fm_wrap_pi(S.enc.theta + S.slip_acc);
            S.omega = S.enc.omega + S.ws;
        } else if (S.use_hfi) {
            S.theta = S.hfi_theta;
            S.omega = S.hfi_omega;
            S.hfi_on = true;
            S.hfi_v = S.p.hfi.v_run + (S.p.hfi.v_max - S.p.hfi.v_run) *
                      fm_min(fm_abs(S.iq_ref) / fm_max(S.p.hfi.i_ref, 1.0f), 1.0f);
        } else if (S.use_obs) {
            S.theta = S.obs.theta;
            S.omega = S.obs.omega;
        } else if (S.p.sensor_mode == FOC_SENSOR_HALL) {
            float m = S.hall_mix;
            if (m > 0.0f) {
                float s1, c1, s2, c2;
                fm_sincos(S.hall.theta, &s1, &c1);
                fm_sincos(S.obs.theta, &s2, &c2);
                S.theta = fm_atan2((1.0f - m) * s1 + m * s2, (1.0f - m) * c1 + m * c2);
            } else {
                S.theta = S.hall.theta;
            }
            S.omega = (fm_abs(S.obs.omega) > fm_abs(S.hall.omega) && flux_ratio > 0.7f && flux_ratio < 1.4f) ? S.obs.omega : S.hall.omega;
        } else {
            S.theta = S.enc.theta;
            S.omega = S.enc.omega;
            /* glissement du codeur : ecart de plus de 30 % avec l'observateur (fiable au-dessus de
             * deux fois la vitesse de sortie I/f) pendant 0.1 s */
            if (S.p.enc_slip_fault && !acim()) {
                float wo = fm_abs(S.obs.omega);
                bool bad = wo > 2.0f * S.p.st_omega_exit && flux_ratio > 0.7f && flux_ratio < 1.4f &&
                           fm_abs(S.enc.omega - S.obs.omega) > 0.3f * wo;
                S.slip_t = bad ? S.slip_t + dt : 0.0f;
                if (S.slip_t > 0.1f) { trip(RESC_FAULT_ENCODER_SLIP); goto output; }
            }
        }
        g_foc_telem.angle_from_sensor = !S.use_obs;
        g_foc_telem.angle_src = S.use_hfi ? 3 : S.use_obs ? 0 : S.p.sensor_mode == FOC_SENSOR_HALL ? 1 : 2;

        /* references internes : courant reduit a basse vitesse, defluxage, MTPA, fluage */
        float iq_cmd = S.cmd.iq_ref;
        if (S.p.st_dec < 0.999f && S.p.st_dec_omega > 0.0f)
            iq_cmd *= S.p.st_dec + (1.0f - S.p.st_dec) * fm_min(fm_abs(S.omega) / S.p.st_dec_omega, 1.0f);
        if (S.p.fw_i_max > 0.0f) {
            float rate = S.p.fw_i_max / fm_max(S.p.fw_ramp_time, 0.01f);
            /* foc_fw_backoff : quand iq depasse sa consigne (plus assez de tension sur q), le defluxage
             * maximal est reduit en proportion, ce qui evite l'emballement d -> q */
            float fw_max = S.p.fw_i_max;
            if (S.p.fw_backoff > 0.001f) {
                float e = fm_sign(S.omega) * (S.iq - S.iq_ref) / S.p.fw_i_max * S.p.fw_backoff;
                fw_max *= 1.0f - fm_sat(e, 0.0f, 1.0f);
            }
            S.fw_id += (S.duty_abs > S.p.fw_duty_start ? -rate : rate) * dt;
            S.fw_id = fm_sat(S.fw_id, -fw_max, 0.0f);
            iq_cmd *= 1.0f - (1.0f - S.p.fw_q_factor) * (-S.fw_id / S.p.fw_i_max);
        } else {
            S.fw_id = 0.0f;
        }
        float id_cmd = S.cmd.id_ref + S.fw_id;
        if (acim()) {
            /* flux nominal jusqu'a la vitesse de base, puis en 1/vitesse ; le module total
             * du courant reste dans la limite de la couche de controle */
            float wr = fm_abs(S.omega - S.ws);
            float kf = (S.p.acim_base_omega > 1.0f && wr > S.p.acim_base_omega) ? S.p.acim_base_omega / wr : 1.0f;
            id_cmd = S.p.acim_im * kf;
            float lim = fm_max(fm_abs(S.cmd.i_hi), fm_abs(S.cmd.i_lo));
            float iq_lim = fm_sqrt(fm_max(lim * lim - id_cmd * id_cmd, 0.0f));
            if (lim > 0.0f) iq_cmd = fm_sat_sym(iq_cmd, iq_lim);
            S.fw_id = 0.0f;
        } else if (S.p.mtpa_mode && S.oc.lq > 1.02f * S.oc.ld) {
            float dl = S.oc.lq - S.oc.ld;
            float iqm = S.p.mtpa_mode == 1 ? iq_cmd : S.iq_lpf;
            float lam = S.lam_eff;
            float id_mtpa = (lam - fm_sqrt(lam * lam + 8.0f * dl * dl * iqm * iqm)) / (4.0f * dl);
            id_cmd += id_mtpa;
        }
        /* modulation filtree et signes precedents (frein), suivis en permanence */
        FM_LPF(S.duty_sf, S.vq / fm_max(S.vbus_f * FM_1_SQRT3, 1.0f), 0.01f);
        bool flip = fm_sign(S.omega) != fm_sign(S.brk_w_prev) || fm_sign(S.vq) != fm_sign(S.brk_vq_prev);
        S.brk_w_prev = S.omega;
        S.brk_vq_prev = S.vq;
        if (S.cmd.brake) {
            /* Frein : courant oppose a la vitesse. Court-circuit (tensions nulles) a chaque changement
             * de signe de la vitesse ou de vq, ou modulation quasi nulle, tant que le courant de phase
             * reste sous la consigne ; au moins 10 periodes avant de reprendre la regulation. */
            float ib = fm_abs(iq_cmd);
            if ((flip || fm_abs(S.duty_sf) < 0.001f || S.brk_n < 10) && S.i_abs_f < ib) {
                S.brk_short = true;
                S.brk_n = 0;
            } else if (S.brk_n < 10) {
                S.brk_short = true;
                S.brk_n++;
            } else {
                if (S.brk_short) {
                    /* reprise : references et integrateurs sur l'etat actuel */
                    S.id_ref = 0.0f;
                    S.iq_ref = S.iq;
                    pi_preload(S.omega, true);
                }
                S.brk_short = false;
            }
            iq_cmd = -fm_sign(S.omega) * ib;
            id_cmd = 0.0f;
            S.fw_id = 0.0f;
        } else {
            S.brk_short = false;
            S.brk_n = 100;          /* entree en frein depuis la marche : regulation directe */
        }
        if (S.slew_t > 0.0f) S.slew_t -= dt; else S.slew = S.p.i_slew;
        float step = S.slew * dt;
        S.id_ref += fm_sat_sym(id_cmd - S.id_ref, step);
        /* la rampe douce apres le relais ne limite que la montee : une baisse de consigne (meme sens)
         * suit a la vitesse normale, sinon le moteur depasse la vitesse voulue (limite de duty hybride) */
        bool dec = iq_cmd * S.iq_ref >= 0.0f && fm_abs(iq_cmd) < fm_abs(S.iq_ref);
        S.iq_ref += fm_sat_sym(iq_cmd - S.iq_ref, dec ? fm_max(step, S.p.i_slew * dt) : step);

        ff_terms(S.omega, S.id_ref, S.iq_ref, &vd_ff, &vq_ff);

        /* Basse vitesse : retour en I/f si autorise, sinon roue libre */
        bool sl_mode = S.p.sensor_mode == FOC_SENSOR_SENSORLESS || S.p.sensor_mode == FOC_SENSOR_VSS;
        if (sl_mode && fm_abs(S.omega) < S.p.st_omega_exit && !S.cmd.brake) S.low_t += dt; else S.low_t = 0.0f;
        if (S.cmd.brake && sl_mode && fm_abs(S.omega) < S.p.st_omega_exit) S.brk_short = true;
        if (S.low_t > fm_max(S.p.st_hyst, 0.02f)) {
            if (S.cmd.allow_start && S.p.sensor_mode == FOC_SENSOR_VSS && hfi_possible() && fm_abs(S.omega) < 0.2f * S.p.st_omega_exit) {
                enter_hfi_start();
                vd_ff = vq_ff = 0.0f;
                goto output;
            } else if (S.cmd.allow_start) {
                S.dir = S.cmd.dir >= 0 ? 1 : -1;
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
        if (S.st == FOC_ST_RUN && S.use_obs && !S.brk_short) {
            if (flux_ratio < 0.5f || flux_ratio > 2.0f) S.flux_bad_t += dt; else S.flux_bad_t = 0.0f;
            if (S.flux_bad_t > 0.02f) {
                if (S.p.sensor_mode == FOC_SENSOR_HALL && sensor_ok) {
                    /* Hall : on reste sur les capteurs (pas de coupure), l'observateur sera repris
                     * quand son flux redevient coherent */
                    S.use_obs = false;
                    S.hall_mix = 0.0f;
                    S.flux_bad_t = 0.0f;
                } else {
                    trip(RESC_FAULT_UNDERSPEED);
                    goto output;
                }
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
    FM_LPF(S.iq_lpf, S.iq, 0.01f);
    /* Perte de synchronisation (doc 07 §7.7, indicateur I1) : sur une periode, le courant suit
     *   L di/dt = v - R i - e(w, i)   (repere rotor, e = FEM et termes croises)
     * avec v = moyenne des deux dernieres commandes (appliquees a cheval sur l'intervalle). Un angle faux
     * met la FEM reelle hors de l'axe attendu : l'ecart grandit avec la vitesse (175 A par periode a 90 deg
     * et 40000 ERPM sur 14 uH ; ecart normal : bruit et erreurs de modele, < 1 A sur le moteur simule, ~5 A
     * pour 1 V d'erreur sur 14 uH). Seuil : 5 % de la coupure (au moins 3 A), 4 periodes de suite apres
     * filtrage. Hors injection HFI, court-circuit de frein, surmodulation, et sur des courants mesures. */
    if (S.st == FOC_ST_RUN && S.br_p1 && S.br_p2 && S.pr_n >= 20u && S.pr_meas && S.pr_meas_prev && !S.hfi_on &&
        !S.brk_short && S.duty_abs < 0.98f && S.oc.ld > 1e-8f && S.oc.lq > 1e-8f) {
        float w = S.omega;
        float vd = 0.5f * (S.pr_vd1 + S.pr_vd2), vq = 0.5f * (S.pr_vq1 + S.pr_vq2);
        float pd = S.pr_id + dt / S.oc.ld * (vd - S.oc.r * S.pr_id + w * S.oc.lq * S.pr_iq);
        float pq = S.pr_iq + dt / S.oc.lq * (vq - S.oc.r * S.pr_iq - w * S.oc.ld * S.pr_id - w * S.lam_eff);
        float ed = S.id - pd, eq = S.iq - pq;
        float r = fm_sqrt(ed * ed + eq * eq) / fm_max(0.05f * S.p.i_abs_trip, 3.0f);
        FM_LPF(S.sync_r, r, 0.25f);
        if (S.sync_r > g_foc_telem.sync_resid_max) g_foc_telem.sync_resid_max = S.sync_r;
        S.sync_bad = S.sync_r > 1.0f ? (uint8_t)(S.sync_bad < 255 ? S.sync_bad + 1 : 255) : 0;
        if (S.sync_bad >= 4) { trip(RESC_FAULT_UNDERSPEED); goto output; }
    } else {
        S.sync_r *= 0.9f;
        S.sync_bad = 0;
    }
    if (S.st == FOC_ST_RUN && S.brk_short) {
        S.vd = S.vq = 0.0f;
        S.int_d = S.int_q = 0.0f;
        S.id_ref = S.iq_ref = 0.0f;
        S.duty_on = false;
    } else if (S.st == FOC_ST_RUN && S.cmd.duty_mode) {
        duty_loop(S.p.kp, S.p.ki, vd_ff, vq_ff, vmax);
    } else {
        if (S.duty_on) {
            /* sortie du mode duty : l'integrateur q reprend la tension appliquee */
            S.duty_on = false;
            S.int_q = S.vq - vq_ff;
        }
        current_loop(S.p.kp, S.p.ki, vd_ff, vq_ff, vmax);
    }
    S.duty_abs = fm_sqrt(S.vd * S.vd + S.vq * S.vq) / fm_max(S.vbus_f * FM_1_SQRT3, 1.0f);
    if (S.st == FOC_ST_RUN && S.hfi_on) hfi_inject(vmax);
    modulate_out();

output:
    if (!S.br) {
        S.duty[0] = S.duty[1] = S.duty[2] = 0.5f;
        S.duty_abs = 0.0f;
        if (S.st != FOC_ST_DETECT && S.st != FOC_ST_START && S.st != FOC_ST_RUN) {
            /* Hors pont actif, Park pour la telemetrie */
            float s, c;
            fm_sincos(S.obs.theta, &s, &c);
            S.id = c * S.i_a + s * S.i_b;
            S.iq = -s * S.i_a + c * S.i_b;
        }
    }

    /* ---- 8. Sorties et historique */
    if (!S.br) S.h_ok = false;
    for (int x = 0; x < 3; x++) {
        S.duty_p2[x] = S.duty_p1[x];
        S.duty_p1[x] = S.duty[x];
        S.duty_h_prev[x] = S.duty_h[x];
        S.duty_h[x] = S.duty[x];          /* seconde moitie identique, sauf mise a jour interpolee */
    }
    if (S.st == FOC_ST_RUN && S.br) {
        S.pr_vd2 = S.pr_vd1; S.pr_vq2 = S.pr_vq1;
        S.pr_vd1 = S.vd;     S.pr_vq1 = S.vq;
        S.pr_id = S.id;      S.pr_iq = S.iq;
        if (S.pr_n < 0xFFFFu) S.pr_n++;
    } else {
        S.pr_n = 0;
    }
    g_foc_telem.sync_resid = S.sync_r;
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
        /* courants : filtre foc_current_filter_const (affichage, limites, puissance) */
        float ki = fm_sat(S.p.i_filter, 0.001f, 1.0f);
        FM_LPF(S.id_f, S.id, ki);
        FM_LPF(S.iq_f, S.iq, ki);
        t->id = S.id_f;
        t->iq = S.iq_f;
        FM_LPF(t->vd, S.vd, k);
        FM_LPF(t->vq, S.vq, k);
        float imag = fm_sqrt(S.id * S.id + S.iq * S.iq);
        FM_LPF(t->i_abs, imag, ki);
        float p = 1.5f * (S.vd * S.id + S.vq * S.iq);
        FM_LPF(t->i_bus, S.br ? p / fm_max(S.vbus_f, 1.0f) : 0.0f, k);
        t->id_ref = S.id_ref;
        t->iq_ref = S.iq_ref;
        t->vbus = S.vbus_f;
        float vmag = fm_sqrt(S.vd * S.vd + S.vq * S.vq);
        float duty = S.br ? vmag / fm_max(S.vbus_f * FM_1_SQRT3, 1.0f) : 0.0f;
        t->duty = S.vq < 0.0f ? -duty : duty;
        /* pont coupe (roue libre) : rapport cyclique equivalent a la FEM, comme VESC -> suit le rotor */
        if (!S.br && !acim() && S.st != FOC_ST_FAULT)
            t->duty = fm_sat_sym(S.obs.omega * S.lam_eff / fm_max(S.vbus_f * FM_1_SQRT3, 1.0f), 1.0f);
        /* vitesse : derivee de l'angle de commande (PLL) ou PLL de l'observateur */
        if (S.st == FOC_ST_RUN || S.st == FOC_ST_START) {
            float e = fm_wrap_pi(S.theta - S.w_pll_theta);
            float w = S.w_pll_omega + S.oc.pll_kp * e;
            S.w_pll_theta = fm_wrap_pi(S.w_pll_theta + w * dt);
            S.w_pll_omega += S.oc.pll_ki * e * dt;
            S.omega_ctl = S.p.speed_source == 1 ? S.obs.omega : w;
        } else {
            S.w_pll_theta = S.theta;
            S.w_pll_omega = S.omega;
            S.omega_ctl = S.omega;
        }
        FM_LPF(t->omega, S.omega_ctl, 0.05f);
        FM_LPF(t->omega_fast, S.omega_ctl, 0.3f);
        t->omega_raw = S.omega_ctl;
        t->theta = S.theta;
        t->obs_omega = S.obs.omega;
        t->obs_theta = S.obs.theta;
        t->flux_ratio = S.obs.flux / S.lam_eff;
        t->lambda_est = S.lam_eff;
        t->fw_id = S.fw_id;
        t->state = S.st;
    }
}

/* Creux du compteur (ctl_mode 2) : meme tension, angle interpole pour la seconde moitie de la
 * commande (sommet -> creux suivant). Rien a ecrire hors marche, pont coupe ou injection HFI. */
RAMFUNC bool foc_isr_half(foc_pwm_t *out)
{
    if (!S.h_ok || !S.br) return false;
    float d0 = S.duty[0], d1 = S.duty[1], d2 = S.duty[2];
    modulate(S.theta_h);
    for (int x = 0; x < 3; x++) S.duty_h[x] = S.duty[x];
    S.duty[0] = d0; S.duty[1] = d1; S.duty[2] = d2;
    out->enable = true;
    out->duty[0] = S.duty_h[0];
    out->duty[1] = S.p.invert_dir ? S.duty_h[2] : S.duty_h[1];
    out->duty[2] = S.p.invert_dir ? S.duty_h[1] : S.duty_h[2];
    return true;
}

void foc_last_sample(foc_sample_t *s)
{
    for (int x = 0; x < 3; x++) {
        s->i[x] = S.i_ph[x];
        s->v[x] = S.vph_raw[x];
        s->duty[x] = S.duty_p1[x];
    }
    s->vzero = S.br_p1 ? 0.5f * S.vbus : (S.vph_raw[0] + S.vph_raw[1] + S.vph_raw[2]) * (1.0f / 3.0f);
    s->theta = S.theta;
    s->i_ab[0] = S.i_a;
    s->i_ab[1] = S.i_b;
    s->state = (uint8_t)S.st;
}
