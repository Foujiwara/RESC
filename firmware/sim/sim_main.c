/*
 * sim_main.c — Essais en boucle fermee du coeur FOC RESC sur un moteur simule.
 * Scenarios : calibration, identification R/L/lambda, demarrage I/f + relais
 * observateur, couple, freinage, inversion, reprise au vol.
 * Code de sortie non nul si un critere d'acceptation echoue.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "foc.h"
#include "sim_motor.h"
#include "ctrl.h"
#include "fault_codes.h"
#include "blackbox.h"

static sim_motor_t M;
static foc_adc_t ADC;
static foc_pwm_t OUT;
static int g_fail = 0;
static double g_trip_peak = 0;

void foc_hw_kill(void) { OUT.enable = false; }
void foc_hw_phase_filter(bool on) { M.pf_on = on; }

static void run(double seconds)
{
    long n = (long)(seconds * M.f_pwm);
    for (long k = 0; k < n; k++) {
        foc_isr(&ADC, &OUT);
        {
            /* creux du compteur : seconde moitie interpolee (ctl_mode 2) */
            foc_pwm_t h;
            M.hn_ok = foc_isr_half(&h);
            for (int x = 0; x < 3; x++) M.duty_hn[x] = h.duty[x];
        }
        sim_step(&M, &OUT, &ADC);
        double i[3];
        sim_phase_currents(&M, i);
        for (int x = 0; x < 3; x++) if (fabs(i[x]) > g_trip_peak) g_trip_peak = fabs(i[x]);
    }
}

static double ang_err(void)
{
    /* la telemetrie date de l'echantillon precedent : on recule l'angle vrai d'une periode */
    double e = g_foc_telem.theta - (sim_theta_e(&M) - sim_omega_e(&M) / M.f_pwm);
    while (e > M_PI) e -= 2 * M_PI;
    while (e < -M_PI) e += 2 * M_PI;
    return e;
}

/* Bruit de mesure : generateur reinitialise au debut de chaque groupe d'essais (graine = SIM_SEED x 100 +
 * groupe), pour qu'un essai ajoute dans un groupe ne change pas le bruit vu par les suivants */
static void sim_reseed(int group)
{
    unsigned base = getenv("SIM_SEED") ? (unsigned)atoi(getenv("SIM_SEED")) : 1u;
    srand(base * 100u + (unsigned)group);
}

static void check(const char *what, bool ok)
{
    printf("  [%s] %s\n", ok ? " OK " : "ECHEC", what);
    if (!ok) g_fail++;
}

static const char *st_name(foc_state_t s)
{
    static const char *n[] = { "INIT", "CALIB", "IDLE", "START", "RUN", "DETECT", "FAULT" };
    return n[s];
}

static void report(const char *tag)
{
    printf("  %-22s t=%6.2fs etat=%-6s erpm=%8.0f (vrai %8.0f) id=%6.2f iq=%6.2f err_ang=%6.1f deg flux=%.2f defaut=%u\n",
           tag, M.t, st_name(foc_state()), g_foc_telem.omega * 60 / (2 * M_PI), sim_omega_e(&M) * 60 / (2 * M_PI),
           g_foc_telem.id, g_foc_telem.iq, ang_err() * 180 / M_PI, g_foc_telem.flux_ratio, (unsigned)g_foc_telem.fault);
}

int main(void)
{
    /* --- Moteur type moyeu (valeurs representatives) */
    sim_reseed(0);
    blackbox_init();
    memset(&M, 0, sizeof(M));
    M.r = 0.06; M.ld = 90e-6; M.lq = 110e-6; M.lambda = 0.018; M.pp = 7;
    M.j = 0.004; M.b = 2e-4; M.t_load = 0.05;   /* a vide pour l'identification */
    M.vbus = 60.0; M.t_dead = 400e-9; M.f_pwm = 15000;
    /* --- Chaine de mesure de l'Ubox 85_200 : 3.44 V, 167 uOhm x20, polarite inversee */
    M.i_lsb = -(3.44 / 4095.0) / ((0.0005 / 3.0) * 20.0);
    M.v_lsb = (3.44 / 4095.0) * (150000.0 + 4700.0) / 4700.0;
    M.vph_lsb = M.v_lsb;
    M.off[0] = 2041; M.off[1] = 2055; M.off[2] = 2049;
    M.noise_lsb = 1.0;
    M.pf_tau = 100e-6;
    /* offsets reels des diviseurs de tension de phase (+0.35 / -0.25 / 0 V), corriges par la calibration
     * au demarrage ; sans elle l'observateur decroche en roue libre (SIM_NOVPHOFF=1 : mesure ideale) */
    if (!getenv("SIM_NOVPHOFF")) { M.vph_off[0] = 0.35; M.vph_off[1] = -0.25; M.vph_off[2] = 0.0; }
    sim_init(&M);

    foc_params_t p;
    memset(&p, 0, sizeof(p));
    p.f_pwm = 15000;
    p.i_scale = (float)M.i_lsb;
    p.v_scale = (float)M.v_lsb;
    p.vph_scale = (float)M.vph_lsb;
    p.cs_lowside = true;
    p.kp = 0.05f; p.ki = 30.0f;            /* remplaces apres identification */
    p.obs_gamma = 1e3f / (0.02f * 0.02f);
    p.pll_kp = 2000; p.pll_ki = 30000;
    p.dt_comp = 300e-9f;
    p.m_max = 0.95f;
    p.t_win = 3e-6f;
    p.delay_comp = 1.0f;
    p.decoupling = 3;                      /* anticipation croisee + FEM */
    p.st_dec = 1.0f;
    p.obs_gain_slow = 0.05f;               /* defaut de la reference */
    p.i_abs_trip = 120;
    p.vbus_trip_max = 90; p.vbus_trip_min = 10;
    p.st_i = 15; p.st_i_max = 60; p.st_t_align = 0.3f; p.st_t_ramp = 1.0f;
    p.st_omega = 1500 * 2 * 3.14159f / 60; p.st_omega_exit = 700 * 2 * 3.14159f / 60; p.st_timeout = 4;
    p.i_slew = 2000;
    if (getenv("SIM_CS")) p.cs_mode = (uint8_t)atoi(getenv("SIM_CS"));
    foc_init(&p);
    if (getenv("SIM_RESUME")) foc_set_resume_mode((uint8_t)atoi(getenv("SIM_RESUME")));

    printf("== 1. Calibration des offsets\n");
    /* defaut au demarrage (watchdog), avant la fin de la calibration : apres effacement, la calibration
     * est refaite et aucune commande (meme en boucle ouverte) n'active le pont avant */
    foc_raise_fault(RESC_FAULT_WATCHDOG_RESET);
    run(0.01);
    check("defaut au demarrage : etat FAULT", foc_state() == FOC_ST_FAULT);
    foc_clear_fault();
    g_foc_cmd.iq_ref = 5; g_foc_cmd.dir = 1; g_foc_cmd.openloop = 1; g_foc_cmd.run = 1;
    run(0.02);
    check("defaut efface sans calibration : retour en CALIB, pont coupe", foc_state() == FOC_ST_CALIB && !OUT.enable);
    g_foc_cmd.run = 0; g_foc_cmd.openloop = 0;
    run(0.4);
    check("calibration terminee, etat IDLE", foc_state() == FOC_ST_IDLE);
    printf("  offsets %.1f %.1f %.1f (vrais %.0f %.0f %.0f)\n", g_foc_telem.off_cs[0], g_foc_telem.off_cs[1],
           g_foc_telem.off_cs[2], M.off[0], M.off[1], M.off[2]);
    check("offsets a +/-2 LSB", fabs(g_foc_telem.off_cs[0] - M.off[0]) < 2 && fabs(g_foc_telem.off_cs[1] - M.off[1]) < 2);
#include "sim_lab.inc"

    printf("== 2. Identification R / Ld / Lq (temps mort partiel sous 4 A, comme de gros MOSFET)\n");
    M.i_soft = 4.0;
    foc_det_t d;
    memset(&d, 0, sizeof(d));
    d.kind = FOC_DET_RL;
    d.i_test = 15;      /* palier haut : 50 % de l_current_max (30 A) */
    check("detection R/L acceptee", foc_detect_start(&d));
    for (int k = 0; k < 600 && !foc_detect_status()->done; k++) run(0.05);
    const foc_det_t *ds = foc_detect_status();
    printf("  R = %.4f (vrai %.4f)  Ld = %.1f uH (vrai %.1f)  Lq = %.1f uH (vrai %.1f)  err=%d  paliers %.1f..%.1f A  "
           "chute %.3f V  temps mort %.3f V  alerte=%d\n",
           ds->r, M.r, ds->ld * 1e6, M.ld * 1e6, ds->lq * 1e6, M.lq * 1e6, ds->error, ds->r_i_low, ds->r_i_high,
           ds->r_v_high, ds->r_v_dt, ds->warn);
    check("R a +/-10 %", fabs(ds->r / M.r - 1) < 0.10);
    check("Ld a +/-20 %", fabs(ds->ld / M.ld - 1) < 0.20);
    check("Lq a +/-20 %", fabs(ds->lq / M.lq - 1) < 0.20);
    float r_id = ds->r, ld_id = ds->ld, lq_id = ds->lq;

    printf("== 3. Identification du flux (boucle ouverte, 12 A, duty 0.3)\n");
    run(0.5);
    memset(&d, 0, sizeof(d));
    d.kind = FOC_DET_LAMBDA;
    d.i_test = 12;
    d.r_known = r_id;
    d.l_known = 0.5f * (ld_id + lq_id);
    d.omega_test = 6000 * 2 * 3.14159f / 60;
    d.ramp_rate = 1800 * 2 * 3.14159f / 60;
    d.duty_target = 0.3f;
    check("detection lambda acceptee", foc_detect_start(&d));
    for (int k = 0; k < 400 && !foc_detect_status()->done; k++) run(0.05);
    printf("  lambda = %.5f Wb (vrai %.5f)  err=%d\n", ds->lambda, M.lambda, ds->error);
    check("lambda a +/-8 %", fabs(ds->lambda / M.lambda - 1) < 0.08);
    float lam_id = ds->lambda;
    if (getenv("SIM_VERBOSE")) printf("  lam_id=%g r_id=%g ld=%g lq=%g\n", lam_id, r_id, ld_id, lq_id);
    run(3.0);   /* laisse ralentir */

    /* Parametres identifies -> regulateurs (bande passante 1000 rad/s, comme VESC Tool) */
    p.r = r_id; p.ld = ld_id; p.lq = lq_id; p.lambda = lam_id;
    p.kp = 0.5f * (ld_id + lq_id) * 1000.0f;
    p.ki = r_id * 1000.0f;
    p.obs_gamma = 1e3f / (lam_id * lam_id);
    foc_set_params(&p);
    run(0.01);

    M.b = 0.06; M.t_load = 0.3;   /* charge realiste pour la suite */
    printf("== 4. Demarrage I/f + relais observateur, couple 20 A\n");
    g_trip_peak = 0;
    g_foc_cmd.iq_ref = 20; g_foc_cmd.id_ref = 0; g_foc_cmd.dir = 1; g_foc_cmd.allow_start = 1; g_foc_cmd.run = 1;
    if (getenv("SIM_VERBOSE")) {
        const foc_params_t *pp = foc_params();
        printf("  params r=%g ld=%g lq=%g lam=%g kp=%g ki=%g gam=%g dtc=%g\n",
               pp->r, pp->ld, pp->lq, pp->lambda, pp->kp, pp->ki, pp->obs_gamma, pp->dt_comp);
    }
    for (int k = 0; k < 8; k++) {
        run(0.25); report("demarrage");
        if (getenv("SIM_VERBOSE"))
            printf("     va=%.2f vb=%.2f ia=%.2f ib=%.2f vbus=%.2f\n", g_foc_telem.dbg_va, g_foc_telem.dbg_vb,
                   g_foc_telem.dbg_ia, g_foc_telem.dbg_ib, g_foc_telem.vbus);
    }
    check("en boucle fermee (RUN)", foc_state() == FOC_ST_RUN);
    check("erreur d'angle < 10 deg", fabs(ang_err()) < 10 * M_PI / 180);
    check("vitesse estimee a +/-5 %", fabs(g_foc_telem.omega / sim_omega_e(&M) - 1) < 0.05);
    check("iq suit la consigne (+/-1 A)", fabs(g_foc_telem.iq - 20) < 1.0);
    printf("  crete de phase pendant le demarrage : %.1f A\n", g_trip_peak);
    check("aucune surintensite (< 40 A)", g_trip_peak < 40);

    printf("== 5. Echelon de couple 20 -> 50 A, puis freinage -30 A\n");
    g_foc_cmd.iq_ref = 50;
    run(0.5); report("50 A");
    check("iq = 50 A (+/-2)", fabs(g_foc_telem.iq - 50) < 2);
    check("erreur d'angle < 10 deg a 50 A", fabs(ang_err()) < 10 * M_PI / 180);
    double w_before = sim_omega_e(&M);
    g_foc_cmd.iq_ref = -30;      /* frein : couple oppose a la rotation, sans redemarrage */
    g_foc_cmd.allow_start = 0;
    run(0.05); report("frein -30 A");
    check("decelere", sim_omega_e(&M) < w_before);
    check("iq = -30 A (+/-2)", fabs(g_foc_telem.iq + 30) < 2);
    for (int k = 0; k < 400 && foc_state() == FOC_ST_RUN; k++) {
        run(0.005);
        if (sim_omega_e(&M) < 1000 * 2 * M_PI / 60) g_foc_cmd.iq_ref = 0;   /* role de la couche de controle */
    }
    report("arret frein");
    check("pont coupe a basse vitesse en freinage", foc_state() == FOC_ST_IDLE);

    printf("== 6. Reprise au vol\n");
    g_foc_cmd.run = 0;
    M.b = 2e-4; M.t_load = 0.02;                     /* volant d'inertie : garde sa vitesse */
    M.omega_m = 4000.0 / 60 * 2 * M_PI / M.pp;     /* moteur lance a 4000 ERPM */
    run(0.3); report("roue libre");
    check("observateur suit le moteur en roue libre (+/-5 %)", fabs(g_foc_telem.omega / sim_omega_e(&M) - 1) < 0.05);
    g_trip_peak = 0;
    g_foc_cmd.iq_ref = 10; g_foc_cmd.allow_start = 1; g_foc_cmd.run = 1;
    run(0.3); report("reprise");
    check("RUN apres reprise", foc_state() == FOC_ST_RUN);
    check("pas de pic de courant (< 25 A)", g_trip_peak < 25);

    printf("== 7. Inversion du sens (-15 A depuis l'arret)\n");
    g_foc_cmd.run = 0;
    M.omega_m = 0; M.id = M.iq = 0;
    run(0.5);
    g_foc_cmd.iq_ref = -15; g_foc_cmd.dir = -1; g_foc_cmd.allow_start = 1; g_foc_cmd.run = 1;
    for (int k = 0; k < 6; k++) run(0.25);
    report("marche arriere");
    check("RUN en marche arriere", foc_state() == FOC_ST_RUN && sim_omega_e(&M) < 0);
    check("erreur d'angle < 10 deg", fabs(ang_err()) < 10 * M_PI / 180);

    printf("== 8. Mesure aberrante : derive de +1000 LSB sur la phase A (~250 A apparents)\n");
    M.off[0] += 1000;
    run(0.01); report("capteur en defaut");
    check("defaut ABS_OVER_CURRENT leve", g_foc_telem.fault == 4);
    check("pont coupe", !OUT.enable && foc_state() == FOC_ST_FAULT);
    M.off[0] -= 1000;
    g_foc_cmd.run = 0;
    run(0.05);
    foc_clear_fault();
    run(0.01);
    check("rearmement apres disparition du defaut", foc_state() == FOC_ST_IDLE);

    /* ------------------------------------------------------------------ */
    printf("== 9. Calibration manuelle des offsets\n");
    M.omega_m = 0; M.id = M.iq = 0;
    run(1.0);
    memset(&d, 0, sizeof(d));
    d.kind = FOC_DET_OFFSETS;
    check("calibration acceptee", foc_detect_start(&d));
    for (int k = 0; k < 40 && !foc_detect_status()->done; k++) run(0.05);
    printf("  offsets sous PWM %.1f %.1f %.1f (vrais %.0f %.0f %.0f), tensions %.3f %.3f %.3f V  err=%d\n",
           ds->off_cs[0], ds->off_cs[1], ds->off_cs[2], M.off[0], M.off[1], M.off[2],
           ds->off_vph[0], ds->off_vph[1], ds->off_vph[2], ds->error);
    check("offsets a +/-2 LSB", ds->error == 0 && fabs(ds->off_cs[0] - M.off[0]) < 2 && fabs(ds->off_cs[1] - M.off[1]) < 2 &&
          fabs(ds->off_cs[2] - M.off[2]) < 2);

    printf("== 10. Detection des sondes Hall (calage 37 deg), puis demarrage sur Hall a 40 A\n");
    M.hall_off = 37.0 * M_PI / 180;
    M.b = 2e-4; M.t_load = 0.05;            /* rotation libre pour la detection */
    memset(&d, 0, sizeof(d));
    d.kind = FOC_DET_HALL;
    d.i_test = 10;
    d.r_known = r_id; d.l_known = 0.5f * (ld_id + lq_id);
    check("detection Hall acceptee", foc_detect_start(&d));
    for (int k = 0; k < 400 && !foc_detect_status()->done; k++) run(0.05);
    double max_err = 0;
    for (int s = 1; s < 7; s++) {
        /* centre theorique de l'etat s */
        double best = 0; int n = 0; double sx = 0, sy = 0;
        for (int a = 0; a < 3600; a++) {
            double th = a * M_PI / 1800, h = th - M.hall_off;
            int st = (sin(h) > 0 ? 1 : 0) | (sin(h - 2 * M_PI / 3) > 0 ? 2 : 0) | (sin(h + 2 * M_PI / 3) > 0 ? 4 : 0);
            if (st == s) { sx += cos(th); sy += sin(th); n++; }
        }
        best = atan2(sy, sx);
        double e = ds->hall_angle[s] - best;
        while (e > M_PI) e -= 2 * M_PI;
        while (e < -M_PI) e += 2 * M_PI;
        if (fabs(e) > max_err) max_err = fabs(e);
        (void)n;
    }
    printf("  erreur max de la table : %.1f deg  err=%d\n", max_err * 180 / M_PI, ds->error);
    check("table Hall a +/-6 deg", ds->error == 0 && max_err < 6 * M_PI / 180);
    run(1.0);
    p.sensor_mode = FOC_SENSOR_HALL;
    for (int s = 0; s < 8; s++) { p.hall.angle[s] = ds->hall_angle[s]; p.hall.valid[s] = ds->hall_valid[s]; }
    p.hall.interp_omega = 500 * 2 * M_PI / 60;
    p.sl_omega = 3000 * 2 * M_PI / 60;
    foc_set_params(&p);
    M.omega_m = 0; M.b = 0.06; M.t_load = 1.5;   /* forte charge au demarrage */
    run(0.3);
    g_trip_peak = 0;
    g_foc_cmd.iq_ref = 40; g_foc_cmd.dir = 1; g_foc_cmd.allow_start = 1; g_foc_cmd.run = 1;
    run(0.05); report("hall 50 ms");
    check("marche immediate sur Hall (sans boucle ouverte)", foc_state() == FOC_ST_RUN && g_foc_telem.angle_from_sensor);
    check("le moteur demarre sous charge", sim_omega_e(&M) > 20);
    run(1.5); report("hall 1.5 s");
    check("bascule sur l'observateur au-dessus de 3000 ERPM", !g_foc_telem.angle_from_sensor && sim_omega_e(&M) * 60 / (2 * M_PI) > 3000);
    check("erreur d'angle < 10 deg", fabs(ang_err()) < 10 * M_PI / 180);
    check("pas de surintensite (< 60 A)", g_trip_peak < 60);
    g_foc_cmd.run = 0;
    run(2.0);

    printf("== 11. Detection d'un codeur 4096 fronts/tour (inverse, offset quelconque), puis marche\n");
    M.enc_cpr = 4096; M.enc_off = 1.234; M.enc_rev = true;
    M.b = 2e-4; M.t_load = 0.05;
    M.omega_m = 0;
    p.sensor_mode = FOC_SENSOR_SENSORLESS;
    p.enc.cpr = 4096;
    foc_set_params(&p);
    run(0.5);
    memset(&d, 0, sizeof(d));
    d.kind = FOC_DET_ENCODER;
    d.i_test = 10;
    d.r_known = r_id; d.l_known = 0.5f * (ld_id + lq_id);
    check("detection codeur acceptee", foc_detect_start(&d));
    for (int k = 0; k < 400 && !foc_detect_status()->done; k++) run(0.05);
    printf("  rapport %.2f (vrai %d), inverse %d, offset %.1f deg  err=%d\n", ds->enc_ratio, M.pp, ds->enc_inverted,
           ds->enc_offset * 180 / M_PI, ds->error);
    check("rapport = paires de poles", ds->error == 0 && fabs(ds->enc_ratio - M.pp) < 0.01);
    check("sens inverse detecte", ds->enc_inverted);
    run(1.0);
    p.sensor_mode = FOC_SENSOR_ENCODER;
    p.enc.ratio = ds->enc_ratio; p.enc.inverted = ds->enc_inverted; p.enc.offset = ds->enc_offset;
    foc_set_params(&p);
    M.b = 0.06; M.t_load = 1.0;
    run(0.2);
    g_foc_cmd.iq_ref = 30; g_foc_cmd.run = 1;
    run(1.0); report("codeur");
    check("RUN sur codeur", foc_state() == FOC_ST_RUN && g_foc_telem.angle_from_sensor);
    check("observateur et codeur concordent (< 5 deg)", fabs(ang_err()) < 5 * M_PI / 180);
    {
        /* angle codeur vs angle reel */
        double e = 0;
        foc_isr(&ADC, &OUT);
        e = g_foc_telem.iq;   /* iq suit la consigne si l'angle est juste */
        check("iq = 30 A sur codeur (+/-2)", fabs(e - 30) < 2);
    }

    /* ------------------------------------------------------------------ */
    printf("== 12. Couche de controle (1 kHz) : courant, frein, vitesse, timeout\n");
    g_foc_cmd.run = 0;
    p.sensor_mode = FOC_SENSOR_SENSORLESS;
    foc_set_params(&p);
    M.b = 0.02; M.t_load = 0.2; M.omega_m = 0;
    run(1.0);
    ctrl_limits_t L;
    memset(&L, 0, sizeof(L));
    L.i_max = 40; L.i_min = -40; L.i_max_scale = L.i_min_scale = 1;
    L.i_in_max = 60; L.i_in_min = -30; L.cc_min_current = 0.1f;
    L.erpm_max = 30000; L.erpm_min = -30000; L.erpm_start = 0.8f;
    L.batt_cut_start = 20; L.batt_cut_end = 15; L.regen_cut_start = 80; L.regen_cut_end = 85;
    L.tfet_start = 85; L.tfet_end = 100; L.duty_max = 0.95f; L.duty_min = 0.005f;
    L.s_kp = 0.004f; L.s_ki = 0.004f; L.s_kd = 0.0001f; L.s_kd_filter = 0.2f; L.s_min_erpm = 900; L.s_ramp_erpms = 25000;
    L.s_allow_braking = true; L.timeout_ms = 1000; L.start_erpm_exit = 700;
    ctrl_init(&L);
    ctrl_set_power_allowed(true);
    uint32_t ms = 0;
#define RUN_CTRL(sec) for (int _k = 0; _k < (int)((sec) * 1000); _k++) { ctrl_update(ms++, 0.001f); run(0.001); }
    ctrl_set_current(15);
    for (int k = 0; k < 15; k++) { ctrl_alive(); RUN_CTRL(0.1); }
    report("SET_CURRENT 15 A");
    check("demarrage et marche sur consigne de courant", foc_state() == FOC_ST_RUN && sim_omega_e(&M) > 0);
    ctrl_set_brake(20);
    for (int k = 0; k < 30 && foc_state() == FOC_ST_RUN; k++) { ctrl_alive(); RUN_CTRL(0.1); }
    report("SET_CURRENT_BRAKE 20 A");
    /* le frein tient le moteur en court-circuit jusqu'a l'arret (comportement VESC), sans le relancer */
    check("frein : arret sans repartir en arriere", fabs(sim_omega_e(&M)) < 50 && foc_state() == FOC_ST_RUN);
    ctrl_release();
    RUN_CTRL(0.02);
    ctrl_set_rpm(8000);
    for (int k = 0; k < 40; k++) { ctrl_alive(); RUN_CTRL(0.1); }
    report("SET_RPM 8000");
    check("regulation de vitesse a +/-5 %", fabs(sim_omega_e(&M) * 60 / (2 * M_PI) / 8000 - 1) < 0.05);
    /* duty : tension imposee, vitesse stable sans pompage, courant borne a i_max */
    ctrl_set_duty(0.3f);
    for (int k = 0; k < 30; k++) { ctrl_alive(); RUN_CTRL(0.1); }
    report("SET_DUTY 0.3");
    {
        double wmin = 1e9, wmax = -1e9, imax_seen = 0;
        for (int k = 0; k < 100; k++) {
            ctrl_alive(); RUN_CTRL(0.01);
            double w = sim_omega_e(&M);
            if (w < wmin) wmin = w;
            if (w > wmax) wmax = w;
        }
        printf("  vitesse %.0f..%.0f ERPM\n", wmin * 60 / (2 * M_PI), wmax * 60 / (2 * M_PI));
        check("duty 0.3 : rapport cyclique tenu (+/-0.01)", fabs(g_foc_telem.duty - 0.3) < 0.01);
        check("duty 0.3 : vitesse stable (ecart < 1 %)", (wmax - wmin) / wmax < 0.01);
        ctrl_set_duty(0.9f);
        for (int k = 0; k < 300; k++) {
            ctrl_alive(); RUN_CTRL(0.001);
            if (fabs(g_foc_telem.iq) > imax_seen) imax_seen = fabs(g_foc_telem.iq);
        }
        printf("  echelon 0.3 -> 0.9 : |iq| max %.1f A (limite %.0f A)\n", imax_seen, L.i_max);
        check("duty : courant borne a i_max (+10 %)", imax_seen < 1.1 * L.i_max);
        imax_seen = 0;
        ctrl_set_duty(0.1f);
        for (int k = 0; k < 300; k++) {
            ctrl_alive(); RUN_CTRL(0.001);
            if (-g_foc_telem.iq > imax_seen) imax_seen = -g_foc_telem.iq;
        }
        printf("  baisse 0.9 -> 0.1 : frein |iq| max %.1f A (limite %.0f A)\n", imax_seen, -L.i_min);
        check("duty : frein borne a i_min (+10 %)", imax_seen < 1.1 * -L.i_min);
        for (int k = 0; k < 30; k++) { ctrl_alive(); RUN_CTRL(0.1); }
        report("SET_DUTY 0.1");
        check("duty 0.1 : rapport cyclique tenu", foc_state() == FOC_ST_RUN && fabs(g_foc_telem.duty - 0.1) < 0.01);
    }
    RUN_CTRL(1.5);   /* plus aucune commande */
    report("apres timeout");
    check("timeout de commande : pont relache", ctrl_stats()->timeout && foc_state() != FOC_ST_RUN);

    printf("== 12b. Surintensites repetees : verrou apres 3 en 10 s, leve par 'clear'\n");
    {
        ctrl_release();
        int n_rearm = 0;
        for (int n = 0; n < 4; n++) {
            foc_raise_fault(RESC_FAULT_ABS_OVER_CURRENT);
            RUN_CTRL(0.01);
            for (int k = 0; k < 100 && foc_state() == FOC_ST_FAULT; k++) RUN_CTRL(0.01);     /* 1 s au plus */
            if (foc_state() != FOC_ST_FAULT) n_rearm++;
        }
        printf("  rearmements automatiques : %d sur 4 surintensites, verrou %s\n", n_rearm, ctrl_oc_locked() ? "pose" : "absent");
        check("2 rearmements automatiques puis verrou (defaut tenu)", n_rearm == 2 && ctrl_oc_locked() && foc_state() == FOC_ST_FAULT);
        ctrl_fault_unlock();
        foc_clear_fault();
        RUN_CTRL(0.05);
        check("'clear' : verrou leve, defaut efface", !ctrl_oc_locked() && foc_state() == FOC_ST_IDLE);
    }

    /* ------------------------------------------------------------------ */
    printf("== 13. Boucle ouverte commandee (foc_openloop, foc_openloop_duty)\n");
    run(1.0);
    ctrl_set_openloop(15, 2000);
    for (int k = 0; k < 30; k++) { ctrl_alive(); RUN_CTRL(0.1); }
    report("openloop 15 A 2000");
    check("boucle ouverte courant : vitesse imposee 2000 ERPM (+/-3 %)",
          foc_state() == FOC_ST_START && fabs(sim_omega_e(&M) * 60 / (2 * M_PI) / 2000 - 1) < 0.03);
    check("boucle ouverte courant : |i| = 15 A (+/-1.5)", fabs(sqrt(g_foc_telem.id * g_foc_telem.id + g_foc_telem.iq * g_foc_telem.iq) - 15) < 1.5);
    /* duty proche de la FEM (0.018 Wb x 314 rad/s = 5.7 V -> 0.16 sous 60 V) : sinon courant non borne */
    ctrl_set_openloop_duty(0.18f, 3000);
    for (int k = 0; k < 30; k++) { ctrl_alive(); RUN_CTRL(0.1); }
    report("openloop duty 0.18 3000");
    check("boucle ouverte duty : 3000 ERPM (+/-3 %)",
          foc_state() == FOC_ST_START && fabs(sim_omega_e(&M) * 60 / (2 * M_PI) / 3000 - 1) < 0.03);
    check("boucle ouverte duty : rapport cyclique 0.18", fabs(fabs(g_foc_telem.duty) - 0.18) < 0.01);
    ctrl_set_openloop(15, -2000);
    for (int k = 0; k < 50; k++) { ctrl_alive(); RUN_CTRL(0.1); }
    report("openloop 15 A -2000");
    check("boucle ouverte courant : inversion vers -2000 ERPM (+/-3 %)",
          foc_state() == FOC_ST_START && fabs(sim_omega_e(&M) * 60 / (2 * M_PI) / -2000 - 1) < 0.03);
    ctrl_release();
    RUN_CTRL(0.05);
    check("stop : pont coupe", foc_state() == FOC_ST_IDLE);

sim_reseed(1);
#include "sim_coast.inc"
sim_reseed(2);
#include "sim_user.inc"
sim_reseed(3);
#include "sim_features.inc"
sim_reseed(4);
#include "sim_vesc1.inc"
sim_reseed(5);
#include "sim_acim.inc"

    printf("\nCharge : %s  (%d echec(s))\n", g_fail ? "ECHEC" : "TOUS LES ESSAIS PASSENT", g_fail);
    return g_fail ? 1 : 0;
}
