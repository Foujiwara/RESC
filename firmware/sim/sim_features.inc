/*
 * sim_features.inc — Essais des options du coeur (inclus a la fin de main() dans sim_main.c) :
 * observateurs, filtres de phase, defluxage, MTPA, melange Hall, HFI, VSS, descente en duty.
 * Utilise p (parametres identifies), L, ms et RUN_CTRL definis plus haut.
 */
#define ERPM(w) ((w) * 60.0 / (2 * M_PI))
#define RAD(e)  ((float)((e) * 2 * M_PI / 60.0))

/* Arret complet, moteur immobile a l'angle electrique th */
#define STOP_AT(th) do { g_foc_cmd.run = 0; g_foc_cmd.openloop = 0; g_foc_cmd.duty_mode = 0; run(0.05); \
                         M.omega_m = 0; M.id = M.iq = 0; M.theta = (th) / M.pp; run(0.3); } while (0)
#define START(i) do { g_foc_cmd.iq_ref = (i); g_foc_cmd.id_ref = 0; g_foc_cmd.dir = (i) >= 0 ? 1 : -1; \
                      g_foc_cmd.allow_start = 1; g_foc_cmd.run = 1; } while (0)

{
    const foc_params_t p0 = p;
    M.b = 0.06; M.t_load = 0.3;

    /* ------------------------------------------------------------------ */
    printf("== 14. Observateurs (foc_observer_type 0..13), demarrage I/f puis 20 A\n");
    static const char *oname[14] = { "ORTEGA_ORIGINAL", "MXLEMMING", "ORTEGA_LAMBDA_COMP", "MXLEMMING_LAMBDA_COMP",
                                     "MXV", "MXV_LAMBDA_COMP", "MXV_LAMBDA_COMP_LIN", "RESC_ACTIVE_FLUX",
                                     "RESC_RADIAL", "RESC_ACTIVE_FLUX_LAMBDA", "RESC_RADIAL_LAMBDA", "RESC_BEMF",
                                     "RESC_BEMF_LAMBDA", "RESC_HYBRID_LAMBDA" };
    for (int ty = 0; ty < 14; ty++) {
        bool adapt = ty == 2 || ty == 3 || ty == 5 || ty == 6 || ty == 9 || ty == 10 || ty == 12 || ty == 13;
        p = p0;
        p.obs_type = (uint8_t)ty;
        if (adapt) p.lambda = 0.85f * p0.lambda;          /* lambda configure faux de 15 % */
        foc_set_params(&p);
        STOP_AT(1.0);
        START(20);
        double emax = 0;
        /* variantes adaptatives : lambda part faux de 15 %, convergence plus longue */
        int nk = adapt ? 70 : 40;
        for (int k = 0; k < nk; k++) {
            run(0.05);
            if (foc_state() == FOC_ST_RUN && k > nk - 20 && fabs(ang_err()) > emax) emax = fabs(ang_err());
            if (getenv("SIM_OBS") && atoi(getenv("SIM_OBS")) == ty) {
                double eo = fmod(g_foc_telem.obs_theta - sim_theta_e(&M) + 9 * M_PI, 2 * M_PI) - M_PI;
                printf("    t=%.2f st=%s w_obs=%.0f w=%.0f err_obs=%.1f flux=%.2f lam=%.4f\n", M.t, st_name(foc_state()),
                       ERPM(g_foc_telem.obs_omega), ERPM(sim_omega_e(&M)), eo * 180 / M_PI, g_foc_telem.flux_ratio,
                       g_foc_telem.lambda_est);
            }
        }
        printf("  %-20s etat=%-6s erpm=%6.0f (vrai %6.0f) err max %.1f deg lambda %.4f (vrai %.4f)\n", oname[ty],
               st_name(foc_state()), g_foc_telem.omega * 60 / (2 * M_PI), ERPM(sim_omega_e(&M)), emax * 180 / M_PI,
               g_foc_telem.lambda_est, M.lambda);
        char t[96];
        snprintf(t, sizeof t, "%s : RUN, erreur d'angle < 10 deg", oname[ty]);
        check(t, foc_state() == FOC_ST_RUN && emax < 10 * M_PI / 180);
        if (adapt) {
            snprintf(t, sizeof t, "%s : lambda reestime a +/-6 %% (configure -15 %%)", oname[ty]);
            check(t, fabs(g_foc_telem.lambda_est / M.lambda - 1) < 0.06);
        }
    }
    p = p0;
    foc_set_params(&p);

    /* ------------------------------------------------------------------ */
    printf("== 15. Filtre de phase actif sous 4000 ERPM (tensions mesurees pour l'observateur)\n");
    p.pf_enable = true; p.pf_no_fault = false; p.pf_omega = RAD(4000);
    foc_set_params(&p);
    STOP_AT(2.0);
    START(20);
    {
        double emax = 0;
        int n_pf = 0;
        for (int k = 0; k < 300; k++) {
            run(0.005);
            if (foc_state() == FOC_ST_RUN && M.pf_on) {
                n_pf++;
                if (fabs(ang_err()) > emax) emax = fabs(ang_err());
            }
        }
        report("filtre de phase");
        printf("  %d echantillons en marche filtre actif, erreur max %.1f deg\n", n_pf, emax * 180 / M_PI);
        check("filtre de phase : marche sans defaut", foc_state() == FOC_ST_RUN && g_foc_telem.fault == 0);
        check("filtre de phase : erreur d'angle < 10 deg filtre actif", n_pf > 20 && emax < 10 * M_PI / 180);
        check("filtre commute hors service au-dessus de 4000 ERPM", ERPM(sim_omega_e(&M)) < 4000 || !M.pf_on);
    }
    p = p0;
    foc_set_params(&p);

    /* ------------------------------------------------------------------ */
    printf("== 16. Defluxage (Vbus 30 V, 30 A d max)\n");
    M.vbus = 30; M.b = 0.004; M.t_load = 0.1;
    STOP_AT(0.5);
    START(25);
    run(3.0);
    double w_nofw = sim_omega_e(&M);
    report("sans defluxage");
    p.fw_i_max = 30; p.fw_duty_start = 0.85f; p.fw_ramp_time = 0.2f; p.fw_q_factor = 0.6f;
    g_foc_cmd.run = 0; run(0.02);
    foc_set_params(&p);
    START(25);
    run(3.0);
    report("avec defluxage");
    printf("  vitesse %0.f -> %.0f ERPM, id defluxage %.1f A\n", ERPM(w_nofw), ERPM(sim_omega_e(&M)), g_foc_telem.fw_id);
    check("defluxage : id negatif", g_foc_telem.fw_id < -3);
    check("defluxage : vitesse plus elevee (+10 %)", sim_omega_e(&M) > 1.10 * w_nofw);
    check("defluxage : marche sans defaut", foc_state() == FOC_ST_RUN);
    M.vbus = 60; M.b = 0.06; M.t_load = 0.3;
    p = p0;
    g_foc_cmd.run = 0; run(0.02);
    foc_set_params(&p);

    /* ------------------------------------------------------------------ */
    printf("== 17. MTPA (Ld < Lq)\n");
    p.mtpa_mode = 1;
    foc_set_params(&p);
    STOP_AT(0.7);
    START(40);
    run(2.5);
    {
        double dl = p.lq - p.ld, lam = p.lambda, iq = g_foc_telem.iq;
        double id_th = (lam - sqrt(lam * lam + 8 * dl * dl * iq * iq)) / (4 * dl);
        report("MTPA 40 A");
        printf("  id = %.2f A (attendu %.2f A)\n", g_foc_telem.id, id_th);
        check("MTPA : id conforme (+/-0.6 A)", foc_state() == FOC_ST_RUN && fabs(g_foc_telem.id - id_th) < 0.6);
    }
    p = p0;
    g_foc_cmd.run = 0; run(0.02);
    foc_set_params(&p);

    /* ------------------------------------------------------------------ */
    printf("== 18. Hall : melange vers l'observateur entre 2000 et 3000 ERPM\n");
    p.sensor_mode = FOC_SENSOR_HALL;
    p.sl_omega_start = RAD(2000); p.sl_omega = RAD(3000);
    foc_set_params(&p);
    STOP_AT(3.0);
    START(25);
    {
        double emax = 0;
        for (int k = 0; k < 300; k++) {
            run(0.005);
            double e = ERPM(sim_omega_e(&M));
            if (e > 2000 && e < 3500 && fabs(ang_err()) > emax) emax = fabs(ang_err());
            if (getenv("SIM_HALL") && e > 800 && e < 4000) printf("    erpm=%5.0f err=%5.1f deg mix=%d\n", e, ang_err() * 180 / M_PI, g_foc_telem.angle_from_sensor);
        }
        report("Hall -> observateur");
        printf("  erreur max pendant le melange %.1f deg\n", emax * 180 / M_PI);
        check("Hall : erreur d'angle < 10 deg pendant le melange (2000..3500 ERPM)", emax < 10 * M_PI / 180);
        check("Hall : observateur au-dessus de 3000 ERPM", !g_foc_telem.angle_from_sensor);
    }
    p = p0;
    g_foc_cmd.run = 0; run(0.02);
    foc_set_params(&p);

    /* ------------------------------------------------------------------ */
    printf("== 19. HFI : demarrage a l'arret depuis 4 positions, 20 A, relais observateur a 3000 ERPM\n");
    M.ld_sat = 0.004;                     /* Ld baisse de 12 %% a +30 A (saturation vers l'aimant) */
    p.sensor_mode = FOC_SENSOR_HFI;
    p.hfi.v_start = 10; p.hfi.v_run = 6; p.hfi.v_max = 8; p.hfi.gain = 0.3f; p.hfi.max_err = 0.3f;
    p.hfi.samples = 16; p.hfi.start_updates = 5; p.hfi.amb_mode = 2; p.hfi.amb_current = 30; p.hfi.amb_time = 0.015f;
    p.hfi.sl_omega = RAD(3000); p.hfi.hyst = 0.1f; p.hfi.i_ref = 40;
    foc_set_params(&p);
    {
        static const double a0[4] = { 0.3, 1.9, 3.5, 5.0 };
        for (int k = 0; k < 4; k++) {
            STOP_AT(a0[k]);
            START(20);
            double wmin = 0, emax = 0;
            bool run_seen = false;
            for (int j = 0; j < 100; j++) {
                run(0.002);
                if (sim_omega_e(&M) < wmin) wmin = sim_omega_e(&M);
                if (foc_state() == FOC_ST_RUN) {
                    run_seen = true;
                    if (fabs(ang_err()) > emax) emax = fabs(ang_err());
                }
            }
            printf("  depart %.1f rad : etat=%s err max %.1f deg, vitesse min %.0f ERPM\n", a0[k], st_name(foc_state()),
                   emax * 180 / M_PI, ERPM(wmin));
            if (getenv("SIM_HFI"))
                printf("    amp0 %.3f amp+ %.3f amp- %.3f theta_hfi(avant) %.2f vrai %.2f\n", g_foc_telem.dbg_amp0,
                       g_foc_telem.dbg_amp_p, g_foc_telem.dbg_amp_m, g_foc_telem.dbg_hfi_th, fmod(sim_theta_e(&M), 2 * M_PI));
            char t[80];
            snprintf(t, sizeof t, "HFI depart %.1f rad : marche, erreur < 20 deg, pas de recul", a0[k]);
            check(t, run_seen && emax < 20 * M_PI / 180 && ERPM(wmin) > -150);
        }
        run(1.5);
        report("HFI apres 1.5 s");
        check("HFI : relais vers l'observateur a haute vitesse", foc_state() == FOC_ST_RUN && !g_foc_telem.hfi_active);
        check("HFI : erreur d'angle < 10 deg (observateur)", fabs(ang_err()) < 10 * M_PI / 180);
    }

    /* ------------------------------------------------------------------ */
    printf("== 20. VSS : position par injection, I/f sans calage, observateur\n");
    p.sensor_mode = FOC_SENSOR_VSS;
    g_foc_cmd.run = 0; run(0.02);
    foc_set_params(&p);
    {
        STOP_AT(4.2);
        START(20);
        double wmin = 0;
        for (int j = 0; j < 300; j++) {
            run(0.005);
            if (sim_omega_e(&M) < wmin) wmin = sim_omega_e(&M);
            if (getenv("SIM_HFI") && j < 80)
                printf("    t=%.3f st=%s sub? erpm=%.0f err=%.1f iq=%.1f id=%.1f\n", M.t, st_name(foc_state()),
                       ERPM(sim_omega_e(&M)), ang_err() * 180 / M_PI, g_foc_telem.iq, g_foc_telem.id);
        }
        report("VSS");
        printf("  vitesse min pendant le demarrage %.0f ERPM\n", ERPM(wmin));
        if (getenv("SIM_HFI"))
            printf("    amp0 %.3f amp+ %.3f amp- %.3f theta_hfi(avant) %.2f\n", g_foc_telem.dbg_amp0,
                   g_foc_telem.dbg_amp_p, g_foc_telem.dbg_amp_m, g_foc_telem.dbg_hfi_th);
        check("VSS : marche sur l'observateur", foc_state() == FOC_ST_RUN && fabs(ang_err()) < 10 * M_PI / 180);
        /* recul selon l'erreur d'angle de l'injection, qui depend du bruit de mesure : 0 a -294 ERPM
         * sur les graines 1 a 6 (SIM_SEED) ; le demarrage aboutit dans tous les cas */
        check("VSS : recul au demarrage limite (> -350 ERPM)", ERPM(wmin) > -350);
    }
    M.ld_sat = 0;
    p = p0;
    g_foc_cmd.run = 0; run(0.02);
    foc_set_params(&p);

    /* ------------------------------------------------------------------ */
    printf("== 21. Duty : descente 0.6 -> 0.2 par le regulateur de frein (kp 20, ki 400)\n");
    p.duty_dr_kp = 20; p.duty_dr_ki = 400;
    foc_set_params(&p);
    STOP_AT(0.0);
    M.b = 0.01; M.t_load = 0.2;
    ctrl_set_duty(0.6f);
    for (int k = 0; k < 30; k++) { ctrl_alive(); RUN_CTRL(0.1); }
    report("duty 0.6");
    ctrl_set_duty(0.2f);
    {
        double imin = 0;
        for (int k = 0; k < 1500; k++) {
            ctrl_alive(); RUN_CTRL(0.001);
            if (g_foc_telem.iq < imin) imin = g_foc_telem.iq;
        }
        double wmin = 1e9, wmax = -1e9;
        for (int k = 0; k < 50; k++) {
            ctrl_alive(); RUN_CTRL(0.01);
            double w = sim_omega_e(&M);
            if (w < wmin) wmin = w;
            if (w > wmax) wmax = w;
        }
        report("duty 0.2");
        printf("  frein |iq| max %.1f A (limite %.0f A), vitesse %.0f..%.0f ERPM\n", -imin, -L.i_min, ERPM(wmin), ERPM(wmax));
        check("descente duty : frein borne a i_min (+10 %)", -imin < 1.1 * -L.i_min && imin < -2);
        check("descente duty : rapport cyclique atteint", fabs(g_foc_telem.duty - 0.2) < 0.01);
        check("descente duty : vitesse stable (ecart < 1 %)", (wmax - wmin) / wmax < 0.01);
    }
    ctrl_release();
    RUN_CTRL(0.05);
    p = p0;
    foc_set_params(&p);

    /* ------------------------------------------------------------------ */
    printf("== 22. Position sur codeur : PID position (kp 0.02, kd 0.0004), consigne +90 deg\n");
    p.sensor_mode = FOC_SENSOR_ENCODER;
    foc_set_params(&p);
    STOP_AT(0.0);
    M.b = 0.01; M.t_load = 0.05;
    {
        ctrl_limits_t L2 = L;
        L2.p_kp = 0.02f; L2.p_ki = 0.0f; L2.p_kd = 0.0004f; L2.p_kd_filter = 0.2f; L2.p_ang_div = 1.0f;
        L2.pole_pairs = (float)M.pp;
        ctrl_set_limits(&L2);
        for (int k = 0; k < 5; k++) { ctrl_alive(); RUN_CTRL(0.01); }
        float start = ctrl_stats()->pos_deg;
        double mech0 = M.theta;
        ctrl_set_pos(fmodf(start + 90.0f, 360.0f));
        for (int k = 0; k < 20; k++) { ctrl_alive(); RUN_CTRL(0.1); }
        double moved = (M.theta - mech0) * 180 / M_PI;
        double w1 = fabs(sim_omega_e(&M));
        printf("  rotation mecanique %.1f deg (consigne 90), vitesse residuelle %.0f ERPM\n", moved, ERPM(w1));
        check("position : +90 deg atteints (+/-3 deg)", fabs(moved - 90.0) < 3.0);
        check("position : arret sur la consigne (< 30 ERPM)", ERPM(w1) < 30);
        ctrl_release();
        RUN_CTRL(0.05);
        ctrl_set_limits(&L);
    }
    p = p0;
    g_foc_cmd.run = 0; run(0.02);
    foc_set_params(&p);
}
