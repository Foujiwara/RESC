/*
 * sim_acim.inc — Moteur a induction (inclus a la fin de main() dans sim_main.c) :
 * identification R/L puis Lm/Lr/Rr, marche avec codeur (commande indirecte), marche sans
 * capteur (magnetisation, rampe de frequence, observateur), defluxage.
 * Moteur asynchrone basse tension : Rs 20 mOhm, Rr 20 mOhm, Lm 1 mH, fuites 40 uH + 40 uH,
 * 2 paires de poles, Tr = 52 ms ; courant magnetisant 40 A (psi_r 40 mWb).
 */
{
    printf("== 23. Moteur a induction : identification\n");
    memset(&M, 0, sizeof(M));
    M.acim = true;
    M.r = 0.02; M.lm = 1.0e-3; M.lr = 1.04e-3; M.rr = 0.02;
    M.ld = M.lq = 1.04e-3 + 0.04e-3 - M.lm * M.lm / M.lr;        /* sigma Ls */
    M.lambda = 0; M.pp = 2;
    M.j = 0.01; M.b = 1e-4; M.t_load = 0.0;
    M.vbus = 60.0; M.t_dead = 400e-9; M.f_pwm = 15000; M.i_soft = 0;
    M.i_lsb = -(3.44 / 4095.0) / ((0.0005 / 3.0) * 20.0);
    M.v_lsb = (3.44 / 4095.0) * (150000.0 + 4700.0) / 4700.0;
    M.vph_lsb = M.v_lsb;
    M.off[0] = 2041; M.off[1] = 2055; M.off[2] = 2049;
    M.noise_lsb = 1.0; M.pf_tau = 100e-6;
    M.enc_cpr = 4096; M.enc_off = 0.7; M.enc_rev = false;
    sim_init(&M);
    double sls = M.ld;

    foc_params_t q;
    memset(&q, 0, sizeof(q));
    q.f_pwm = 15000; q.i_scale = (float)M.i_lsb; q.v_scale = (float)M.v_lsb; q.vph_scale = (float)M.vph_lsb;
    q.cs_lowside = true; q.kp = 0.05f; q.ki = 30.0f; q.pll_kp = 2000; q.pll_ki = 30000;
    q.dt_comp = 300e-9f; q.m_max = 0.95f; q.t_win = 3e-6f; q.delay_comp = 1.0f; q.i_abs_trip = 200;
    q.vbus_trip_max = 90; q.vbus_trip_min = 10; q.st_i = 40; q.st_i_max = 60; q.st_t_align = 0.3f; q.st_t_ramp = 1.0f;
    q.st_omega = RAD(1500); q.st_omega_exit = RAD(700); q.st_timeout = 4; q.i_slew = 2000;
    q.decoupling = 3; q.st_dec = 1.0f; q.obs_gain_slow = 0.05f;
    q.motor_kind = 1;
    foc_init(&q);
    g_foc_cmd.run = 0; g_foc_cmd.openloop = 0; g_foc_cmd.duty_mode = 0;
    run(0.4);
    check("induction : calibration terminee", foc_state() == FOC_ST_IDLE);

    foc_det_t d2;
    memset(&d2, 0, sizeof(d2));
    d2.kind = FOC_DET_RL;
    d2.i_test = 40;
    check("induction : detection R/L acceptee", foc_detect_start(&d2));
    for (int k = 0; k < 600 && !foc_detect_status()->done; k++) run(0.05);
    const foc_det_t *r2 = foc_detect_status();
    printf("  R = %.2f mOhm (vrai %.2f)  L = %.1f / %.1f uH (sigma Ls vrai %.1f)  err=%d\n", r2->r * 1e3, M.r * 1e3,
           r2->ld * 1e6, r2->lq * 1e6, sls * 1e6, r2->error);
    check("induction : Rs a +/-10 %", fabs(r2->r / M.r - 1) < 0.10);
    check("induction : L mesuree = sigma Ls a +/-25 %", fabs(0.5 * (r2->ld + r2->lq) / sls - 1) < 0.25);
    float rs_id = r2->r, sls_id = 0.5f * (r2->ld + r2->lq);
    run(0.3);

    memset(&d2, 0, sizeof(d2));
    d2.kind = FOC_DET_ACIM;
    d2.i_test = 60;
    d2.r_known = rs_id;
    d2.l_known = sls_id;
    check("induction : identification Lm/Lr/Rr acceptee", foc_detect_start(&d2));
    for (int k = 0; k < 2000 && !foc_detect_status()->done; k++) {
        run(0.005);
        if (getenv("SIM_ACIM") && k >= 395 && k < 440)
            printf("    t=%.3f vd=%.4f id=%.2f iq=%.2f psi_r=%.2f mWb\n", M.t, g_foc_telem.vd, g_foc_telem.id, g_foc_telem.iq,
                   hypot(M.pra, M.prb) * 1e3);
    }
    printf("  Lm =%.3f mH (vrai %.3f)  Lr = %.3f mH (vrai %.3f)  Rr = %.2f mOhm (vrai %.2f)  Tr = %.1f ms (vrai %.1f)  err=%d\n",
           r2->acim_lm * 1e3, M.lm * 1e3, r2->acim_lr * 1e3, M.lr * 1e3, r2->acim_rr * 1e3, M.rr * 1e3,
           r2->acim_tr * 1e3, M.lr / M.rr * 1e3, r2->error);
    check("induction : Lm a +/-15 %", fabs(r2->acim_lm / M.lm - 1) < 0.15);
    check("induction : Tr a +/-15 %", fabs(r2->acim_tr / (M.lr / M.rr) - 1) < 0.15);
    check("induction : Rr a +/-20 %", fabs(r2->acim_rr / M.rr - 1) < 0.20);

    q.r = rs_id; q.ld = q.lq = sls_id;
    q.acim_lm = r2->acim_lm; q.acim_lr = r2->acim_lr; q.acim_rr = r2->acim_rr;
    q.acim_im = 40;
    q.kp = sls_id * 1000.0f; q.ki = rs_id * 1000.0f;
    {
        float lam = q.acim_lm * q.acim_lm / q.acim_lr * q.acim_im;
        q.lambda = lam;
        q.obs_gamma = 1e3f / (lam * lam);
    }
    run(0.5);

    /* angle de flux rotorique vrai pour comparer a l'angle de commande */
#define FLUX_ERR() (fmod(g_foc_telem.theta - atan2(M.prb, M.pra) + 9 * M_PI, 2 * M_PI) - M_PI)

    printf("== 24. Induction avec codeur (commande indirecte), 60 A de couple\n");
    q.sensor_mode = FOC_SENSOR_ENCODER;
    q.enc.cpr = 4096; q.enc.ratio = 2; q.enc.inverted = false; q.enc.offset = 0;
    foc_set_params(&q);
    run(0.02);
    M.t_load = 0.5; M.b = 0.002;
    g_foc_cmd.iq_ref = 60; g_foc_cmd.dir = 1; g_foc_cmd.allow_start = 1; g_foc_cmd.i_hi = 100; g_foc_cmd.i_lo = -100;
    g_foc_cmd.run = 1;
    {
        double emax = 0;
        for (int k = 0; k < 40; k++) {
            run(0.05);
            if (foc_state() == FOC_ST_RUN && k > 8 && fabs(FLUX_ERR()) > emax) emax = fabs(FLUX_ERR());
        }
        printf("  etat=%s erpm=%.0f psi_r=%.1f mWb (vrai %.1f) glissement=%.1f rad/s erreur flux max %.1f deg\n",
               st_name(foc_state()), ERPM(sim_omega_e(&M)), g_foc_telem.acim_psir * 1e3, hypot(M.pra, M.prb) * 1e3,
               g_foc_telem.acim_slip, emax * 180 / M_PI);
        check("codeur : RUN et le moteur accelere", foc_state() == FOC_ST_RUN && sim_omega_e(&M) > RAD(1000));
        check("codeur : angle de flux a +/-10 deg", emax < 10 * M_PI / 180);
        check("codeur : flux rotorique a +/-10 %", fabs(hypot(M.pra, M.prb) / (M.lm * 40) - 1) < 0.10);
    }

    printf("== 25. Induction sans capteur : magnetisation, rampe de frequence, observateur\n");
    g_foc_cmd.run = 0;
    run(1.0);
    M.omega_m = 0; M.ia_s = M.ib_s = 0; M.pra = M.prb = 0;
    q.sensor_mode = FOC_SENSOR_SENSORLESS;
    foc_set_params(&q);
    run(0.3);
    g_foc_cmd.iq_ref = 60; g_foc_cmd.run = 1;
    {
        double emax = 0;
        for (int k = 0; k < 60; k++) {
            run(0.05);
            if (foc_state() == FOC_ST_RUN && fabs(FLUX_ERR()) > emax) emax = fabs(FLUX_ERR());
        }
        printf("  etat=%s erpm=%.0f (vrai %.0f) erreur flux max en marche %.1f deg flux=%.2f\n", st_name(foc_state()),
               g_foc_telem.omega * 60 / (2 * M_PI), ERPM(sim_omega_e(&M)), emax * 180 / M_PI, g_foc_telem.flux_ratio);
        check("sans capteur : RUN sur l'observateur", foc_state() == FOC_ST_RUN);
        check("sans capteur : angle de flux a +/-12 deg", emax < 12 * M_PI / 180);
    }

    printf("== 26. Induction : defluxage au-dessus de 2000 ERPM\n");
    q.acim_base_omega = RAD(2000);
    g_foc_cmd.run = 0; run(0.02);
    foc_set_params(&q);
    M.t_load = 0.1;
    g_foc_cmd.run = 1;
    run(3.0);
    printf("  erpm=%.0f id=%.1f A (attendu ~%.1f) psi_r=%.1f mWb\n", ERPM(sim_omega_e(&M)), g_foc_telem.id,
           40.0 * 2000.0 / fabs(ERPM(sim_omega_e(&M))), g_foc_telem.acim_psir * 1e3);
    check("defluxage : courant magnetisant reduit en 1/vitesse", ERPM(sim_omega_e(&M)) > 2500 &&
          fabs(g_foc_telem.id - 40.0 * 2000.0 / ERPM(sim_omega_e(&M))) < 4.0);
    check("defluxage : marche sans defaut", foc_state() == FOC_ST_RUN);
    g_foc_cmd.run = 0; run(0.05);
}
