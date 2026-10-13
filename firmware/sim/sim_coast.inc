/* sim_coast.inc — inclus dans sim_main.c : relacher les gaz en roue libre puis reprendre sur l'elan,
 * sur Hall, par la couche de controle (consigne relative envoyee chaque ms comme l'app ADC). */
{
    printf("== 27. Hall : relacher les gaz en roue libre, puis reprise des gaz sur l'elan\n");
    ctrl_release();
    RUN_CTRL(0.1);
    const foc_params_t p_saved = p;
    const ctrl_limits_t L_saved = L;
    const double b_saved = M.b, tl_saved = M.t_load;
    p.sensor_mode = FOC_SENSOR_HALL;
    p.hall.interp_omega = 500 * 2 * M_PI / 60;
    p.sl_omega = 3000 * 2 * M_PI / 60;
    p.sl_omega_start = p.sl_omega;
    foc_set_params(&p);
    L.i_max = 40; L.i_min = -40;
    ctrl_set_limits(&L);
    const double erpm_targets[3] = { 1500, 6000, 16000 };   /* 16000 : FEM ~ 85 % de la tension max */
    for (int sc = 0; sc < 3; sc++) {
        M.b = 2e-4; M.t_load = 0.05; M.omega_m = 0;
        RUN_CTRL(0.3);
        /* gaz jusqu'a la vitesse visee */
        for (int k = 0; k < 8000 && sim_omega_e(&M) * 60 / (2 * M_PI) < erpm_targets[sc]; k++) {
            ctrl_set_current_rel(sc == 2 ? 0.05f : 0.4f);
            RUN_CTRL(0.001);
        }
        report("gaz");
        /* relacher : consigne nulle une fois, puis l'app reste vivante (ctrl_alive) */
        ctrl_set_current_rel(0.0f);
        int transitions = 0, n_run = 0;
        foc_state_t prev = foc_state();
        double iq_coast_max = 0;
        for (int k = 0; k < 1500; k++) {
            ctrl_alive();
            RUN_CTRL(0.001);
            if (foc_state() != prev) { transitions++; prev = foc_state(); }
            if (foc_state() == FOC_ST_RUN) n_run++;
            if (fabs(g_foc_telem.iq) > iq_coast_max) iq_coast_max = fabs(g_foc_telem.iq);
        }
        report("roue libre 1.5 s");
        if (sc == 1) {
            /* decrochage de l'observateur en roue libre (vu sur la machine : 2914 ERPM affiches a 38000) */
            foc_dbg_obs_reset(0.0f, 0.08f * (float)sim_omega_e(&M));
            run(0.0005);
            printf("  observateur force a %.0f ERPM (vrai %.0f)\n", g_foc_telem.omega * 60 / (2 * M_PI),
                   sim_omega_e(&M) * 60 / (2 * M_PI));
        }
        printf("  roue libre : %d changements d'etat, %d ms en RUN, |iq| max %.1f A, defaut %u\n", transitions, n_run,
               iq_coast_max, (unsigned)g_foc_telem.fault);
        check("roue libre : pont relache et stable (pas de coupures / reprises)", transitions <= 1 && foc_state() == FOC_ST_IDLE);
        /* reprise des gaz sur l'elan : 0.4 x 40 A = 16 A demandes */
        g_trip_peak = 0;
        double iq_max = 0, iq_min = 0, err_max = 0;
        int t_run = -1;
        for (int k = 0; k < 300; k++) {
            ctrl_set_current_rel(sc == 2 ? 0.05f : 0.4f);
            RUN_CTRL(0.001);
            if (t_run < 0 && foc_state() == FOC_ST_RUN) t_run = k;
            if (foc_state() == FOC_ST_RUN) {
                if (g_foc_telem.iq > iq_max) iq_max = g_foc_telem.iq;
                if (g_foc_telem.iq < iq_min) iq_min = g_foc_telem.iq;
                if (fabs(ang_err()) > err_max) err_max = fabs(ang_err());
            }
        }
        report("reprise 0.3 s");
        printf("  reprise a %.0f ERPM : RUN apres %d ms, iq %.1f..%.1f A (demande 16), crete phase %.1f A, erreur d'angle max %.1f deg\n",
               erpm_targets[sc], t_run, iq_min, iq_max, g_trip_peak, err_max * 180 / M_PI);
        check("reprise : marche en moins de 5 ms", t_run >= 0 && t_run <= 5);
        check("reprise : pas de couple inverse ni de pic (iq -2..20 A)", iq_min > -2.0 && iq_max < 20.0);
        /* sous foc_hall_interp_erpm le Hall n'a que 6 positions : +/-30 deg physiques (+ limiteur) */
        check(sc == 0 ? "reprise sur Hall a basse vitesse : angle a +/-60 deg (resolution des sondes)"
                      : "reprise sur l'observateur : angle juste (< 15 deg)",
              err_max < (sc == 0 ? 60.0 : 15.0) * M_PI / 180);
        ctrl_release();
        RUN_CTRL(0.05);
    }
    /* Sensorless : l'observateur doit retrouver seul le moteur en roue libre (sans Hall) */
    printf("== 27b. Sensorless : observateur fausse en roue libre, il doit se recaler seul\n");
    p.sensor_mode = FOC_SENSOR_SENSORLESS;
    foc_set_params(&p);
    M.b = 2e-4; M.t_load = 0.05; M.omega_m = 0;
    RUN_CTRL(0.3);
    for (int k = 0; k < 8000 && sim_omega_e(&M) * 60 / (2 * M_PI) < 6000; k++) { ctrl_set_current_rel(0.4f); RUN_CTRL(0.001); }
    ctrl_set_current_rel(0.0f);
    for (int k = 0; k < 300; k++) { ctrl_alive(); RUN_CTRL(0.001); }
    foc_dbg_obs_reset(1.0f, 0.1f * (float)sim_omega_e(&M));
    int t_rec = -1;
    for (int k = 0; k < 500; k++) {
        ctrl_alive(); RUN_CTRL(0.001);
        if (t_rec < 0 && fabs(g_foc_telem.omega / sim_omega_e(&M) - 1) < 0.05 && fabs(ang_err()) < 10 * M_PI / 180) t_rec = k;
    }
    report("roue libre recalee");
    printf("  observateur fausse (10 %% de la vitesse) : recale seul apres %d ms\n", t_rec);
    check("sensorless : l'observateur retrouve seul vitesse et angle en roue libre (< 100 ms)", t_rec >= 0 && t_rec < 100);
    {
        g_trip_peak = 0;
        double iq_min = 0, iq_max = 0;
        for (int k = 0; k < 200; k++) {
            ctrl_set_current_rel(0.4f); RUN_CTRL(0.001);
            if (g_foc_telem.iq > iq_max) iq_max = g_foc_telem.iq;
            if (g_foc_telem.iq < iq_min) iq_min = g_foc_telem.iq;
        }
        printf("  reprise sensorless : iq %.1f..%.1f A, crete phase %.1f A\n", iq_min, iq_max, g_trip_peak);
        check("sensorless : reprise sans couple inverse ni pic", iq_min > -2.0 && iq_max < 20.0);
        ctrl_release(); RUN_CTRL(0.05);
    }

    printf("== 27d. Sensorless : reprise des gaz avec l'observateur decale de 90 deg (reprise validee)\n");
    {
        M.b = 2e-4; M.t_load = 0.05; M.omega_m = 0;
        RUN_CTRL(0.3);
        for (int k = 0; k < 8000 && sim_omega_e(&M) * 60 / (2 * M_PI) < 12000; k++) { ctrl_set_current_rel(0.6f); RUN_CTRL(0.001); }
        ctrl_set_current_rel(0.0f);
        for (int k = 0; k < 300; k++) { ctrl_alive(); RUN_CTRL(0.001); }
        double erpm0 = sim_omega_e(&M) * 60 / (2 * M_PI);
        /* decrochage simule juste avant la reprise : angle faux de 90 deg, vitesse juste */
        foc_dbg_obs_reset((float)(sim_theta_e(&M) + M_PI / 2), (float)sim_omega_e(&M));
        g_trip_peak = 0;
        int t_run = -1;
        double err_run = 0;
        for (int k = 0; k < 300; k++) {
            ctrl_set_current_rel(0.1f);                   /* 4 A */
            ctrl_update(ms++, 0.001f);
            for (int j = 0; j < 15; j++) {
                run(1.0 / M.f_pwm);
                if (t_run < 0 && foc_state() == FOC_ST_RUN) { t_run = k; err_run = ang_err() * 180 / M_PI; }
            }
        }
        printf("  reprise a %.0f ERPM, observateur decale de 90 deg : RUN apres %d ms (erreur d'angle %.1f deg), "
               "crete phase %.1f A, recalages %u\n", erpm0, t_run, err_run, g_trip_peak, (unsigned)g_foc_telem.obs_resync);
        check("reprise validee : marche en moins de 100 ms", t_run >= 0 && t_run < 100);
        check("reprise validee : angle juste a la reprise (< 15 deg)", t_run >= 0 && fabs(err_run) < 15);
        check("reprise validee : crete de phase < 15 A (demande 4 A)", g_trip_peak < 15.0);
        ctrl_release(); RUN_CTRL(0.05);
    }
    p = p_saved;
    foc_set_params(&p);
    L = L_saved;
    ctrl_set_limits(&L);
    M.b = b_saved; M.t_load = tl_saved;
    M.omega_m = 0;                   /* moteur arrete pour les essais suivants */
    RUN_CTRL(0.5);
}
