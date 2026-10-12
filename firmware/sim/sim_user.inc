/* sim_user.inc — inclus dans sim_main.c : moteur et reglages de l'utilisateur (Ubox 85_200, 72 V),
 * reprise des gaz en roue libre a ~40000 ERPM avec une petite consigne (mesure du 12/10 : -10 / +18 A). */
{
    printf("== 27c. Moteur de l'utilisateur : reprise a 40000 ERPM, consigne 2 A\n");
    ctrl_release();
    RUN_CTRL(0.1);
    const sim_motor_t M_saved = M;
    const foc_params_t p_saved2 = p;
    const ctrl_limits_t L_saved2 = L;
    /* moteur reel (R mesuree par VESC 8 mOhm), carte : 72 V, temps mort 660 ns */
    M.r = 0.008; M.ld = 13.8e-6; M.lq = 26.1e-6; M.lambda = 0.00892; M.pp = 7;
    M.j = 0.0015; M.b = 1e-4; M.t_load = 0.02; M.vbus = 72.0; M.t_dead = 660e-9;
    /* reglages lus sur le controleur : R 12.6 mOhm, compensation de temps mort 0.165 us */
    p.r = 0.01262f; p.ld = 13.8e-6f; p.lq = 26.1e-6f; p.lambda = 0.00892f;
    p.kp = 0.0199f; p.ki = 12.6f; p.dt_comp = 0.165e-6f;
    p.obs_gamma = 1.26e7f;
    p.sensor_mode = FOC_SENSOR_SENSORLESS;
    p.vbus_trip_max = 90;
    foc_set_params(&p);
    L.i_max = 200; L.i_min = -200;
    ctrl_set_limits(&L);
    M.omega_m = 4000.0 / 60 * 2 * M_PI / M.pp;     /* lance depuis 4000 ERPM */
    RUN_CTRL(0.2);
    for (int k = 0; k < 6000 && sim_omega_e(&M) * 60 / (2 * M_PI) < 40000; k++) {
        ctrl_set_current_rel(0.1f);
        RUN_CTRL(0.001);
    }
    report("gaz 40000 ERPM");
    /* trace detaillee d'une reprise : 3 ms, chaque periode de boucle */
    {
        ctrl_set_current_rel(0.0f);
        for (int k = 0; k < 150; k++) { ctrl_alive(); RUN_CTRL(0.001); }
        ctrl_set_current_rel(0.01f);
        ctrl_update(ms++, 0.001f);
        printf("  trace reprise (k : etat id iq | err_ang)\n");
        for (int k = 0; k < 45; k++) {
            run(1.0 / M.f_pwm);
            if (k < 12 || k % 4 == 0)
                printf("   %2d : %d id=%7.2f iq=%7.2f | err %5.1f deg\n", k, foc_state(), g_foc_telem.id, g_foc_telem.iq,
                       ang_err() * 180 / M_PI);
        }
    }
    double worst = 0;
    for (int cyc = 0; cyc < 4; cyc++) {
        ctrl_set_current_rel(0.0f);
        for (int k = 0; k < 150; k++) { ctrl_alive(); RUN_CTRL(0.001); }
        double iq_min = 0, iq_max = 0;
        g_trip_peak = 0;
        for (int k = 0; k < 100; k++) {
            ctrl_set_current_rel(0.01f);                /* 2 A */
            RUN_CTRL(0.001);
            if (g_foc_telem.iq > iq_max) iq_max = g_foc_telem.iq;
            if (g_foc_telem.iq < iq_min) iq_min = g_foc_telem.iq;
        }
        printf("  reprise %d a %.0f ERPM : iq %.1f..%.1f A (demande 2 A), crete phase %.1f A\n", cyc + 1,
               sim_omega_e(&M) * 60 / (2 * M_PI), iq_min, iq_max, g_trip_peak);
        if (iq_max - 2.0 > worst) worst = iq_max - 2.0;
        if (-iq_min > worst) worst = -iq_min;
    }
    check("moteur de l'utilisateur : reprise a 40000 ERPM sans a-coup (ecart < 4 A)", worst < 4.0);
    ctrl_release();
    RUN_CTRL(0.05);
    M = M_saved;
    p = p_saved2;
    foc_set_params(&p);
    L = L_saved2;
    ctrl_set_limits(&L);
    M.omega_m = 0;
    RUN_CTRL(0.5);
}
