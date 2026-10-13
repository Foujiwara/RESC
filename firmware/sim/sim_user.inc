/* sim_user.inc — inclus dans sim_main.c : moteur et reglages de l'utilisateur (Ubox 85_200, 72 V),
 * reprise des gaz en roue libre a 40000 ERPM avec une petite consigne (mesure du 12/10 : -10 / +18 A).
 * Variables : SIM_NOVPHERR=1 mesure ideale, SIM_VPHDRIFT=<V> derive d'offset, SIM_TDEAD=<s> temps mort reel,
 * SIM_DBG27C=1 journal des defauts.
 * Etat (doc 15 §15.7) : avec des diviseurs realistes, la reprise a 40000 ERPM fait ~47 A de crete pour 2 A ;
 * la 1.6 faisait de meme (46 / 20 / 33 / 48 A, sans defaut), l'ancien essai tournait a 29600 ERPM. */
{
    printf("== 27c. Moteur de l'utilisateur : reprise a 40000 ERPM, consigne 2 A\n");
    ctrl_release();
    RUN_CTRL(0.1);
    const sim_motor_t M_saved = M;
    const foc_params_t p_saved2 = p;
    const ctrl_limits_t L_saved2 = L;
    /* moteur reel (R mesuree par VESC 8 mOhm), carte : 72 V, temps mort 660 ns */
    M.r = 0.008; M.ld = 13.8e-6; M.lq = 26.1e-6; M.lambda = 0.00892; M.pp = 7;
    M.j = 0.0015; M.b = 1e-4; M.t_load = 0.02; M.vbus = 72.0; M.t_dead = getenv("SIM_TDEAD") ? atof(getenv("SIM_TDEAD")) : 660e-9;
    /* reglages lus sur le controleur : R 12.6 mOhm, compensation de temps mort 0.165 us */
    p.r = 0.01262f; p.ld = 13.8e-6f; p.lq = 26.1e-6f; p.lambda = 0.00892f;
    p.kp = 0.0199f; p.ki = 12.6f; p.dt_comp = 0.165e-6f;
    p.obs_gamma = 1.26e7f;
    p.sensor_mode = FOC_SENSOR_SENSORLESS;
    p.vbus_trip_max = 90;
    p.i_abs_trip = 300;                             /* coupure de la carte de l'utilisateur (defaut Ubox 85_200) */
    foc_set_params(&p);
    L.i_max = 200; L.i_min = -200;
    ctrl_set_limits(&L);
    /* diviseurs de tension de phase reels : gains +2 / -2 / +1 %. Ils sont la des le demarrage : la calibration
     * (moteur arrete, phases a Vbus/2) en corrige la part de mode commun, gain x Vbus/2 (sinon 0.7 V d'offset
     * jamais calibre, irrealiste). Reste une derive de 0.1 V depuis la calibration (SIM_VPHDRIFT). */
    if (!getenv("SIM_NOVPHERR")) {
        const double g[3] = { 0.02, -0.02, 0.01 };
        double drift = getenv("SIM_VPHDRIFT") ? atof(getenv("SIM_VPHDRIFT")) : 0.1;
        const double dr[3] = { drift, -drift, 0.5 * drift };
        for (int x = 0; x < 3; x++) {
            M.vph_gain[x] = g[x];
            M.vph_off[x] += -g[x] * M.vbus / 2 + dr[x];
        }
    }
    M.omega_m = 4000.0 / 60 * 2 * M_PI / M.pp;     /* lance depuis 4000 ERPM */
    RUN_CTRL(0.2);
    for (int k = 0; k < 6000 && sim_omega_e(&M) * 60 / (2 * M_PI) < 40000; k++) {
        ctrl_set_current_rel(0.1f);
        RUN_CTRL(0.001);
    }
    report("gaz");
    if (getenv("SIM_DBG27C")) {
        for (uint32_t k = g_foc_telem.fault_log_n > 8 ? g_foc_telem.fault_log_n - 8 : 0; k < g_foc_telem.fault_log_n; k++)
            printf("   defaut %u ligne %u a %.3f s\n", (unsigned)g_foc_telem.fault_log[k & 7].code,
                   (unsigned)g_foc_telem.fault_log[k & 7].line, g_foc_telem.fault_log[k & 7].ticks / M.f_pwm);
        printf("   resid max %.2f refus %u recalages %u\n", g_foc_telem.sync_resid_max, (unsigned)g_foc_telem.resume_refused,
               (unsigned)g_foc_telem.obs_resync);
    }
    /* la consigne seule n'atteint pas 40000 ERPM (charge) : le moteur y est amene pour les reprises */
    ctrl_set_current_rel(0.0f);
    M.omega_m = 40000.0 / 60 * 2 * M_PI / M.pp;
    for (int k = 0; k < 100; k++) { ctrl_alive(); RUN_CTRL(0.001); }
    report("40000 ERPM roue libre");
    if (getenv("SIM_DBG27C")) {
        for (uint32_t k = g_foc_telem.fault_log_n > 4 ? g_foc_telem.fault_log_n - 4 : 0; k < g_foc_telem.fault_log_n; k++)
            printf("   defaut %u ligne %u a %.3f s\n", (unsigned)g_foc_telem.fault_log[k & 7].code,
                   (unsigned)g_foc_telem.fault_log[k & 7].line, g_foc_telem.fault_log[k & 7].ticks / M.f_pwm);
    }
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
    double worst = 0, worst_peak = 0;
    int n_run = 0;
    for (int cyc = 0; cyc < 4; cyc++) {
        ctrl_set_current_rel(0.0f);
        M.omega_m = 40000.0 / 60 * 2 * M_PI / M.pp;   /* chaque reprise a 40000 ERPM */
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
        if (g_trip_peak > worst_peak) worst_peak = g_trip_peak;
        if (foc_state() == FOC_ST_RUN && iq_max > 1.0) n_run++;
    }
    if (getenv("SIM_DBG27C")) {
        for (uint32_t k = g_foc_telem.fault_log_n > 4 ? g_foc_telem.fault_log_n - 4 : 0; k < g_foc_telem.fault_log_n; k++)
            printf("   defaut %u ligne %u a %.3f s\n", (unsigned)g_foc_telem.fault_log[k & 7].code,
                   (unsigned)g_foc_telem.fault_log[k & 7].line, g_foc_telem.fault_log[k & 7].ticks / M.f_pwm);
        printf("   etat %d, verrou surintensite %d\n", foc_state(), ctrl_oc_locked());
    }
    check("moteur de l'utilisateur : les 4 reprises repassent en marche (courant etabli)", n_run == 4);
    check("moteur de l'utilisateur : reprise a 40000 ERPM sans a-coup (ecart < 4 A)", worst < 4.0);
    check("moteur de l'utilisateur : crete de phase < 12 A a chaque reprise (demande 2 A)", worst_peak < 12.0);
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
