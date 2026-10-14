/* sim_lab.inc — inclus dans sim_main.c apres la calibration. SIM_LAB=1 : banc d'experiences sur la reprise au vol
 * du moteur de l'utilisateur, puis sortie. Une reprise = moteur lance a la vitesse voulue, roue libre 150 ms, consigne
 * pendant 100 ms ; defaut rearme entre deux reprises. Courants vrais du modele (non filtres).
 * Variables : SIM_ERPM=10000,20000,... SIM_VBUS=72 SIM_IREQ=2 (A) SIM_NREP=4 SIM_RESUME=0|1 SIM_TDEAD=<s>
 * SIM_NOVPHERR=1 SIM_VPHDRIFT=<V> SIM_BWK=<facteur sur kp, ki> SIM_DEC=<foc_cc_decoupling> SIM_DTCOMP=<s> */
if (getenv("SIM_LAB")) {
    uint32_t lms = 0;
#define LRUN(sec) for (int _k = 0; _k < (int)((sec) * 1000); _k++) { ctrl_update(lms++, 0.001f); run(0.001); }
    ctrl_limits_t LL;
    memset(&LL, 0, sizeof(LL));
    LL.i_max = 200; LL.i_min = -200; LL.i_max_scale = LL.i_min_scale = 1;
    LL.i_in_max = 300; LL.i_in_min = -300; LL.cc_min_current = 0.1f;
    LL.erpm_max = 100000; LL.erpm_min = -100000; LL.erpm_start = 0.8f;
    LL.batt_cut_start = 10; LL.batt_cut_end = 8; LL.regen_cut_start = 95; LL.regen_cut_end = 99;
    LL.tfet_start = 85; LL.tfet_end = 100; LL.duty_max = 0.95f; LL.duty_min = 0.005f;
    LL.timeout_ms = 1000;
    ctrl_init(&LL);
    ctrl_set_power_allowed(true);
    double vbus = getenv("SIM_VBUS") ? atof(getenv("SIM_VBUS")) : 72.0;
    M.r = 0.008; M.ld = 13.8e-6; M.lq = 26.1e-6; M.lambda = 0.00892; M.pp = 7;
    M.j = 0.0015; M.b = 1e-4; M.t_load = 0.02; M.vbus = vbus;
    M.t_dead = getenv("SIM_TDEAD") ? atof(getenv("SIM_TDEAD")) : 660e-9;
    double bwk = getenv("SIM_BWK") ? atof(getenv("SIM_BWK")) : 1.0;
    p.r = 0.01262f; p.ld = 13.8e-6f; p.lq = 26.1e-6f; p.lambda = 0.00892f;
    p.kp = (float)(0.0199 * bwk); p.ki = (float)(12.6 * bwk);
    p.dt_comp = getenv("SIM_DTCOMP") ? (float)atof(getenv("SIM_DTCOMP")) : 0.165e-6f;
    p.obs_gamma = 1.26e7f;
    p.obs_type = getenv("SIM_OBS") ? (uint8_t)atoi(getenv("SIM_OBS")) : p.obs_type;
    p.sensor_mode = FOC_SENSOR_SENSORLESS;
    p.decoupling = getenv("SIM_DEC") ? (uint8_t)atoi(getenv("SIM_DEC")) : 3;
    p.vbus_trip_max = 99; p.vbus_trip_min = 8;
    p.i_abs_trip = 300;
    p.m_max = 0.947f;
    foc_set_params(&p);
    if (!getenv("SIM_NOVPHERR")) {
        const double g[3] = { 0.02, -0.02, 0.01 };
        double drift = getenv("SIM_VPHDRIFT") ? atof(getenv("SIM_VPHDRIFT")) : 0.1;
        const double dr[3] = { drift, -drift, 0.5 * drift };
        for (int x = 0; x < 3; x++) { M.vph_gain[x] = g[x]; M.vph_off[x] += -g[x] * vbus / 2 + dr[x]; }
    }
    double ireq = getenv("SIM_IREQ") ? atof(getenv("SIM_IREQ")) : 2.0;
    int nrep = getenv("SIM_NREP") ? atoi(getenv("SIM_NREP")) : 4;
    const char *speeds = getenv("SIM_ERPM") ? getenv("SIM_ERPM") : "10000,20000,30000,40000";
    LRUN(0.3);
    printf("LAB obs=%d vbus=%.0f V  resume=%d  t_dead=%.0f ns  dt_comp=%.0f ns  bw x%.2f  decouplage=%d  diviseurs=%s  consigne=%.1f A\n",
           p.obs_type, vbus, foc_resume_mode(), M.t_dead * 1e9, p.dt_comp * 1e9, bwk, p.decoupling,
           getenv("SIM_NOVPHERR") ? "ideaux" : "reels", ireq);
    printf("  ERPM     | crete phase max / moy (A) | id min..max      | iq min..max      | defauts | refus | obs: angle, flux, vitesse\n");
    if (getenv("SIM_LAB_VPH")) {
        /* outils de banc : vph_check en roue libre a 10000 ERPM, puis enregistrement d'une reprise */
        ctrl_set_current(0.0f);
        M.omega_m = 10000.0 / 60 * 2 * M_PI / M.pp;
        for (int k = 0; k < 100; k++) { ctrl_alive(); LRUN(0.001); }
        foc_vph_check_start((uint32_t)(0.5 * M.f_pwm));
        for (int k = 0; k < 600; k++) { ctrl_alive(); LRUN(0.001); }
        foc_vph_check_t vr;
        if (foc_vph_check_result(&vr)) {
            double acm = (vr.ac[0] + vr.ac[1] + vr.ac[2]) / 3, dcm = (vr.dc[0] + vr.dc[1] + vr.dc[2]) / 3;
            printf("VPH n=%u erpm=%.0f\n", (unsigned)vr.n, vr.omega * 9.5493);
            for (int x = 0; x < 3; x++)
                printf("  phase %c : derive %+.3f V (injecte %+.3f), gain %+.2f %% (injecte %+.2f)\n", 'A' + x, vr.dc[x] - dcm,
                       M.vph_off[x] + M.vph_gain[x] * vbus / 2 - 0.35 * (x == 0) + 0.25 * (x == 1) -
                           ((M.vph_off[0] + M.vph_gain[0] * vbus / 2 - 0.35) + (M.vph_off[1] + M.vph_gain[1] * vbus / 2 + 0.25) +
                            (M.vph_off[2] + M.vph_gain[2] * vbus / 2)) / 3,
                       100 * (vr.ac[x] / acm - 1),
                       100 * ((1 + M.vph_gain[x]) / (1 + (M.vph_gain[0] + M.vph_gain[1] + M.vph_gain[2]) / 3) - 1));
            printf("  flux %.5f (vrai %.5f), vd %.3f vq %.3f\n", vr.emf / vr.omega, M.lambda, vr.vd, vr.vq);
        } else printf("VPH : pas de resultat\n");
        blackbox_release();
        for (int k = 0; k < 50; k++) { ctrl_alive(); LRUN(0.001); }
        blackbox_arm_start();
        for (int k = 0; k < 100; k++) { ctrl_set_current((float)ireq); LRUN(0.001); }
        uint32_t bl = 0;
        printf("ENREG evenement %u, fige %d, %d periodes dont %d avant\n", (unsigned)blackbox_fault(&bl, 0), blackbox_frozen(),
               blackbox_count(), blackbox_trigger_index());
        exit(0);
    }
    char buf[256];
    strncpy(buf, speeds, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    for (char *tok = strtok(buf, ","); tok; tok = strtok(0, ",")) {
        double erpm = atof(tok);
        double pk_max = 0, pk_sum = 0, idmin = 0, idmax = 0, iqmin = 0, iqmax = 0;
        int nf = 0;
        double eang = 0, eflux = 0, ew = 0;
        uint32_t ref0 = g_foc_telem.resume_refused;
        for (int r = 0; r < nrep; r++) {
            ctrl_set_current(0.0f);
            if (foc_state() == FOC_ST_FAULT) { ctrl_fault_unlock(); foc_clear_fault(); LRUN(0.05); }
            M.omega_m = erpm / 60 * 2 * M_PI / M.pp;
            for (int k = 0; k < 150; k++) { ctrl_alive(); LRUN(0.001); }
            {
                /* etat de l'observateur juste avant la reprise (pont coupe) */
                double e = g_foc_telem.obs_theta - (sim_theta_e(&M) - sim_omega_e(&M) / M.f_pwm);
                while (e > M_PI) e -= 2 * M_PI;
                while (e < -M_PI) e += 2 * M_PI;
                if (fabs(e) > fabs(eang)) eang = e;
                if (fabs(g_foc_telem.flux_ratio - 1) > fabs(eflux)) eflux = g_foc_telem.flux_ratio - 1;
                double we = g_foc_telem.obs_omega / sim_omega_e(&M) - 1;
                if (fabs(we) > fabs(ew)) ew = we;
            }
            g_trip_peak = 0;
            uint32_t f0 = g_foc_telem.fault_count;
            for (int k = 0; k < 100; k++) {
                ctrl_set_current((float)ireq);
                ctrl_update(lms++, 0.001f);
                for (int j = 0; j < 15; j++) {
                    run(1.0 / M.f_pwm);
                    if (M.id < idmin) idmin = M.id;
                    if (M.id > idmax) idmax = M.id;
                    if (M.iq < iqmin) iqmin = M.iq;
                    if (M.iq > iqmax) iqmax = M.iq;
                }
            }
            if (g_foc_telem.fault_count != f0) nf++;
            pk_sum += g_trip_peak;
            if (g_trip_peak > pk_max) pk_max = g_trip_peak;
        }
        printf("  %8.0f | %7.1f / %7.1f         | %6.1f..%6.1f   | %6.1f..%6.1f   | %d/%d     | %5u | %5.1f deg %+5.1f %% %+5.1f %%\n",
               erpm, pk_max, pk_sum / nrep, idmin, idmax, iqmin, iqmax, nf, nrep,
               (unsigned)(g_foc_telem.resume_refused - ref0), eang * 180 / M_PI, eflux * 100, ew * 100);
    }
    exit(0);
#undef LRUN
}
