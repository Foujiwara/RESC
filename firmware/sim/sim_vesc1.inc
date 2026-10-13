/*
 * sim_vesc1.inc — Fonctions VESC reprises (inclus dans main() de sim_main.c, moteur a aimants) :
 * echantillonnage V0 + V7 interpole, V0 et V7 (shunts en ligne, boucle a 2 x f_pwm), frein en
 * court-circuit jusqu'a l'arret avec low-sides passants, frein a main, surmodulation, modes de
 * mesure des courants.
 */
{
    const foc_params_t p0 = p;

    /* ------------------------------------------------------------------ */
    printf("== 27. V0 + V7 interpole (foc_control_sample_mode 2) a haute vitesse electrique\n");
    double rip[2] = { 0, 0 }, emx[2] = { 0, 0 };
    for (int mode = 0; mode < 2; mode++) {
        p = p0;
        p.ctl_mode = mode ? 2 : 0;
        foc_set_params(&p);
        STOP_AT(0.5);
        M.b = 0.002; M.t_load = 0.1;
        START(25);
        run(3.0);
        g_foc_cmd.iq_ref = 15;
        run(0.5);
        double s1 = 0, s2 = 0; int n = 0;
        for (int k = 0; k < 3000; k++) {
            run(1.0 / 15000);
            double e = fabs(ang_err());
            if (e > emx[mode]) emx[mode] = e;
        }
        for (int k = 0; k < 3000; k++) {
            foc_isr(&ADC, &OUT);
            foc_pwm_t h; M.hn_ok = foc_isr_half(&h);
            for (int x = 0; x < 3; x++) M.duty_hn[x] = h.duty[x];
            sim_step(&M, &OUT, &ADC);
            double i[3]; sim_phase_currents(&M, i);
            (void)i;
            s1 += M.iq; s2 += M.iq * M.iq; n++;
        }
        rip[mode] = sqrt(fmax(s2 / n - (s1 / n) * (s1 / n), 0.0));
        printf("  %-14s etat=%s erpm=%.0f err max %.1f deg, ondulation iq rms %.2f A\n", mode ? "V0+V7 interp." : "V0",
               st_name(foc_state()), ERPM(sim_omega_e(&M)), emx[mode] * 180 / M_PI, rip[mode]);
    }
    check("V0 + V7 interpole : RUN au-dessus de 10000 ERPM", foc_state() == FOC_ST_RUN && ERPM(sim_omega_e(&M)) > 10000);
    check("V0 + V7 interpole : erreur d'angle < 10 deg", emx[1] < 10 * M_PI / 180);
    check("V0 + V7 interpole : ondulation de iq pas plus forte qu'en V0 (+10 %)", rip[1] < 1.1 * rip[0] + 0.05);

    /* ------------------------------------------------------------------ */
    printf("== 28. V0 et V7 (shunts en ligne) : boucle a 30 kHz pour une PWM a 15 kHz\n");
    p = p0;
    p.ctl_mode = 1;
    p.f_pwm = 30000;
    p.cs_lowside = false;
    p.delay_comp = 1.5f;
    STOP_AT(0.5);
    foc_set_params(&p);
    M.f_pwm = 30000; M.full_delay = true;
    M.b = 0.06; M.t_load = 0.3;
    run(0.01);
    START(20);
    {
        double emax = 0;
        for (int k = 0; k < 40; k++) {
            run(0.05);
            if (foc_state() == FOC_ST_RUN && k > 25 && fabs(ang_err()) > emax) emax = fabs(ang_err());
        }
        report("V0 et V7");
        check("V0 et V7 : RUN, erreur d'angle < 10 deg", foc_state() == FOC_ST_RUN && emax < 10 * M_PI / 180);
        check("V0 et V7 : iq suit la consigne (+/-1.5 A)", fabs(g_foc_telem.iq - 20) < 1.5);
    }
    STOP_AT(0.5);
    p = p0;
    foc_set_params(&p);
    M.f_pwm = 15000; M.full_delay = false;
    run(0.01);

    /* ------------------------------------------------------------------ */
    printf("== 29. Frein jusqu'a l'arret : court-circuit sous la vitesse utile, low-sides passants\n");
    for (int ls = 0; ls < 2; ls++) {
        p = p0;
        p.short_ls = ls != 0;
        foc_set_params(&p);
        STOP_AT(0.5);
        M.b = 0.002; M.t_load = 0.05;
        START(25);
        run(1.5);
        double w0 = ERPM(sim_omega_e(&M));
        g_foc_cmd.run = 0; run(0.002);
        ctrl_set_brake(20);
        double imax = 0;
        for (int k = 0; k < 60; k++) {
            ctrl_alive(); RUN_CTRL(0.05);
            if (fabs(g_foc_telem.iq) > imax) imax = fabs(g_foc_telem.iq);
            if (getenv("SIM_BRK")) printf("    t=%.2f st=%s erpm=%.0f iq=%.1f f=%u\n", M.t, st_name(foc_state()),
                                          ERPM(sim_omega_e(&M)), g_foc_telem.iq, (unsigned)g_foc_telem.fault);
        }
        bool zero = OUT.enable && OUT.duty[0] == 0.0f && OUT.duty[1] == 0.0f && OUT.duty[2] == 0.0f;
        bool half = OUT.enable && OUT.duty[0] == 0.5f && OUT.duty[1] == 0.5f && OUT.duty[2] == 0.5f;
        printf("  short_ls=%d : %.0f -> %.0f ERPM, etat %s (defaut %u), |iq| max %.1f A, sorties %.2f %.2f %.2f pont %d\n",
               ls, w0, ERPM(sim_omega_e(&M)), st_name(foc_state()), (unsigned)g_foc_telem.fault, imax, OUT.duty[0],
               OUT.duty[1], OUT.duty[2], OUT.enable);
        char t[120];
        snprintf(t, sizeof t, "frein (short_ls=%d) : arret (< 150 ERPM), pont tenu en court-circuit", ls);
        check(t, fabs(ERPM(sim_omega_e(&M))) < 150 && foc_state() == FOC_ST_RUN && (ls ? zero : half));
        snprintf(t, sizeof t, "frein (short_ls=%d) : courant de frein borne (< 24 A)", ls);
        check(t, imax < 24);
        ctrl_release(); RUN_CTRL(0.02);
    }
    check("relache : pont coupe", foc_state() == FOC_ST_IDLE);
    {
        /* depuis l'arret : le frein tient le moteur en court-circuit */
        STOP_AT(1.0);
        ctrl_set_brake(10);
        for (int k = 0; k < 5; k++) { ctrl_alive(); RUN_CTRL(0.02); }
        check("frein a l'arret : pont actif, phases en court-circuit", foc_state() == FOC_ST_RUN && OUT.enable &&
              OUT.duty[0] == 0.0f && OUT.duty[1] == 0.0f);
        ctrl_release(); RUN_CTRL(0.02);
    }

    /* ------------------------------------------------------------------ */
    printf("== 30. Frein a main (courant continu a angle fige)\n");
    p = p0;
    foc_set_params(&p);
    STOP_AT(2.0);
    M.b = 0.02; M.t_load = 0.0;
    {
        ctrl_set_handbrake(25);
        for (int k = 0; k < 10; k++) { ctrl_alive(); RUN_CTRL(0.05); }
        double th0 = M.theta;
        M.t_load = 0.4;                      /* couple exterieur */
        for (int k = 0; k < 20; k++) { ctrl_alive(); RUN_CTRL(0.05); }
        double moved = fabs(M.theta - th0) * M.pp * 180 / M_PI;
        printf("  deplacement sous 0.4 Nm : %.1f deg electriques, |i| %.1f A, etat %s\n", moved,
               sqrt(g_foc_telem.id * g_foc_telem.id + g_foc_telem.iq * g_foc_telem.iq), st_name(foc_state()));
        check("frein a main : rotor tenu (< 45 deg electriques sous 0.4 Nm)", moved < 45 && foc_state() == FOC_ST_START);
        {
            double mn = 1e9, mx = -1e9;
            for (int k = 0; k < 500; k++) {
                ctrl_alive(); RUN_CTRL(0.001);
                double v = ctrl_stats()->i_motor;
                if (v < mn) mn = v;
                if (v > mx) mx = v;
            }
            printf("  courant moteur affiche par l'outil pendant le frein a main : %.1f .. %.1f A\n", mn, mx);
            check("frein a main : courant affiche = +25 A (+/-3), sans changement de signe", mn > 22.0 && mx < 28.0);
        }
        check("frein a main : courant 25 A (+/-2)", fabs(sqrt(g_foc_telem.id * g_foc_telem.id + g_foc_telem.iq * g_foc_telem.iq) - 25) < 2);
        ctrl_release(); RUN_CTRL(0.02);
        M.t_load = 0.0;
    }

    /* ------------------------------------------------------------------ */
    printf("== 31. Surmodulation (foc_overmod_factor) : vitesse maximale a vide\n");
    double wtop[2];
    for (int k = 0; k < 2; k++) {
        p = p0;
        p.overmod = k ? 1.15f : 1.0f;
        foc_set_params(&p);
        STOP_AT(0.5);
        M.b = 0.0005; M.t_load = 0.05;
        START(40);
        run(4.0);
        wtop[k] = ERPM(sim_omega_e(&M));
        printf("  overmod %.2f : %.0f ERPM, etat %s\n", p.overmod, wtop[k], st_name(foc_state()));
    }
    check("surmodulation 1.15 : vitesse maximale plus elevee (> +5 %)", wtop[1] > 1.05 * wtop[0] && foc_state() == FOC_ST_RUN);

    /* ------------------------------------------------------------------ */
    printf("== 32. Modes de mesure des courants (foc_current_sample_mode) a 50 A\n");
    static const char *cname[3] = { "Longest Zero Time", "All Sensors Combined", "High Current" };
    for (int cm = 0; cm < 3; cm++) {
        p = p0;
        p.cs_mode = (uint8_t)cm;
        foc_set_params(&p);
        STOP_AT(0.5);
        M.b = 0.06; M.t_load = 0.5;
        START(20);
        run(1.5);
        g_foc_cmd.iq_ref = 50;
        run(0.5);
        printf("  %-21s etat=%s iq=%.1f err=%.1f deg\n", cname[cm], st_name(foc_state()), g_foc_telem.iq,
               ang_err() * 180 / M_PI);
        char t[96];
        snprintf(t, sizeof t, "%s : RUN, iq = 50 A (+/-2)", cname[cm]);
        check(t, foc_state() == FOC_ST_RUN && fabs(g_foc_telem.iq - 50) < 2);
    }

    /* ------------------------------------------------------------------ */
    printf("== 33. Courant batterie : regeneration (l_in_current_min) et limite du BMS\n");
    p = p0;
    foc_set_params(&p);
    {
        double ibr[2] = { 0, 0 }, ibd[2] = { 0, 0 };
        for (int lim = 0; lim < 2; lim++) {
            /* regeneration : lance a vide, puis frein 40 A */
            ctrl_limits_t L3 = L;
            if (lim) L3.i_in_min = -5.0f;
            ctrl_set_limits(&L3);
            STOP_AT(0.5);
            M.b = 0.0005; M.t_load = 0.02;
            ctrl_set_current(30);
            for (int k = 0; k < 25; k++) { ctrl_alive(); RUN_CTRL(0.1); }
            ctrl_set_brake(40);
            for (int k = 0; k < 300; k++) {
                ctrl_alive(); RUN_CTRL(0.001);
                if (k > 15 && g_foc_telem.i_bus < ibr[lim]) ibr[lim] = g_foc_telem.i_bus;
            }
            ctrl_release(); RUN_CTRL(0.05);
            ctrl_set_limits(&L);
            /* decharge : charge forte, 30 A, avec ou sans limite du BMS a 4 A */
            STOP_AT(0.5);
            M.b = 0.06; M.t_load = 0.5;
            if (lim) ctrl_set_bms_limits(-1e9f, 4.0f);
            ctrl_set_current(30);
            for (int k = 0; k < 30; k++) {
                ctrl_alive(); RUN_CTRL(0.1);
                if (k > 15 && g_foc_telem.i_bus > ibd[lim]) ibd[lim] = g_foc_telem.i_bus;
            }
            ctrl_set_bms_limits(-1e9f, 1e9f);
            ctrl_release(); RUN_CTRL(0.05);
        }
        printf("  frein 40 A : courant batterie min %.1f A sans limite, %.1f A limite a -5 A\n", ibr[0], ibr[1]);
        printf("  30 A en charge : courant batterie max %.1f A sans limite, %.1f A avec le BMS a 4 A\n", ibd[0], ibd[1]);
        check("regeneration : sans limite au-dela de -10 A, tenue a -5 A (+1 A) avec", ibr[0] < -10.0 && ibr[1] > -6.0 && ibr[1] < -3.0);
        check("BMS : sans limite au-dela de 6 A, tenu a 4 A (+1 A) avec", ibd[0] > 6.0 && ibd[1] < 5.0 && ibd[1] > 3.0);
    }

    /* ------------------------------------------------------------------ */
    printf("== 34. ADC hybride courant / rapport cyclique (ctrl_set_current_hybrid) : vitesse a vide selon les gaz\n");
    p = p0;
    foc_set_params(&p);
    {
        double wss[2] = { 0, 0 }, w_rip[2] = { 0, 0 };
        static const double thr[2] = { 0.5, 1.0 };
        for (int k = 0; k < 2; k++) {
            STOP_AT(0.5);
            M.b = 0.0005; M.t_load = 0.02;
            double wmin = 1e9, wmax = -1e9;
            for (int j = 0; j < 2000; j++) {          /* 4 s, une commande toutes les 2 ms (app ADC) */
                ctrl_set_current_hybrid((float)(thr[k] * L.i_max), (float)(thr[k] * L.duty_max), 0.15f);
                RUN_CTRL(0.002);
                if (getenv("SIM_HYB") && j % 50 == 0)
                    printf("    t=%.2f erpm=%.0f iq=%.2f duty=%.3f\n", j * 0.002, ERPM(sim_omega_e(&M)), g_foc_telem.iq,
                           g_foc_telem.duty);
                if (j >= 1500) {
                    double w = ERPM(sim_omega_e(&M));
                    if (w < wmin) wmin = w;
                    if (w > wmax) wmax = w;
                }
            }
            wss[k] = 0.5 * (wmin + wmax);
            w_rip[k] = wmax - wmin;
            char t[40];
            snprintf(t, sizeof t, "hybride gaz %.0f %%", thr[k] * 100);
            report(t);
            ctrl_release(); RUN_CTRL(0.05);
        }
        printf("  vitesse a vide : %.0f ERPM a mi-gaz, %.0f ERPM plein gaz (rapport %.2f), ondulation %.0f / %.0f ERPM\n",
               wss[0], wss[1], wss[0] / wss[1], w_rip[0], w_rip[1]);
        check("hybride : mi-gaz = environ mi-vitesse (rapport 0.4 a 0.6)", wss[0] / wss[1] > 0.4 && wss[0] / wss[1] < 0.6);
        check("hybride : vitesse stable (ondulation < 3 %)", w_rip[0] < 0.03 * wss[0] && w_rip[1] < 0.03 * wss[1]);
    }

    /* ------------------------------------------------------------------ */
    printf("== 35. Duty : relache en roue libre, affichage du duty, reprise au vol\n");
    p = p0;
    foc_set_params(&p);
    {
        STOP_AT(0.5);
        M.b = 0.0005; M.t_load = 0.02;
        ctrl_set_duty(0.4f);
        for (int k = 0; k < 20; k++) { ctrl_alive(); RUN_CTRL(0.1); }
        double w0 = ERPM(sim_omega_e(&M));
        ctrl_set_duty(0.0f);                          /* relache : roue libre */
        RUN_CTRL(0.05);
        bool idle = foc_state() == FOC_ST_IDLE;
        double dshow = g_foc_telem.duty, w1 = ERPM(sim_omega_e(&M));
        printf("  vitesse %.0f -> %.0f ERPM, etat %s, duty affiche %.3f\n", w0, w1, st_name(foc_state()), dshow);
        check("relache : pont coupe (roue libre, pas de frein)", idle && w1 > 0.8 * w0);
        check("relache : le duty affiche suit le rotor (FEM), pas 0", dshow > 0.5 * 0.4 && dshow < 1.3 * 0.4);
        ctrl_set_duty(0.4f);                          /* reprise pendant l'elan */
        double imin = 1e9, wmin = 1e9;
        for (int k = 0; k < 500; k++) {
            ctrl_alive(); RUN_CTRL(0.001);
            if (g_foc_telem.iq < imin) imin = g_foc_telem.iq;
            double w = ERPM(sim_omega_e(&M));
            if (w < wmin) wmin = w;
        }
        printf("  reprise : iq min %.1f A, vitesse min %.0f ERPM, etat %s\n", imin, wmin, st_name(foc_state()));
        check("reprise au vol en duty : pas de coup de frein (iq > -8 A)", imin > -8.0 && foc_state() == FOC_ST_RUN);
        check("reprise au vol en duty : pas de perte de vitesse (> 90 %)", wmin > 0.9 * w1);
        ctrl_release(); RUN_CTRL(0.05);
    }

    /* ------------------------------------------------------------------ */
    /* ------------------------------------------------------------------ */
    printf("== 36. Mode duty simple (Basic Duty Mode) : 0.8 -> 0.4, roue libre jusqu'a la consigne\n");
    {
        double imin_b[2], imax_b[2], t_hit[2], dfin[2];
        bool run_b[2] = { false, false };
        for (int basic = 0; basic < 2; basic++) {
            p = p0;
            p.duty_basic = basic;
            p.duty_dr_kp = 20; p.duty_dr_ki = 400;
            foc_set_params(&p);
            STOP_AT(0.5);
            M.b = 0.004; M.t_load = 0.02;
            ctrl_set_duty(0.8f);
            for (int k = 0; k < 40; k++) { ctrl_alive(); RUN_CTRL(0.1); }
            ctrl_set_duty(0.4f);
            imin_b[basic] = 1e9; imax_b[basic] = -1e9; t_hit[basic] = -1;
            for (int k = 0; k < 8000; k++) {
                ctrl_alive(); RUN_CTRL(0.001);
                if (k > 30 && k < 600) {
                    if (g_foc_telem.iq < imin_b[basic]) imin_b[basic] = g_foc_telem.iq;
                    if (g_foc_telem.iq > imax_b[basic]) imax_b[basic] = g_foc_telem.iq;
                }
                if (t_hit[basic] < 0 && fabs(g_foc_telem.duty) < 0.405) t_hit[basic] = k * 0.001;
            }
            dfin[basic] = g_foc_telem.duty;
            run_b[basic] = foc_state() == FOC_ST_RUN;
            printf("  basic=%d : iq pendant la descente %.1f .. %.1f A, consigne atteinte a %.2f s, duty final %.3f, etat %s\n",
                   basic, imin_b[basic], imax_b[basic], t_hit[basic], dfin[basic], st_name(foc_state()));
            ctrl_release(); RUN_CTRL(0.05);
        }
        check("duty simple : ni frein ni couple pendant la descente (|iq| < 4 A)", imin_b[1] > -4.0 && imax_b[1] < 4.0);
        check("duty normal : le regulateur de descente freine (iq < -10 A)", imin_b[0] < -10.0);
        check("duty simple : reprend la consigne 0.4 (+/- 0.02) en marche", fabs(dfin[1] - 0.4) < 0.02 && run_b[1]);
        check("duty simple : plus lent que le frein pour atteindre la consigne", t_hit[1] > t_hit[0]);
    }
    p = p0;
    foc_set_params(&p);

    /* ------------------------------------------------------------------ */
    printf("== 37. Mode courant (clavier) : relache puis reprise pendant l'elan\n");
    {
        static const double tacc[3] = { 0.15, 0.6, 3.0 };      /* duree de poussee : vitesse atteinte */
        static const double trel[4] = { 0.05, 0.2, 0.5, 1.5 };  /* duree de relache */
        int bad = 0;
        for (int a = 0; a < 3; a++) {
            for (int r = 0; r < 4; r++) {
                p = p0; foc_set_params(&p);
                STOP_AT(0.5);
                M.b = 0.002; M.t_load = 0.02;
                g_foc_telem.fault_count = 0;
                ctrl_set_current(30);
                for (int k = 0; k < (int)(tacc[a] * 1000); k++) { ctrl_alive(); RUN_CTRL(0.001); }
                double w0 = ERPM(sim_omega_e(&M));
                ctrl_set_current(0);
                for (int k = 0; k < (int)(trel[r] * 1000); k++) { ctrl_alive(); RUN_CTRL(0.001); }
                double w1 = ERPM(sim_omega_e(&M));
                ctrl_set_current(30);
                double imin = 1e9, wmin = 1e9;
                int idle_ms = 0;
                for (int k = 0; k < 1500; k++) {
                    ctrl_alive(); RUN_CTRL(0.001);
                    if (g_foc_telem.iq < imin) imin = g_foc_telem.iq;
                    double w = ERPM(sim_omega_e(&M));
                    if (w < wmin) wmin = w;
                    if (foc_state() != FOC_ST_RUN && foc_state() != FOC_ST_START) idle_ms++;
                }
                bool ok = g_foc_telem.fault_count == 0 && idle_ms < 50 && foc_state() == FOC_ST_RUN;
                printf("  poussee %.2f s (%.0f ERPM), relache %.2f s (%.0f ERPM) : iq min %.0f A, vitesse min %.0f, hors marche %d ms, defauts %u -> %s\n",
                       tacc[a], w0, trel[r], w1, imin, wmin, idle_ms, (unsigned)g_foc_telem.fault_count, ok ? "ok" : "PROBLEME");
                if (!ok) bad++;
                ctrl_release(); RUN_CTRL(0.05);
            }
        }
        check("courant clavier : relache puis reprise sans defaut ni temps mort (12 cas)", bad == 0);
    }

    p = p0;
    STOP_AT(0.5);
    foc_set_params(&p);
    run(0.01);
}
