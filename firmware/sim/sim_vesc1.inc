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

    p = p0;
    STOP_AT(0.5);
    foc_set_params(&p);
    run(0.01);
}
