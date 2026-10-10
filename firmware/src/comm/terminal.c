/*
 * terminal.c — Console texte RESC (page Terminal de VESC Tool ou tools/resc_cli.py).
 */
#include <stdlib.h>
#include <string.h>
#include "terminal.h"
#include "commands.h"
#include "detect_app.h"
#include "foc.h"
#include "ctrl.h"
#include "resc_conf.h"
#include "hw_config.h"
#include "platform.h"
#include "motor_hw.h"
#include "systime.h"
#include "wdg.h"
#include "board.h"
#include "flash.h"

#define P commands_printf

static const char *state_name(foc_state_t s)
{
    static const char *n[] = { "INIT", "CALIB", "IDLE", "START", "RUN", "DETECT", "FAULT" };
    return (unsigned)s < 7 ? n[s] : "?";
}

static const char *fault_name(uint32_t f)
{
    switch (f) {
    case 0: return "aucun";
    case 1: return "SURTENSION";
    case 2: return "SOUS-TENSION";
    case 3: return "DRIVER";
    case 4: return "SURINTENSITE";
    case 5: return "TEMP FET";
    case 6: return "TEMP MOTEUR";
    case 15: case 16: case 17: return "OFFSET COURANT";
    case 19: return "BREAK MATERIEL";
    case 32: return "DEMARRAGE / SYNCHRO";
    default: return "autre";
    }
}

static void cmd_status(void)
{
    const foc_telem_t *t = &g_foc_telem;
    const ctrl_stats_t *s = ctrl_stats();
    P("%s sur %s  etat=%s  defaut=%s (%u)", RESC_VERSION_STR, BOARD_NAME, state_name(t->state), fault_name(t->fault), (unsigned)t->fault);
    P("Vbus=%.2f V  ERPM=%.0f  duty=%.3f  Id=%.2f Iq=%.2f A  Ibat=%.2f A", (double)s->vbus, (double)s->erpm, (double)s->duty,
      (double)t->id, (double)t->iq, (double)s->i_in);
    P("T_FET=%.1f C  T_mot=%.1f C  flux=%.2f  limitation=%.2f  moteur %s", (double)s->tfet, (double)s->tmot,
      (double)t->flux_ratio, (double)s->i_scale_lim, resc_motor_configured() ? "identifie" : "NON IDENTIFIE");
}

static void cmd_rt(void)
{
    float f = motor_hw_freq();
    float load = (float)g_foc_telem.isr_cycles * f / 168e6f * 100.0f;
    P("f_PWM=%.0f Hz  ISR=%u cycles (%.2f us, %.1f %%)  max=%u  depassements=%u  reconstructions=%u",
      (double)f, (unsigned)g_foc_telem.isr_cycles, (double)(g_foc_telem.isr_cycles / 168.0f), (double)load,
      (unsigned)g_foc_telem.isr_cycles_max, (unsigned)g_foc_telem.n_overrun, (unsigned)g_foc_telem.n_predicted);
}

static void cmd_conf(void)
{
    const foc_params_t *p = foc_params();
    const ctrl_limits_t *l = ctrl_limits();
    P("Moteur : R=%.2f mOhm Ld=%.1f uH Lq=%.1f uH lambda=%.3f mWb", (double)(p->r * 1e3f), (double)(p->ld * 1e6f),
      (double)(p->lq * 1e6f), (double)(p->lambda * 1e3f));
    P("Courant : kp=%.4f ki=%.1f  obs gamma=%.3g  PLL %.0f/%.0f", (double)p->kp, (double)p->ki, (double)p->obs_gamma,
      (double)p->pll_kp, (double)p->pll_ki);
    P("Limites effectives : I=%.1f/%.1f A  Ibat=%.1f/%.1f A  Iabs=%.1f A  (plafond carte %.0f %%)", (double)l->i_min,
      (double)l->i_max, (double)l->i_in_min, (double)l->i_in_max, (double)p->i_abs_trip, (double)(resc_current_cap_frac() * 100.0f));
    P("Demarrage : I=%.1f A, %.0f ERPM, rampe %.2f s ; Vbus %.1f-%.1f V", (double)p->st_i,
      (double)(p->st_omega * 60.0f / 6.2831853f), (double)p->st_t_ramp, (double)p->vbus_trip_min, (double)p->vbus_trip_max);
}

/* ------------------------------------------------ parametres accessibles au terminal */
typedef struct { const char *name; float *f; uint8_t *u8; uint32_t *u32; const char *unit; } param_t;

static const param_t *params(int *n)
{
    static param_t t[24];
    static int cnt;
    if (!cnt) {
        vesc_mcconf_t *m = &g_mcconf;
        vesc_appconf_t *a = &g_appconf;
        param_t p[] = {
            { "l_current_max", &m->l_current_max, 0, 0, "A" },
            { "l_current_min", &m->l_current_min, 0, 0, "A" },
            { "l_in_current_max", &m->l_in_current_max, 0, 0, "A" },
            { "l_in_current_min", &m->l_in_current_min, 0, 0, "A" },
            { "l_abs_current_max", &m->l_abs_current_max, 0, 0, "A" },
            { "l_min_vin", &m->l_min_vin, 0, 0, "V" },
            { "l_max_vin", &m->l_max_vin, 0, 0, "V" },
            { "l_max_erpm", &m->l_max_erpm, 0, 0, "ERPM" },
            { "foc_motor_r", &m->foc_motor_r, 0, 0, "Ohm" },
            { "foc_motor_l", &m->foc_motor_l, 0, 0, "H" },
            { "foc_motor_ld_lq_diff", &m->foc_motor_ld_lq_diff, 0, 0, "H" },
            { "foc_motor_flux_linkage", &m->foc_motor_flux_linkage, 0, 0, "Wb" },
            { "foc_current_kp", &m->foc_current_kp, 0, 0, "V/A" },
            { "foc_current_ki", &m->foc_current_ki, 0, 0, "V/As" },
            { "foc_observer_gain", &m->foc_observer_gain, 0, 0, "" },
            { "foc_f_zv", &m->foc_f_zv, 0, 0, "Hz (= 2 f_PWM)" },
            { "foc_openloop_rpm", &m->foc_openloop_rpm, 0, 0, "ERPM" },
            { "foc_sl_erpm", &m->foc_sl_erpm, 0, 0, "ERPM" },
            { "foc_sensor_mode", 0, &m->foc_sensor_mode, 0, "0 sensorless, 1 codeur, 2 Hall" },
            { "m_encoder_counts", 0, 0, &m->m_encoder_counts, "fronts/tour" },
            { "si_motor_poles", &m->si_motor_poles, 0, 0, "poles" },
            { "controller_id", &a->controller_id, 0, 0, "" },
            { "timeout_msec", 0, 0, &a->timeout_msec, "ms" },
        };
        cnt = (int)(sizeof(p) / sizeof(p[0]));
        for (int i = 0; i < cnt; i++) t[i] = p[i];
    }
    *n = cnt;
    return t;
}

static void show_param(const param_t *p)
{
    if (p->f) P("  %-24s = %.6g %s", p->name, (double)*p->f, p->unit);
    else if (p->u8) P("  %-24s = %u %s", p->name, (unsigned)*p->u8, p->unit);
    else P("  %-24s = %lu %s", p->name, (unsigned long)*p->u32, p->unit);
}

static void cmd_get_set(int argc, char **argv)
{
    int n;
    const param_t *t = params(&n);
    if (!strcmp(argv[0], "list")) { for (int i = 0; i < n; i++) show_param(&t[i]); return; }
    if (argc < 2) { P("Usage : get <nom> | set <nom> <valeur>"); return; }
    for (int i = 0; i < n; i++) {
        if (strcmp(argv[1], t[i].name)) continue;
        if (!strcmp(argv[0], "set")) {
            if (argc < 3) { P("Valeur manquante"); return; }
            if (foc_state() == FOC_ST_RUN || foc_state() == FOC_ST_START) { P("Refuse : moteur en marche"); return; }
            double v = atof(argv[2]);
            if (t[i].f) *t[i].f = (float)v;
            else if (t[i].u8) *t[i].u8 = (uint8_t)v;
            else *t[i].u32 = (uint32_t)v;
            resc_conf_clamp_mc(&g_mcconf);
            resc_conf_clamp_app(&g_appconf);
            resc_conf_apply();
        }
        show_param(&t[i]);
        return;
    }
    P("Parametre inconnu : %s (taper list)", argv[1]);
}

/* Gains a partir de R, L, lambda, sur demande : bande passante de courant en rad/s */
static void cmd_foc_gains(int argc, char **argv)
{
    float bw = argc >= 2 ? (float)atof(argv[1]) : 1000.0f;
    vesc_mcconf_t *m = &g_mcconf;
    if (!(m->foc_motor_r > 0.0f && m->foc_motor_l > 0.0f && m->foc_motor_flux_linkage > 0.0f)) {
        P("R, L et flux doivent etre mesures ou saisis d'abord");
        return;
    }
    resc_conf_derive_current_gains(m, bw);
    resc_conf_derive_observer_gain(m);
    resc_conf_apply();
    resc_conf_store_mc();
    P("kp=%.4f ki=%.2f gain observateur=%.4g (bande passante %.0f rad/s), sauvegarde.",
      (double)m->foc_current_kp, (double)m->foc_current_ki, (double)m->foc_observer_gain, (double)bw);
}

static void cmd_help(void)
{
    P("Commandes RESC :");
    P("  status | rt | conf | hw | faults | clear");
    P("  detect_rl [A]                  : R (paliers 1/2, 3/4, 1 x A ; defaut 50 %% de l_current_max) puis L");
    P("  detect_lambda <A> [duty]       : flux (moteur LIBRE, il va tourner)");
    P("  offsets_cal                    : calibration des offsets (moteur a l'arret, sauvegarde)");
    P("  detect_hall [A]                : table des sondes Hall (le moteur tourne, sauvegarde)");
    P("  detect_enc [A]                 : codeur ABI : rapport, sens, offset (sauvegarde)");
    P("  sensor                         : etat des capteurs (Hall, source d'angle)");
    P("  current <A> | brake <A> | duty <d> | rpm <erpm> | stop");
    P("  foc_openloop <A> <erpm>        : boucle ouverte a courant impose (stop pour arreter)");
    P("  foc_openloop_duty <d> <erpm>   : boucle ouverte a rapport cyclique impose, sans regulation");
    P("  list | get <nom> | set <nom> <valeur>");
    P("  foc_gains [bande_rad_s]        : recalcule kp, ki, gain observateur (auto apres R/L et flux)");
    P("  store                          : sauvegarde la configuration (moteur + application)");
}

void terminal_process(char *line)
{
    char *argv[6];
    int argc = 0;
    for (char *t = strtok(line, " \t\r\n"); t && argc < 6; t = strtok(0, " \t\r\n")) argv[argc++] = t;
    if (argc == 0) return;
    const char *c = argv[0];

    if (!strcmp(c, "help")) cmd_help();
    else if (!strcmp(c, "status")) cmd_status();
    else if (!strcmp(c, "rt")) cmd_rt();
    else if (!strcmp(c, "conf")) cmd_conf();
    else if (!strcmp(c, "hw")) {
        P("Carte %s (%s), statut %d, OCP materielle %d, dead-time %d ns", BOARD_NAME, BOARD_ORIGIN, BOARD_STATUS, BOARD_HW_OCP, BOARD_DEADTIME_NS);
        P("Offsets courant %.1f %.1f %.1f, reset watchdog : %s", (double)g_foc_telem.off_cs[0], (double)g_foc_telem.off_cs[1],
          (double)g_foc_telem.off_cs[2], wdg_was_reset() ? "oui" : "non");
        P("Bootloader (secteur 11, mise a jour par VESC Tool) : %s", flash_bootloader_present() ? "present" : "ABSENT");
    }
    else if (!strcmp(c, "faults")) P("Defaut actif : %s (%u), crete de phase vue : %.1f A", fault_name(g_foc_telem.fault),
                                     (unsigned)g_foc_telem.fault, (double)g_foc_telem.i_phase_max);
    else if (!strcmp(c, "clear")) {
        if (board_gate_fault()) { P("Refuse : la ligne de defaut du driver est encore active"); return; }
        ctrl_release();
        foc_clear_fault();
        motor_hw_reset_stats();
        g_foc_telem.i_phase_max = 0;
        P("Defaut efface");
    }
    else if (!strcmp(c, "detect_rl")) detect_app_start_rl(0, argc >= 2 ? (float)atof(argv[1]) : 0.0f);
    else if (!strcmp(c, "detect_lambda") && argc >= 2) {
        detect_app_start_lambda(0, (float)atof(argv[1]), 1800.0f, argc >= 3 ? (float)atof(argv[2]) : 0.3f,
                                g_mcconf.foc_motor_r, g_mcconf.foc_motor_l);
    }
    else if (!strcmp(c, "offsets_cal") || !strcmp(c, "foc_dc_cal")) detect_app_start_offsets(0);
    else if (!strcmp(c, "detect_hall")) detect_app_start_hall(0, argc >= 2 ? (float)atof(argv[1]) : 0.0f);
    else if (!strcmp(c, "detect_enc")) detect_app_start_encoder(0, argc >= 2 ? (float)atof(argv[1]) : 0.0f);
    else if (!strcmp(c, "sensor")) {
        const foc_params_t *p = foc_params();
        static const char *mode[] = { "sensorless", "codeur", "Hall" };
        P("Mode %s, etat Hall=%u, angle %s, bascule observateur a %.0f ERPM", mode[p->sensor_mode < 3 ? p->sensor_mode : 0],
          g_foc_telem.hall, g_foc_telem.angle_from_sensor ? "capteur" : "observateur", (double)(p->sl_omega * 60.0f / 6.2831853f));
    }
    else if (!strcmp(c, "current") && argc >= 2) ctrl_set_current((float)atof(argv[1]));
    else if (!strcmp(c, "brake") && argc >= 2) ctrl_set_brake((float)atof(argv[1]));
    else if (!strcmp(c, "duty") && argc >= 2) ctrl_set_duty((float)atof(argv[1]));
    else if (!strcmp(c, "rpm") && argc >= 2) ctrl_set_rpm((float)atof(argv[1]));
    else if ((!strcmp(c, "foc_openloop") || !strcmp(c, "foc_openloop_duty")) && argc >= 3) {
        bool duty = !strcmp(c, "foc_openloop_duty");
        float v = (float)atof(argv[1]), erpm = (float)atof(argv[2]);
        if (!resc_power_allowed()) { P("Refuse : carte non relue (statut IMPORTED)"); return; }
        if (!resc_motor_configured()) { P("Refuse : moteur non identifie (R, L et flux necessaires)"); return; }
        if (duty) ctrl_set_openloop_duty(v, erpm); else ctrl_set_openloop(v, erpm);
        P("Boucle ouverte %s %.3g %s a %.0f ERPM ; 'stop' pour arreter (coupure aussi au timeout de commande)",
          duty ? "duty" : "courant", (double)v, duty ? "" : "A", (double)erpm);
    }
    else if (!strcmp(c, "stop")) ctrl_release();
    else if (!strcmp(c, "store")) P(resc_conf_store_mc() && resc_conf_store_app() ? "Configuration sauvegardee" : "Refuse : moteur en marche");
    else if (!strcmp(c, "list") || !strcmp(c, "get") || !strcmp(c, "set")) cmd_get_set(argc, argv);
    else if (!strcmp(c, "foc_gains")) cmd_foc_gains(argc, argv);
    else P("Commande inconnue : %s (taper help)", c);
}
