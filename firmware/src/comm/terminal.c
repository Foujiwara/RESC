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
#include "apps.h"
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
    static const char *const ld[4] = { "vide (defauts)", "OK", "MIGREE depuis l'ancien format", "ILLISIBLE (defauts)" };
    P("Configuration en flash : moteur %s, application %s", ld[resc_conf_load_status(false)], ld[resc_conf_load_status(true)]);
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

/* ------------------------------------------------ parametres accessibles au terminal
 * Tous les champs de configuration (moteur puis application), par leur nom C :
 * nom du XML avec '.' et '__' remplaces par '_' (ex. app_ppm_conf_ctrl_type).
 * 'list' : parametres principaux ; 'list <debut>' : champs dont le nom commence ainsi. */
static const char *const MAIN_PARAMS[] = {
    "l_current_max", "l_current_min", "l_in_current_max", "l_in_current_min", "l_abs_current_max",
    "l_min_vin", "l_max_vin", "l_max_erpm", "foc_motor_kind", "foc_motor_r", "foc_motor_l", "foc_motor_ld_lq_diff",
    "foc_motor_flux_linkage", "foc_current_kp", "foc_current_ki", "foc_observer_gain", "foc_observer_type",
    "foc_f_zv", "foc_openloop_rpm", "foc_sl_erpm", "foc_sensor_mode", "m_encoder_counts", "si_motor_poles",
    "foc_acim_lm", "foc_acim_lr", "foc_acim_rr", "foc_acim_im", "controller_id", "timeout_msec", "app_to_use",
};

static void show_field(bool app, const conf_field_t *f)
{
    P("  %-34s = %.7g", f->name, (double)resc_conf_field_get(app, f));
}

static bool find(const char *name, bool *app, const conf_field_t **f)
{
    *f = resc_conf_field(false, name);
    *app = false;
    if (!*f) { *f = resc_conf_field(true, name); *app = true; }
    return *f != NULL;
}

static void cmd_get_set(int argc, char **argv)
{
    bool app;
    const conf_field_t *f;
    if (!strcmp(argv[0], "list")) {
        if (argc < 2) {
            for (unsigned i = 0; i < sizeof(MAIN_PARAMS) / sizeof(MAIN_PARAMS[0]); i++)
                if (find(MAIN_PARAMS[i], &app, &f)) show_field(app, f);
            P("('list <debut de nom>' pour les autres champs, 'list all' pour tous)");
            return;
        }
        bool all = !strcmp(argv[1], "all");
        size_t k = strlen(argv[1]);
        for (int t = 0; t < 2; t++) {
            const conf_field_t *tab = t ? vesc_appconf_fields : vesc_mcconf_fields;
            uint16_t n = t ? vesc_appconf_nfields : vesc_mcconf_nfields;
            for (uint16_t i = 0; i < n; i++)
                if (all || !strncmp(tab[i].name, argv[1], k)) show_field(t != 0, &tab[i]);
        }
        return;
    }
    if (argc < 2) { P("Usage : get <nom> | set <nom> <valeur>"); return; }
    if (!find(argv[1], &app, &f)) { P("Parametre inconnu : %s (taper list)", argv[1]); return; }
    if (!strcmp(argv[0], "set")) {
        if (argc < 3) { P("Valeur manquante"); return; }
        if (foc_state() == FOC_ST_RUN || foc_state() == FOC_ST_START) { P("Refuse : moteur en marche"); return; }
        resc_conf_field_set(app, f, (float)atof(argv[2]));
        resc_conf_clamp_mc(&g_mcconf);
        resc_conf_clamp_app(&g_appconf);
        resc_conf_apply();
        if (app) apps_configure();
        P("(applique ; 'store' pour sauvegarder)");
    }
    show_field(app, f);
}

/* Gains a partir de R, L, lambda, sur demande : bande passante de courant en rad/s */
static void cmd_foc_gains(int argc, char **argv)
{
    float bw = argc >= 2 ? (float)atof(argv[1]) : g_mcconf.foc_resc_cc_bw;
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
