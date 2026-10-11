/*
 * ctrl.h — Couche de controle RESC (1 kHz) : transforme les consignes recues
 * (courant, frein, duty, vitesse) en references de courant pour le coeur FOC,
 * applique les limitations (batterie, temperature, courant d'entree, puissance,
 * ERPM), le timeout de commande et integre les compteurs (Ah, Wh, tachymetre).
 */
#ifndef CTRL_H
#define CTRL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CTRL_MODE_NONE = 0,
    CTRL_MODE_CURRENT,
    CTRL_MODE_BRAKE,
    CTRL_MODE_DUTY,
    CTRL_MODE_RPM,
    CTRL_MODE_HANDBRAKE,
    CTRL_MODE_POS,              /* position (degres mecaniques, PID -> courant) */
    CTRL_MODE_OPENLOOP,         /* courant impose, angle a vitesse imposee */
    CTRL_MODE_OPENLOOP_DUTY     /* rapport cyclique impose, angle a vitesse imposee */
} ctrl_mode_t;

typedef struct {
    /* courant moteur (A, crete de phase) */
    float i_max, i_min;              /* i_min < 0 : couple negatif / frein */
    float i_max_scale, i_min_scale;
    float i_in_max, i_in_min;        /* batterie ; i_in_min < 0 : regeneration */
    float watt_max, watt_min;
    float cc_min_current;
    /* vitesse */
    float erpm_max, erpm_min;        /* erpm_min < 0 */
    float erpm_start;                /* fraction ou commence la limitation */
    /* tension */
    float batt_cut_start, batt_cut_end;
    float regen_cut_start, regen_cut_end;
    /* temperatures */
    float tfet_start, tfet_end;
    float tmot_start, tmot_end;
    bool  tmot_enabled;
    /* duty */
    float duty_max, duty_min;
    /* vitesse PID (semantique VESC : sortie en fraction de i_max, gains / 20) */
    float s_kp, s_ki, s_kd, s_kd_filter;
    float s_min_erpm, s_ramp_erpms;
    bool  s_allow_braking;
    /* commande */
    uint32_t timeout_ms;
    float timeout_brake;
    /* demarrage */
    float start_erpm_exit;           /* frein/roue libre sous cette vitesse */
    /* limitations supplementaires */
    float duty_start;                /* motorisation reduite lineairement de duty_start a duty_max */
    float temp_accel_dec;            /* limitation thermique avancee en motorisation (0..1) */
    uint32_t fault_stop_ms;          /* duree minimale pont coupe apres un defaut */
    uint8_t add_faults;              /* bit1 survitesse (110 %), bit3 survitesse absolue (130 %) */
    float batt_tau;                  /* s : filtre de la tension batterie des limitations */
    /* PID vitesse : cadence et source de vitesse */
    uint8_t s_rate_div;              /* execute un pas sur s_rate_div (1 kHz / s_rate_div) */
    uint8_t s_speed_src;             /* 0 PLL filtree, 1 rapide, 2 tres rapide */
    /* PID position */
    float p_kp, p_ki, p_kd, p_kd_filter;
    float p_ang_div;                 /* l'angle est divise par ce facteur (reduction) */
    float p_gain_dec;                /* degres : gains reduits en dessous de cette erreur */
    float p_offset;                  /* degres ajoutes a la consigne */
    float p_kd_proc;                 /* derivee sur la position mesuree (amortissement sans a-coup de consigne) */
    float pole_pairs;
    /* courant d'entree cartographie (l_in_current_map_start, l_in_current_map_filter) */
    float in_map_start;              /* fraction de i_in_max ; au-dessus, courant moteur max reduit lineairement */
    float in_map_filter;             /* filtre du courant d'entree par periode PWM */
    float f_isr;                     /* Hz, frequence de la boucle FOC (conversion du filtre) */
} ctrl_limits_t;

typedef struct {
    float ah, ah_chg, wh, wh_chg;
    int32_t tacho, tacho_abs;
    float tfet, tfet1, tfet2, tfet3, tmot;
    float i_motor;       /* courant moteur signe (filtre) */
    float i_in;
    float duty;
    float erpm;
    float vbus;
    float i_scale_lim;   /* facteur de limitation actif (0..1), diagnostic */
    float pos_deg;       /* position mecanique (degres, 0..360 divises par p_ang_div) */
    uint32_t fault;
    bool timeout;
} ctrl_stats_t;

void ctrl_init(const ctrl_limits_t *l);
void ctrl_set_limits(const ctrl_limits_t *l);
const ctrl_limits_t *ctrl_limits(void);

/* Consignes (protocole). Chacune rearme le timeout. */
void ctrl_set_current(float amps);
void ctrl_set_current_rel(float rel);
void ctrl_set_brake(float amps);
void ctrl_set_brake_rel(float rel);
void ctrl_set_handbrake(float amps);
void ctrl_set_duty(float duty);
void ctrl_set_rpm(float erpm);
void ctrl_set_pos(float deg);
void ctrl_release(void);
/* Boucle ouverte (essai) : courant (A) ou duty (0..1), vitesse en ERPM signee */
void ctrl_set_openloop(float amps, float erpm);
void ctrl_set_openloop_duty(float duty, float erpm);
void ctrl_alive(void);
void ctrl_set_off_delay(float seconds);
void ctrl_estop(void);
/* Autorisation globale du pont (statut de carte, moteur configure). */
void ctrl_set_power_allowed(bool allowed);

ctrl_mode_t ctrl_mode(void);
float ctrl_pos_set_deg(void);          /* consigne de position (degres), position actuelle hors mode position */

/* Pas de controle, a appeler a 1 kHz. now_ms : horloge systeme. */
void ctrl_update(uint32_t now_ms, float dt);

/* Limites du BMS sur le courant batterie (A ; lo <= 0 regeneration, hi >= 0 decharge), appliquees
 * en plus de i_in_min / i_in_max ; sans BMS : -1e9 / 1e9 */
void ctrl_set_bms_limits(float lo, float hi);

/* Mesures lentes fournies par la plateforme */
void ctrl_set_temps(float tfet1, float tfet2, float tfet3, float tmot);

const ctrl_stats_t *ctrl_stats(void);
void ctrl_reset_counters(void);

#endif /* CTRL_H */
