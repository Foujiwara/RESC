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
    float duty_kp, duty_ki;
    /* vitesse PID (semantique VESC : sortie en fraction de i_max, gains / 20) */
    float s_kp, s_ki, s_kd, s_kd_filter;
    float s_min_erpm, s_ramp_erpms;
    bool  s_allow_braking;
    /* commande */
    uint32_t timeout_ms;
    float timeout_brake;
    /* demarrage */
    float start_erpm_exit;           /* frein/roue libre sous cette vitesse */
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

/* Pas de controle, a appeler a 1 kHz. now_ms : horloge systeme. */
void ctrl_update(uint32_t now_ms, float dt);

/* Mesures lentes fournies par la plateforme */
void ctrl_set_temps(float tfet1, float tfet2, float tfet3, float tmot);

const ctrl_stats_t *ctrl_stats(void);
void ctrl_reset_counters(void);

#endif /* CTRL_H */
