/*
 * sim_motor.h — Modele de PMSM + onduleur + ADC pour tester le coeur FOC RESC sur PC.
 * Modele moyen par periode PWM (pas d'ondulation), temps mort modelise comme
 * une perte de tension dependant du signe du courant, mise a jour des rapports
 * cycliques au creux du compteur (une demi-periode apres l'echantillon).
 */
#ifndef SIM_MOTOR_H
#define SIM_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "foc.h"

typedef struct {
    /* moteur */
    double r, ld, lq, lambda;
    int    pp;
    double j, b, t_load;      /* inertie (kg m^2), frottement visqueux, couple resistant */
    /* onduleur et mesure */
    double vbus;
    double t_dead;            /* s, temps mort effectif */
    double i_soft;            /* A : en dessous, l'erreur de temps mort est partielle (Coss des MOSFET) */
    double ld_sat;            /* saturation : Ld incremental = Ld (1 - ld_sat id), id > 0 vers l'aimant */
    bool   pf_on;             /* filtre RC des tensions de phase commute */
    double pf_tau;            /* s */
    double vph_f[3];          /* tensions de phase filtrees */
    double vph_off[3];        /* V : erreur d'offset de la mesure de tension de phase (diviseurs reels) */
    /* moteur asynchrone (acim = true) : r = Rs, ld = lq = sigma Ls (inductance transitoire) */
    bool   acim;
    double lm, lr, rr;        /* inductance magnetisante, rotorique, resistance rotorique ramenee */
    double ia_s, ib_s;        /* courant statorique alpha/beta */
    double pra, prb;          /* flux rotorique alpha/beta */
    double ea_s, eb_s;        /* (Lm/Lr) d(psi_r)/dt, pour les tensions pont coupe */
    double f_pwm;
    double i_lsb;             /* A par LSB (signe = polarite de l'ampli) */
    double v_lsb, vph_lsb;    /* V par LSB */
    double off[3];            /* offsets ADC des courants */
    double noise_lsb;
    /* capteurs */
    double hall_off;          /* rad electriques : calage des sondes */
    uint32_t enc_cpr;
    double enc_off;           /* rad mecaniques */
    bool enc_rev;
    /* etat */
    double id, iq, theta, omega_m;
    double t;
    double duty_cur[3], duty_next[3];
    bool   en_cur, en_next;
    /* foc_control_sample_mode : seconde moitie interpolee (2), boucle a 2 x f_pwm (1) */
    bool   h_ok, hn_ok;           /* rapports de la seconde moitie fournis (foc_isr_half) */
    double duty_h[3], duty_hn[3];
    bool   full_delay;            /* V0 et V7 : commande appliquee une periode de boucle apres l'echantillon */
} sim_motor_t;

void sim_init(sim_motor_t *m);
/* Avance d'une periode PWM : applique la sortie FOC precedente et produit l'echantillon suivant. */
void sim_step(sim_motor_t *m, const foc_pwm_t *out, foc_adc_t *adc);
double sim_theta_e(const sim_motor_t *m);
double sim_omega_e(const sim_motor_t *m);
void sim_phase_currents(const sim_motor_t *m, double i[3]);

#endif
