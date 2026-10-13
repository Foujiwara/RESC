/*
 * foc.h — Coeur FOC RESC : boucle de courant executee a chaque periode PWM.
 *
 * Le coeur ne touche aucun registre : il recoit les mesures ADC brutes
 * (foc_adc_t) et rend les rapports cycliques et l'etat du pont (foc_pwm_t).
 * La couche bsp/ fait le lien avec TIM1/ADC ; sim/ le relie a un modele moteur.
 *
 * Conventions :
 *  - Clarke invariante en amplitude : |i_dq| = courant crete de phase.
 *  - Courant de phase positif = courant entrant dans le moteur.
 *  - duty[x] = fraction de conduction du high-side de la phase x (0..1).
 *  - Angles electriques en rad, vitesses electriques en rad/s.
 */
#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include <stdbool.h>
#include "angle_sensors.h"

/* ---------------------------------------------------------------- E/S */
typedef struct {
    uint16_t cs[3];      /* courants A, B, C (brut ADC) */
    uint16_t vph[3];     /* tensions de phase A, B, C (brut) */
    uint16_t vbus;       /* tension bus (brut) */
    uint8_t  hall;       /* etat des sondes Hall : H1 | H2 << 1 | H3 << 2 */
    uint32_t enc;        /* compteur du codeur ABI */
} foc_adc_t;

/* Source d'angle (valeurs du protocole, foc_sensor_mode) */
#define FOC_SENSOR_SENSORLESS 0
#define FOC_SENSOR_ENCODER    1
#define FOC_SENSOR_HALL       2
#define FOC_SENSOR_HFI        3   /* injection haute frequence a basse vitesse, observateur au-dessus */
#define FOC_SENSOR_VSS        4   /* demarrage : position par injection, puis I/f sans calage, observateur */

/* Injection haute frequence (HFI) : creneaux de tension ++-- sur l'axe d estime,
 * demodulation de la reponse en courant sur l'axe q estime (saillance Lq > Ld). */
typedef struct {
    float v_start, v_run, v_max;  /* V : injection au demarrage, en marche, a courant max */
    float gain;                   /* correction d'angle par mise a jour (0..1) */
    float max_err;                /* rad : erreur d'angle max prise en compte par mise a jour */
    uint16_t samples;             /* creneaux (de 4 periodes) moyennes par mise a jour */
    uint16_t start_updates;       /* mises a jour de convergence avant le test de polarite */
    uint8_t amb_mode;             /* polarite : 0 six vecteurs, 1 impulsion simple, 2 impulsion double */
    float amb_current;            /* A */
    float amb_time;               /* s par impulsion */
    float sl_omega;               /* rad/s : observateur au-dessus */
    float hyst;                   /* fraction de sl_omega */
    float i_ref;                  /* A : courant pour v_max */
    float reset_omega;            /* rad/s : pont coupe au-dessus, redemarrage sur l'observateur sans recherche */
    float obs_ovr;                /* s : angle HFI recale sur l'observateur apres le retour a basse vitesse */
} hfi_cfg_t;

typedef struct {
    float duty[3];       /* phases A, B, C */
    bool  enable;        /* true : pont actif (MOE=1) */
} foc_pwm_t;

/* ------------------------------------------------------------ Parametres */
typedef struct {
    /* Echantillonnage */
    float f_pwm;          /* Hz (= frequence de la boucle FOC) */
    /* Conversion ADC -> unites physiques */
    float i_scale;        /* A par LSB (signe inclus : polarite de l'ampli) */
    float v_scale;        /* V par LSB, tension bus */
    float vph_scale;      /* V par LSB, tensions de phase */
    bool  cs_lowside;     /* true : shunts low-side (fenetres de mesure) */
    bool  cs_two_shunts;  /* true : 2 shunts seulement (phase C reconstruite) */
    bool  invert_dir;     /* echange des phases B et C */
    /* Moteur */
    float r, ld, lq, lambda;
    /* Boucle de courant */
    float kp, ki;
    /* Observateur et PLL */
    float obs_gamma;      /* gain non lineaire (1 / (Wb^2 s)) */
    float pll_kp, pll_ki;
    float dt_comp;        /* s, compensation du temps mort cote observateur */
    /* Modulation et mesure */
    float m_max;          /* modulation lineaire max (1 = Vbus/sqrt3) */
    float t_win;          /* s, fenetre low-side minimale pour un echantillon valide */
    float delay_comp;     /* retard compense, en periodes PWM */
    /* Protections rapides (dans l'ISR) */
    float i_abs_trip;     /* A crete */
    bool  enc_slip_fault; /* defaut si la vitesse du codeur s'ecarte de celle de l'observateur */
    float vbus_trip_max;  /* V */
    float vbus_trip_min;  /* V (pont actif seulement) */
    /* Demarrage sensorless (I/f) */
    float st_i;           /* A : courant de demarrage minimal (foc_sl_openloop_boost_q) */
    float st_i_max;       /* A : courant de demarrage maximal (foc_sl_openloop_max_q) */
    float st_t_align;     /* s */
    float st_t_ramp;      /* s, 0 -> vitesse de relais */
    float st_omega;       /* rad/s el., vitesse de relais vers l'observateur */
    float st_omega_exit;  /* rad/s el., retour en I/f en dessous */
    float st_timeout;     /* s */
    /* Fluage des references internes */
    float i_slew;         /* A/s */
    /* Capteurs d'angle */
    uint8_t sensor_mode;  /* FOC_SENSOR_* */
    hall_cfg_t hall;
    enc_cfg_t enc;
    float sl_omega;       /* rad/s : au-dessus, l'observateur remplace les sondes Hall */
    /* Observateur (suite) */
    uint8_t obs_type;     /* foc_observer_type 0..6, voir observer.h */
    float obs_gain_slow;  /* fraction du gain a modulation nulle */
    float obs_offset;     /* avance d'angle supplementaire (periodes) */
    uint8_t speed_source; /* 0 : derivee de l'angle de commande, 1 : PLL de l'observateur */
    /* Compensations */
    uint8_t sat_comp_mode; /* 0 aucune, 1 inductances, 2 lambda, 3 les deux */
    float sat_comp;       /* baisse relative a i_sat_ref */
    float i_sat_ref;      /* A */
    bool  temp_comp;      /* R (cuivre) et lambda (aimants) suivant la temperature moteur */
    float temp_comp_base; /* degC : temperature des valeurs identifiees */
    uint8_t decoupling;   /* bit0 : termes croises, bit1 : FEM */
    uint8_t mtpa_mode;    /* 0 off, 1 sur la consigne iq, 2 sur iq mesure */
    /* Defluxage */
    float fw_i_max;       /* A (0 = off) */
    float fw_duty_start;  /* modulation relative a m_max ou il commence */
    float fw_ramp_time;   /* s pour atteindre fw_i_max */
    float fw_q_factor;    /* fraction de iq conservee a defluxage maximal */
    /* Demarrage (suite) */
    float st_low;         /* vitesse de relais a courant nul, en fraction (0 = fixe) */
    float st_i_ref;       /* A : courant pleine vitesse de relais (st_low) */
    float st_t_hold;      /* s a pleine vitesse avant le relais */
    float st_hyst;        /* s sous la vitesse de sortie avant retour en I/f */
    float st_dec;         /* fraction du courant autorisee a l'arret (1 = off) */
    float st_dec_omega;   /* rad/s : courant plein au-dessus */
    /* Hall (suite) */
    float sl_omega_start; /* rad/s : debut du melange Hall -> observateur */
    uint8_t hall_extra;   /* lectures identiques exigees pour un nouvel etat */
    /* HFI / VSS */
    hfi_cfg_t hfi;
    /* Filtres de phase (tension de phase filtree par RC, commutee par la carte) */
    bool  pf_enable;
    bool  pf_no_fault;
    float pf_omega;       /* rad/s : tensions mesurees pour l'observateur en dessous */
    /* Mode duty : regulateur de descente (frein) */
    bool  duty_basic;     /* mode duty simple : roue libre (courant nul) quand la consigne descend sous le duty */
    float duty_dr_kp;     /* A par unite de duty */
    float duty_dr_ki;     /* A/s par unite de duty */
    /* Moteur a induction (motor_kind = 1) : r = Rs, ld = lq = sigma Ls (inductance transitoire) */
    uint8_t motor_kind;   /* 0 aimants permanents, 1 induction */
    float acim_lm, acim_lr, acim_rr;   /* H, H, Ohm (rotor ramene au stator) */
    float acim_im;        /* A : courant magnetisant (axe d) */
    float acim_base_omega;/* rad/s el. : defluxage au-dessus (0 = non) */
    /* Modulation (foc_overmod_factor, foc_mag_vd_max, foc_short_ls_on_zero_duty) */
    float overmod;        /* rayon du cercle de tension x overmod (1..1.5, surmodulation au-dela de 1) */
    float vd_max;         /* fraction du cercle autorisee sur l'axe d */
    bool  short_ls;       /* tensions nulles : low-sides passants en continu au lieu d'un court-circuit module */
    /* Echantillonnage (foc_control_sample_mode, foc_current_sample_mode, foc_current_filter_const) */
    uint8_t ctl_mode;     /* 0 V0, 1 V0 et V7 (boucle a 2 x f_pwm, shunts en ligne), 2 V0 + mise a jour interpolee en V7 */
    uint8_t cs_mode;      /* 0 plus longue fenetre, 1 tous les capteurs valides, 2 fort courant */
    float i_filter;       /* filtre des courants d/q affiches et limites (par periode) */
    float fw_backoff;     /* defluxage reduit quand iq depasse sa consigne */
    /* Offsets enregistres (calibration manuelle) */
    bool  use_stored_offsets;
    float stored_off_cs[3];   /* LSB */
    float off_vph[3];         /* V, tensions de phase pont coupe */
} foc_params_t;

/* ------------------------------------------------------------ Commande */
/* Ecrit par la couche de controle (1 kHz), lu par l'ISR. Publication par foc_cmd_publish() : compteur
 * de sequence impair pendant l'ecriture ; l'ISR (plus prioritaire) garde alors la commande precedente,
 * elle ne voit jamais un melange de deux commandes. Les essais sur PC peuvent ecrire les champs
 * directement (contexte unique, compteur pair). */
typedef struct {
    uint32_t seq;                  /* pair : stable, impair : ecriture en cours */
    float   id_ref;
    float   iq_ref;
    uint8_t run;          /* pont souhaite actif */
    uint8_t allow_start;  /* autorise le demarrage I/f depuis l'arret */
    int8_t  dir;          /* sens du demarrage I/f : +1 / -1 */
    uint8_t openloop;     /* boucle ouverte commandee : 0 non, 1 courant (iq_ref), 2 duty,
                                      3 frein a main (courant iq_ref sur l'axe d, angle fige) */
    float   ol_omega;     /* rad/s el. (signe = sens) */
    float   ol_duty;      /* -1..1, mode 2 */
    uint8_t duty_mode;    /* marche a rapport cyclique impose (tension q), courant borne */
    float   duty;         /* -1..1 */
    float   i_hi, i_lo;   /* A : bornes de iq en mode duty (i_lo <= 0 : frein) */
    uint8_t brake;        /* frein : |iq_ref| oppose a la vitesse ; phases en court-circuit si ce
                                      courant n'est pas atteignable (basse vitesse), y compris a l'arret */
} foc_cmd_t;

/* ------------------------------------------------------------ Etats */
typedef enum {
    FOC_ST_INIT = 0,
    FOC_ST_CALIB,      /* mesure des offsets, pont coupe */
    FOC_ST_IDLE,       /* pont coupe, observateur sur tensions mesurees */
    FOC_ST_START,      /* demarrage I/f */
    FOC_ST_RUN,        /* boucle fermee sur l'observateur */
    FOC_ST_DETECT,     /* identification moteur */
    FOC_ST_FAULT
} foc_state_t;

typedef enum {
    FOC_DET_NONE = 0,
    FOC_DET_RL,        /* resistance puis inductances */
    FOC_DET_LAMBDA,    /* flux en boucle ouverte */
    FOC_DET_OFFSETS,   /* offsets courant (pont coupe puis vecteur nul) et tensions de phase */
    FOC_DET_HALL,      /* table des sondes Hall */
    FOC_DET_ENCODER,   /* rapport, sens et offset du codeur */
    FOC_DET_ACIM       /* moteur a induction : Lm, Lr, Rr par echelon de courant continu */
} foc_det_kind_t;

typedef struct {
    foc_det_kind_t kind;
    volatile bool  busy;
    volatile bool  done;
    volatile int   error;      /* 0 = OK, <0 = erreur, >0 = code de defaut */
    float i_test;              /* A */
    float omega_test;          /* rad/s (lambda) */
    float ramp_rate;           /* rad/s^2 (lambda) */
    float duty_target;         /* modulation a atteindre avant la mesure (lambda) */
    float r_known, l_known;    /* pour le calcul du flux */
    /* resultats */
    float r, ld, lq, lambda;
    float off_cs[3], off_vph[3];   /* LSB ; V pont coupe (ecart a la moyenne) */
    float off_vph_drv[3];          /* V sous PWM (ecart a la moyenne des 3 phases) */
    float hall_angle[8];       /* rad ; hall_valid[s] faux = etat invalide */
    bool  hall_valid[8];
    float enc_offset, enc_ratio;
    bool  enc_inverted;
    float r_i_low, r_i_high, r_v_high;   /* paliers utilises pour R, chute ohmique entre eux */
    float r_v_dt;              /* V : tension perdue a courant constant (temps mort, diodes) */
    int   warn;                /* 1 : chute ohmique faible ; 2 : temps mort non sature au palier bas */
    float acim_k, acim_tr;     /* induction : Lm^2/Lr (H), constante de temps rotorique (s) */
    float acim_lm, acim_lr, acim_rr;
} foc_det_t;

/* ------------------------------------------------------------ Telemetrie */
typedef struct {
    foc_state_t state;
    float id, iq, vd, vq;         /* filtres */
    float id_ref, iq_ref;
    float i_abs;                  /* |i_dq| filtre */
    float i_bus;                  /* courant batterie estime (A) */
    float vbus;
    float duty;                   /* modulation signee (-1..1) */
    float omega;                  /* rad/s el. (PLL) */
    float omega_fast;             /* rad/s el., filtrage leger */
    float omega_raw;              /* rad/s el., sans filtrage */
    float theta;                  /* rad el. (angle de commande) */
    float obs_omega, obs_theta;   /* observateur sensorless seul (diagnostic) */
    float sens_theta;             /* rad el. : angle du capteur (Hall ou codeur) */
    float flux_ratio;             /* |flux estime| / lambda */
    float lambda_est;             /* lambda utilise par l'observateur (adaptatif) */
    float acim_psir;              /* induction : flux rotorique du modele (Wb) */
    float acim_slip;              /* induction : glissement (rad/s el.) */
    float fw_id;                  /* courant de defluxage (A, <= 0) */
    float hfi_err;                /* rad, derniere erreur d'angle HFI */
    bool  hfi_active;
    uint8_t angle_src;            /* 0 observateur, 1 Hall, 2 codeur, 3 HFI, 4 boucle ouverte */
    float i_phase_max;            /* crete de phase observee (A) */
    uint32_t isr_cycles, isr_cycles_max;
    uint32_t ticks;
    uint32_t n_predicted;         /* echantillons de courant reconstruits */
    uint32_t n_overrun;
    float off_cs[3];
    uint32_t fault;               /* mc_fault_code */
    uint32_t last_fault;          /* dernier defaut vu (reste apres l'effacement automatique) */
    uint32_t fault_count;         /* nombre de defauts depuis le demarrage */
    uint32_t last_fault_line;     /* origine du dernier defaut (ligne de foc.c, 0 = externe) */
    struct { uint32_t code, line, ticks; } fault_log[8];   /* 8 derniers defauts (circulaire, ticks = periodes FOC) */
    uint32_t fault_log_n;
    uint32_t obs_resync;
    float off_vph_used[3];        /* V : offsets de tension pont coupe en service */
    bool off_vph_boot;            /* calibres au demarrage (moteur a l'arret) */
    float off_vph_sd;             /* V : ecart-type entre phases pendant la calibration (arret si < 0.3) */          /* reprises ou l'observateur a ete recale sur les Hall */
    uint8_t hall;                 /* etat Hall courant */
    bool angle_from_sensor;       /* angle de commande issu d'un capteur */
#ifdef RESC_HOST
    float dbg_va, dbg_vb, dbg_ia, dbg_ib;
    float dbg_amp0, dbg_amp_p, dbg_amp_m, dbg_hfi_th;
#endif
} foc_telem_t;

/* ------------------------------------------------------------ API */
void foc_init(const foc_params_t *p);
void foc_dbg_obs_reset(float theta, float omega);   /* essais : decrochage simule de l'observateur */
void foc_set_params(const foc_params_t *p);   /* pris en compte a la periode suivante */
const foc_params_t *foc_params(void);

/* Boucle FOC (ISR, RAM). Retourne les rapports cycliques a appliquer. */
void foc_isr(const foc_adc_t *adc, foc_pwm_t *out);
/* Mise a jour de demi-periode (creux du compteur, ctl_mode 2) : rapports cycliques de la seconde
 * moitie avec l'angle interpole. Retourne false s'il n'y a rien a ecrire. */
bool foc_isr_half(foc_pwm_t *out);
/* Echantillons bruts de la derniere periode (echantillonnage pour VESC Tool) */
typedef struct {
    float i[3];          /* A, courants de phase */
    float v[3];          /* V, tensions de phase */
    float vzero;         /* V, point milieu (Vbus / 2 pont actif) */
    float duty[3];
    float theta;         /* rad, angle de commande */
    float i_ab[2];       /* A, alpha / beta */
    uint8_t state;
} foc_sample_t;
void foc_last_sample(foc_sample_t *s);

extern volatile foc_cmd_t g_foc_cmd;
/* Publie une commande complete (couche de controle) */
void foc_cmd_publish(const foc_cmd_t *c);
extern foc_telem_t g_foc_telem;

foc_state_t foc_state(void);
bool foc_calibrated(void);

/* Defaut detecte hors ISR (EXTI, break materiel, controle lent) */
void foc_raise_fault(uint32_t code);
/* Temperature moteur (degC) pour la compensation, appelee par la couche lente */
void foc_set_motor_temp(float t);
/* Rearmement (uniquement si aucune condition de defaut active) */
bool foc_clear_fault(void);

/* Identification moteur (lancee hors ISR, executee dans l'ISR) */
bool foc_detect_start(const foc_det_t *req);
const foc_det_t *foc_detect_status(void);
void foc_detect_abort(void);

/* Crochets bas niveau fournis par bsp/ (et par la simulation) :
 * coupure rapide du pont (break logiciel), commutation du filtre de phase */
void foc_hw_kill(void);
void foc_hw_phase_filter(bool on);
/* Autorise de nouveau l'activation du pont (sortie de l'etat de defaut). Definition faible vide dans
 * foc.c (simulation, tests) ; bsp/ la remplace. */
void foc_hw_rearm(void);

#endif /* FOC_H */
