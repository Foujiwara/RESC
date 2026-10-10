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
    float vbus_trip_max;  /* V */
    float vbus_trip_min;  /* V (pont actif seulement) */
    /* Demarrage sensorless (I/f) */
    float st_i;           /* A */
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
    /* Offsets enregistres (calibration manuelle) */
    bool  use_stored_offsets;
    float stored_off_cs[3];   /* LSB */
    float off_vph[3];         /* V, tensions de phase pont coupe */
} foc_params_t;

/* ------------------------------------------------------------ Commande */
/* Ecrit par la couche de controle (1 kHz), lu par l'ISR. */
typedef struct {
    volatile float   id_ref;
    volatile float   iq_ref;
    volatile uint8_t run;          /* pont souhaite actif */
    volatile uint8_t allow_start;  /* autorise le demarrage I/f depuis l'arret */
    volatile int8_t  dir;          /* sens du demarrage I/f : +1 / -1 */
    volatile uint8_t openloop;     /* boucle ouverte commandee : 0 non, 1 courant (iq_ref), 2 duty */
    volatile float   ol_omega;     /* rad/s el. (signe = sens) */
    volatile float   ol_duty;      /* -1..1, mode 2 */
    volatile uint8_t duty_mode;    /* marche a rapport cyclique impose (tension q), courant borne */
    volatile float   duty;         /* -1..1 */
    volatile float   i_hi, i_lo;   /* A : bornes de iq en mode duty (i_lo <= 0 : frein) */
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
    FOC_DET_ENCODER    /* rapport, sens et offset du codeur */
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
    float theta;                  /* rad el. (angle de commande) */
    float obs_omega, obs_theta;   /* observateur sensorless seul (diagnostic) */
    float flux_ratio;             /* |flux estime| / lambda */
    float i_phase_max;            /* crete de phase observee (A) */
    uint32_t isr_cycles, isr_cycles_max;
    uint32_t ticks;
    uint32_t n_predicted;         /* echantillons de courant reconstruits */
    uint32_t n_overrun;
    float off_cs[3];
    uint32_t fault;               /* mc_fault_code */
    uint8_t hall;                 /* etat Hall courant */
    bool angle_from_sensor;       /* angle de commande issu d'un capteur */
#ifdef RESC_HOST
    float dbg_va, dbg_vb, dbg_ia, dbg_ib;
#endif
} foc_telem_t;

/* ------------------------------------------------------------ API */
void foc_init(const foc_params_t *p);
void foc_set_params(const foc_params_t *p);   /* pris en compte a la periode suivante */
const foc_params_t *foc_params(void);

/* Boucle FOC (ISR, RAM). Retourne les rapports cycliques a appliquer. */
void foc_isr(const foc_adc_t *adc, foc_pwm_t *out);

extern foc_cmd_t   g_foc_cmd;
extern foc_telem_t g_foc_telem;

foc_state_t foc_state(void);
bool foc_calibrated(void);

/* Defaut detecte hors ISR (EXTI, break materiel, controle lent) */
void foc_raise_fault(uint32_t code);
/* Rearmement (uniquement si aucune condition de defaut active) */
bool foc_clear_fault(void);

/* Identification moteur (lancee hors ISR, executee dans l'ISR) */
bool foc_detect_start(const foc_det_t *req);
const foc_det_t *foc_detect_status(void);
void foc_detect_abort(void);

/* Crochet bas niveau : coupure rapide du pont (break logiciel), fourni par bsp/ */
void foc_hw_kill(void);

#endif /* FOC_H */
