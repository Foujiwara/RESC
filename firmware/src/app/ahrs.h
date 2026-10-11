/*
 * ahrs.h — Estimation d'attitude a partir d'un gyrometre et d'un accelerometre (sans
 * magnetometre), comme l'IMU de VESC : filtres de Madgwick et de Mahony, et filtre « Fusion »
 * (retour de l'erreur de gravite par produit vectoriel, gain constant). La confiance dans
 * l'accelerometre baisse quand le module de l'acceleration s'ecarte de 1 g
 * (accel_confidence_decay). Angles en radians ; gyrometre en rad/s ; accelerometre en g.
 */
#ifndef AHRS_H
#define AHRS_H

typedef enum { AHRS_MADGWICK = 0, AHRS_MAHONY, AHRS_FUSION } ahrs_mode_t;

typedef struct {
    float q0, q1, q2, q3;
    float ifb[3];          /* integrale du retour (Mahony) */
    float acc_mag_f;       /* module filtre de l'acceleration */
    float decay, kp, ki, beta;
} ahrs_t;

void ahrs_init(ahrs_t *a);
void ahrs_params(ahrs_t *a, float acc_conf_decay, float mahony_kp, float mahony_ki, float beta);
void ahrs_update(ahrs_t *a, ahrs_mode_t mode, const float gyro_rad[3], const float acc_g[3], float dt);
void ahrs_rpy(const ahrs_t *a, float rpy[3]);

/* Filtre passe-bas du second ordre (Butterworth, Q = 0.707), fc en fraction de la frequence d'echantillonnage */
typedef struct { float a0, a1, a2, b1, b2, z1, z2; } biquad_t;
void biquad_lowpass(biquad_t *f, float fc);
float biquad_run(biquad_t *f, float x);

#endif
