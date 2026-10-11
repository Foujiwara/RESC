/*
 * imu.h — Centrale inertielle (imu_conf de VESC) : BMI160 interne (I2C logiciel de la carte)
 * ou externe (I2C sur les broches du port UART). Echantillonnage dans la boucle principale a
 * imu_conf.sample_rate_hz, filtres passe-bas, rotation de montage, decalages, attitude
 * (Madgwick, Mahony, Fusion). Donnees pour VESC Tool (COMM_GET_IMU_DATA) et calibration
 * (COMM_GET_IMU_CALIBRATION, sans bloquer la boucle principale).
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "packet.h"

void imu_configure(void);                /* d'apres g_appconf (imu_conf_*) */
void imu_poll(uint32_t now_us);          /* boucle principale */
bool imu_ready(void);
void imu_get_rpy(float rpy[3]);          /* rad */
void imu_get_accel(float a[3]);          /* g, apres rotation et decalages */
void imu_get_gyro(float g[3]);           /* deg/s */
void imu_get_quat(float q[4]);
/* COMM_GET_IMU_DATA : [masque u16] -> reponse */
int32_t imu_data_reply(uint8_t *b, uint16_t mask);
/* COMM_GET_IMU_CALIBRATION : demarre la calibration ; la reponse part a la fin (~4 s) */
void imu_calibration_start(float yaw_deg, const comm_iface_t *reply);

#endif
