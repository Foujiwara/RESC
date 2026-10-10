/*
 * vesc_compat_defaults.h — Valeurs par defaut de la configuration exposee a
 * VESC Tool, derivees de la carte RESC (hw_config.h), puis completees par
 * les defauts de reference (vesc_defaults.h, genere).
 *
 * Ordre : 1) valeurs imposees par la carte (DEF_*), 2) choix prudents RESC
 * pour les premiers essais, 3) defauts de reference pour tout le reste.
 */
#ifndef VESC_COMPAT_DEFAULTS_H
#define VESC_COMPAT_DEFAULTS_H

#include <stdbool.h>
#include "hw_config.h"

/* --- 1. Valeurs de la carte --- */
#define MCCONF_L_MAX_VOLTAGE        ((float)DEF_VBUS_MAX_V)
#define MCCONF_L_MIN_VOLTAGE        ((float)DEF_VBUS_MIN_V)
#define MCCONF_L_MAX_ABS_CURRENT    ((float)DEF_I_ABS_TRIP_A)
#define MCCONF_FOC_F_ZV             (2.0f * (float)DEF_F_PWM_HZ)   /* f_zv = 2 f_PWM */
#define MCCONF_DEFAULT_MOTOR_TYPE   MOTOR_TYPE_FOC
#define MCCONF_FOC_SAMPLE_V0_V7     false
#define MCCONF_FOC_DT_US            ((float)BOARD_DEADTIME_NS * 0.25e-3f)

/* --- 2. Choix prudents RESC (premiers essais ; modifiables dans VESC Tool) --- */
#define MCCONF_L_CURRENT_MAX        30.0f
#define MCCONF_L_CURRENT_MIN        (-30.0f)
#define MCCONF_L_IN_CURRENT_MAX     30.0f
#define MCCONF_L_IN_CURRENT_MIN     (-20.0f)
#define MCCONF_FOC_OPENLOOP_RPM     1500.0f

/* --- Application --- */
uint8_t plat_default_controller_id(void);
#define HW_DEFAULT_ID               plat_default_controller_id()
#define APPCONF_CAN_STATUS_MSGS_R1  0x1F      /* STATUS 1 a 5 diffuses (ecrans, maitres CAN) */

/* --- 3. Defauts de reference --- */
#include "vesc_defaults.h"

#endif /* VESC_COMPAT_DEFAULTS_H */
