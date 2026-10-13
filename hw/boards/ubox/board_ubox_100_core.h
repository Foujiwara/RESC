/*
 * board_ubox_100_core.h — Famille Ubox 100 V (Spintend) : SINGLE_85_200,
 * SINGLE_80, SINGLE_100, V2_100. Ne pas inclure directement : passer par le
 * fichier de variante, qui definit BOARD_UBOX_<variante> puis inclut ce core.
 *
 * Origine : bldc release_7_00, hwconf/Ubox/100v/hw_ubox_100_core.{h,c} et
 * hw_ubox_single_*.h. Relu ligne a ligne contre ces sources (2026-10-10).
 * Ce n'est PAS une validation au banc : statut REVIEWED.
 *
 * Particularites de cette carte, importantes pour le firmware :
 *  1. Alimentation auto-maintenue : PA15 doit passer a 1 tres tot au boot
 *     (sinon le regulateur 12 V se coupe des que le bouton est relache).
 *     Le bouton (PC13) se lit avec une astuce open-drain : son niveau bas
 *     vaut ~1 V, ce qui ne suffit pas toujours a un 0 logique (voir le
 *     commentaire de hw_ubox_100_core.c).
 *  2. Gate drivers discrets : pas d'EN, pas de nFAULT et pas de BKIN
 *     exposes au MCU. PB12 (seule broche BKIN libre) sert a l'IMU (SCL
 *     logiciel). => AUCUNE protection surintensite independante du CPU
 *     n'est connue (BOARD_HW_OCP = 0). Voir docs/11_cartes_vesc.md.
 *  3. 3 shunts low-side (pas de shunts de phase), polarite d'ampli inversee.
 *  4. Dead-time materiel impose par VESC : 660 ns.
 *  5. Les macros VESC NRF5x_SWDIO (PA15) / SWCLK (PB3) entrent en conflit avec
 *     le maintien d'alimentation : la programmation SWD du module NRF n'est
 *     PAS reprise.
 *  6. Les variantes « _no_limits » de VESC ne sont volontairement pas
 *     supportees : les limites absolues ne sont jamais desactivables.
 */
#ifndef BOARD_UBOX_100_CORE_H
#define BOARD_UBOX_100_CORE_H

/* ------------------------------------------------------------- Identite */
#if defined(BOARD_UBOX_SINGLE_85_200)
#define BOARD_NAME              "UBOX_SINGLE_85_200"
#elif defined(BOARD_UBOX_SINGLE_80)
#define BOARD_NAME              "UBOX_SINGLE_80"
#elif defined(BOARD_UBOX_SINGLE_100)
#define BOARD_NAME              "UBOX_SINGLE_100"
#elif defined(BOARD_UBOX_V2_100)
#define BOARD_NAME              "UBOX_V2_100"
#else
#error "Variante Ubox 100 non definie (inclure un fichier board_ubox_*.h)"
#endif
#define BOARD_HW_MAJOR          0   /* non defini par VESC pour cette famille */
#define BOARD_HW_MINOR          0
#define BOARD_ORIGIN            "vesc-7.00:hwconf/Ubox/100v/hw_ubox_100_core.h"
#ifndef BOARD_STATUS
#define BOARD_STATUS            HW_BOARD_STATUS_REVIEWED
#endif

/* --------------------------------------------------- Etage de puissance */
/* PWM : brochage commun (PA8-10 / PB13-15), actif haut */
#define BOARD_GATE_DRIVER       HW_GD_GENERIC
#define BOARD_DEADTIME_NS       660         /* HW_DEAD_TIME_NSEC */
#define BOARD_GD_INTERNAL_DEADTIME 0

/* ----------------------------------------------- Protection materielle */
#define BOARD_BKIN_PIN          HW_PIN_NONE /* PB12 occupe par l'IMU */
#define BOARD_HW_OCP            0           /* aucune protection independante connue [V] */

/* ------------------------------------------------- Mesure de courant */
/* Voies communes : IN10/IN11/IN12 (PC0/PC1/PC2) sur ADC1/ADC2/ADC3 */
#define BOARD_CS_TOPOLOGY       HW_CS_LOWSIDE_3
#define BOARD_CS_SHUNT_OHM      (0.0005f / 3.0f)  /* 3 x 0.5 mOhm en parallele */
#define BOARD_CS_AMP_GAIN       20.0f
#define BOARD_CS_INVERTED       1                 /* INVERTED_SHUNT_POLARITY */
/* => 3.33 mV/A ; +/-516 A pleine echelle autour de VREF/2 ; 0.25 A/LSB [E] */

/* ------------------------------------------------- Mesure de tension */
#define BOARD_VREF_V            3.44f       /* V_REG */
#define BOARD_VBUS_R_TOP_OHM    150000.0f   /* VIN_R1 */
#define BOARD_VBUS_R_BOT_OHM    4700.0f     /* VIN_R2 => 113 V pleine echelle */
/* VESC convertit les tensions de phase avec le diviseur VIN : meme valeurs [V] */
#define BOARD_VPH_R_TOP_OHM     BOARD_VBUS_R_TOP_OHM
#define BOARD_VPH_R_BOT_OHM     BOARD_VBUS_R_BOT_OHM
#define BOARD_PHASE_FILTER_PIN  HW_PC9

/* ------------------------------------------------------ Temperatures */
/* hw100_250_get_temp() : max de 3 NTC 10k beta 3380, NTC cote VREF, 10k vers GND */
#define BOARD_TFET_MODEL        HW_TEMP_MAX_OF_3
#define BOARD_TFET_CH           3           /* PA3, IN3  (ADC3 en VESC) */
#define BOARD_TFET2_CH          8           /* PB0, IN8  (ADC1) */
#define BOARD_TFET3_CH          9           /* PB1, IN9  (ADC2) */
#define BOARD_TFET_NTC_R25_OHM  10000.0f
#define BOARD_TFET_NTC_BETA     3380.0f
#define BOARD_TFET_R_FIXED_OHM  10000.0f
#define BOARD_TFET_NTC_POS      HW_TEMP_NTC_TO_VREF
/* Moteur : IN14 (PC4), NTC vers GND, 10k vers VREF : defaut commun */

/* ---------------------------------------------------------- Annexes */
#define BOARD_ADC_EXT3_CH       15          /* PC5, IN15 */
#define BOARD_LED_GREEN_PIN     HW_PB5
#define BOARD_LED_RED_PIN       HW_PB7
#define BOARD_AUX_PIN           HW_PC12
#define BOARD_PWR_HOLD_PIN      HW_PA15     /* UBOX_POWER_EN : 1 = maintien */
#define BOARD_PWR_HOLD_EARLY    1           /* a piloter avant toute autre init */
#define BOARD_PWR_KEY_PIN       HW_PC13     /* open-drain + pull-up, lecture specifique */
#define BOARD_PWR_KEY_OD_TRICK  1
#define BOARD_IMU_SDA_PIN       HW_PB4      /* BMI160, I2C logiciel */
#define BOARD_IMU_SCL_PIN       HW_PB12
#define BOARD_IMU_FLIP          1           /* IMU_FLIP de VESC : axes x et z inverses */

/* ---------------------------------------------------- Communications */
/* UART VESC : USART3 PB10/PB11 (commun). UART permanent NRF52 : UART4 */
#define BOARD_UART2_INST        4
#define BOARD_UART2_TX_PIN      HW_PC10
#define BOARD_UART2_RX_PIN      HW_PC11
#define BOARD_UART2_BAUD        115200
/* CAN1 PB8/PB9, halls PC6-8/TIM3, servo PB6, SPI1 PA4-7 : communs */

/* ----------------------------------- Limites absolues (HW_LIM_* VESC) */
#if defined(BOARD_UBOX_SINGLE_85_200)
#define HWLIM_I_MOTOR_A         300
#define HWLIM_I_INPUT_A         300
/* VESC : 420 A. Abaisse a 350 A (defaut 300 A) : pas de protection materielle (BOARD_HW_OCP 0), coupure
 * logicielle avec une periode PWM de retard et pleine echelle des shunts a 516 A (doc 15, S1). A relever
 * quand les MOSFET (I_DM) seront identifies, doc 08 U1. */
#define HWLIM_I_ABS_TRIP_A      350
#define HWLIM_VBUS_MIN_V        11
#define HWLIM_VBUS_MAX_V        85
#elif defined(BOARD_UBOX_SINGLE_80)
#define HWLIM_I_MOTOR_A         150
#define HWLIM_I_INPUT_A         150
#define HWLIM_I_ABS_TRIP_A      210
#define HWLIM_VBUS_MIN_V        11
#define HWLIM_VBUS_MAX_V        85
#else   /* SINGLE_100, V2_100 */
#define HWLIM_I_MOTOR_A         135
#define HWLIM_I_INPUT_A         135
#define HWLIM_I_ABS_TRIP_A      180
#define HWLIM_VBUS_MIN_V        11
#define HWLIM_VBUS_MAX_V        95
#endif
#define HWLIM_ERPM              200000
#define HWLIM_DUTY_MAX_PERMILLE 990
#define HWLIM_T_FET_C           110

/* -------------------------------------- Defauts (MCCONF_* de VESC) */
/* VESC : foc_f_zv = 30 kHz, PWM centree => f_PWM = f_zv / 2 = 15 kHz */
#define DEF_F_PWM_HZ            15000
#define DEF_VBUS_MIN_V          12
#if defined(BOARD_UBOX_SINGLE_85_200)
#define DEF_VBUS_MAX_V          82
#define DEF_I_ABS_TRIP_A        300     /* VESC : 400 A ; voir HWLIM_I_ABS_TRIP_A */
#define DEF_I_INPUT_MAX_A       250
#define DEF_I_INPUT_MIN_A       (-200)
#elif defined(BOARD_UBOX_SINGLE_80)
#define DEF_VBUS_MAX_V          78
#define DEF_I_ABS_TRIP_A        160
#define DEF_I_INPUT_MAX_A       84
#define DEF_I_INPUT_MIN_A       (-65)
#else
#define DEF_VBUS_MAX_V          93
#define DEF_I_ABS_TRIP_A        160
#define DEF_I_INPUT_MAX_A       84
#define DEF_I_INPUT_MIN_A       (-65)
#endif

#endif /* BOARD_UBOX_100_CORE_H */
