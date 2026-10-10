/*
 * board_x12_pro24.h — X12_PRO24
 *
 * GENERE AUTOMATIQUEMENT par tools/hw_import/vesc_board_import.py
 * depuis bldc release_7_00 : hwconf/tronic/hw_x12_pro24.h (+ hw_x12_pro24.c).
 * Statut IMPORTED : etage de puissance desactive tant que la carte n'a
 * pas ete relue contre les sources VESC (puis passee en REVIEWED).
 *
 * Points a verifier [V] :
 *  - sonde FET specifique (custom:hwt12t_get_temp) : beta 3380 suppose
 *  - fonctions specifiques du .c VESC (alimentation, bouton, capteurs) non reprises
 */
#ifndef BOARD_TRONIC_HW_X12_PRO24_H
#define BOARD_TRONIC_HW_X12_PRO24_H

#define BOARD_NAME              "X12_PRO24"
#define BOARD_HW_MAJOR          0
#define BOARD_HW_MINOR          0
#define BOARD_ORIGIN            "vesc-7.00:hwconf/tronic/hw_x12_pro24.h"
#define BOARD_STATUS            HW_BOARD_STATUS_IMPORTED

/* Etage de puissance */
#define BOARD_GATE_DRIVER       HW_GD_GENERIC
#define BOARD_DEADTIME_NS       350

/* Protection materielle */
#define BOARD_HW_OCP            0   /* [V] deduit du HW VESC */

/* Mesure de courant */
#define BOARD_CS_TOPOLOGY       HW_CS_INLINE_3
#define BOARD_CS_A_CH           10
#define BOARD_CS_A_ADC          1
#define BOARD_CS_B_CH           11
#define BOARD_CS_B_ADC          2
#define BOARD_CS_C_CH           12
#define BOARD_CS_C_ADC          3
#define BOARD_CS_SHUNT_OHM      ((float)(1.0))
#define BOARD_CS_AMP_GAIN       ((float)(1.35/750))
#define BOARD_CS_INVERTED       0

/* Mesure de tension */
#define BOARD_VREF_V            3.3f
#define BOARD_VBUS_R_TOP_OHM    100000.0f
#define BOARD_VBUS_R_BOT_OHM    2700.0f
#define BOARD_VPH_R_TOP_OHM     BOARD_VBUS_R_TOP_OHM   /* [V] VESC utilise le diviseur VIN */
#define BOARD_VPH_R_BOT_OHM     BOARD_VBUS_R_BOT_OHM
#define BOARD_VBUS_CH           13
#define BOARD_VBUS_ADC          3
#define BOARD_VPH_A_CH          0
#define BOARD_VPH_A_ADC         1
#define BOARD_VPH_B_CH          1
#define BOARD_VPH_B_ADC         2
#define BOARD_VPH_C_CH          2
#define BOARD_VPH_C_ADC         3
#define BOARD_PHASE_FILTER_PIN  HW_PC13

/* Temperatures */
#define BOARD_TFET_MODEL        HW_TEMP_MAX_OF_3
#define BOARD_TFET_NTC_POS      HW_TEMP_NTC_TO_VREF
#define BOARD_TFET2_CH          15
#define BOARD_TFET3_CH          14
#define BOARD_TFET_CH           14
#define BOARD_TFET_NTC_R25_OHM  10000.0f
#define BOARD_TFET_NTC_BETA     3380.0f
#define BOARD_TFET_R_FIXED_OHM  10000.0f
#define BOARD_TMOT_CH           9

/* Annexes et communications */
#define BOARD_ADC_EXT1_CH       3
#define BOARD_ADC_EXT2_CH       4
#define BOARD_ADC_EXT3_CH       7
#define BOARD_LED_GREEN_PIN     HW_PC9
#define BOARD_LED_RED_PIN       HW_PB2
#define BOARD_PWR_HOLD_PIN      HW_PIN_NONE
#define BOARD_SENSOR_5V_PIN     HW_PIN_NONE
#define BOARD_HALL1_PIN         HW_PC6
#define BOARD_HALL2_PIN         HW_PC7
#define BOARD_HALL3_PIN         HW_PC8
#define BOARD_SERVO_PIN         HW_PB6
#define BOARD_SPI_SCK_PIN       HW_PA5
#define BOARD_SPI_MISO_PIN      HW_PA6
#define BOARD_SPI_MOSI_PIN      HW_PA7
#define BOARD_SPI_NSS_PIN       HW_PA4
#define BOARD_I2C_SCL_PIN       HW_PB10
#define BOARD_I2C_SDA_PIN       HW_PB11
#define BOARD_ENC_TIM           3
#define BOARD_UART_INST         3
#define BOARD_UART_TX_PIN       HW_PB10
#define BOARD_UART_RX_PIN       HW_PB11

/* Limites absolues (HW_LIM_* VESC) */
#define HWLIM_I_MOTOR_A         500
#define HWLIM_I_INPUT_A         250
#define HWLIM_I_ABS_TRIP_A      600
#define HWLIM_VBUS_MIN_V        18
#define HWLIM_VBUS_MAX_V        115
#define HWLIM_ERPM              200000
#define HWLIM_DUTY_MAX_PERMILLE 980
#define HWLIM_T_FET_C           90

/* Defauts (MCCONF_* de la carte, sinon mcconf_default.h de VESC 7.00) */
#define DEF_F_PWM_HZ            15000   /* f_zv / 2 */
#define DEF_I_ABS_TRIP_A        550
#define DEF_I_INPUT_MAX_A       250
#define DEF_I_INPUT_MIN_A       (-250)
#define DEF_VBUS_MIN_V          18
#define DEF_VBUS_MAX_V          110

#endif /* BOARD_TRONIC_HW_X12_PRO24_H */
