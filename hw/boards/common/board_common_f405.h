/*
 * board_common_f405.h — Brochage commun « de facto » des cartes VESC
 * a base de STM32F405RGT6 (LQFP64).
 *
 * Origine : recensement automatique de bldc/hwconf (release_7_00), 172
 * configurations, dont 154 en LQFP64, mono-moteur, avec TIM1 sur
 * PA8-PA10/PB13-PB15. Outil : tools/analysis/vesc_hw_survey.py,
 * donnees : docs/data/vesc_hw_survey.csv. Les comptages ci-dessous portent
 * sur l'ensemble des 172 configurations.
 *
 * Regles :
 *  - Chaque valeur est un DEFAUT (#ifndef) : la carte la remplace si besoin.
 *  - Seuls les BROCHAGES et les affectations ADC ont un defaut ici.
 *  - Les valeurs analogiques, les limites, le dead-time et l'etat de la
 *    protection n'ont volontairement aucun defaut (hw_check.h les exige).
 */
#ifndef BOARD_COMMON_F405_H
#define BOARD_COMMON_F405_H

/* --- Pont triphase : TIM1, 156/172 cartes (les autres sont en LQFP100) --- */
#ifndef BOARD_PWM_AH_PIN
#define BOARD_PWM_AH_PIN        HW_PA8    /* TIM1_CH1  AF1 */
#endif
#ifndef BOARD_PWM_BH_PIN
#define BOARD_PWM_BH_PIN        HW_PA9    /* TIM1_CH2  AF1 */
#endif
#ifndef BOARD_PWM_CH_PIN
#define BOARD_PWM_CH_PIN        HW_PA10   /* TIM1_CH3  AF1 */
#endif
#ifndef BOARD_PWM_AL_PIN
#define BOARD_PWM_AL_PIN        HW_PB13   /* TIM1_CH1N AF1 */
#endif
#ifndef BOARD_PWM_BL_PIN
#define BOARD_PWM_BL_PIN        HW_PB14   /* TIM1_CH2N AF1 */
#endif
#ifndef BOARD_PWM_CL_PIN
#define BOARD_PWM_CL_PIN        HW_PB15   /* TIM1_CH3N AF1 */
#endif
/* Aucune carte VESC 7.00 ne definit INVERTED_TOP/BOTTOM_DRIVER_INPUT */
#ifndef BOARD_PWM_HS_ACTIVE_HIGH
#define BOARD_PWM_HS_ACTIVE_HIGH 1
#endif
#ifndef BOARD_PWM_LS_ACTIVE_HIGH
#define BOARD_PWM_LS_ACTIVE_HIGH 1
#endif
#ifndef BOARD_GD_INTERNAL_DEADTIME
#define BOARD_GD_INTERNAL_DEADTIME 0
#endif

/* --- Gate driver : pas de defaut commun (EN PB5 : 22 cartes, PC10 : 11) --- */
#ifndef BOARD_GD_EN_PIN
#define BOARD_GD_EN_PIN         HW_PIN_NONE
#endif
#ifndef BOARD_GD_EN_ACTIVE_HIGH
#define BOARD_GD_EN_ACTIVE_HIGH 1
#endif
#ifndef BOARD_GD_FAULT_PIN
#define BOARD_GD_FAULT_PIN      HW_PIN_NONE
#endif
#ifndef BOARD_GD_FAULT_ACTIVE_LOW
#define BOARD_GD_FAULT_ACTIVE_LOW 1
#endif
#ifndef BOARD_GD_SPI_SCK_PIN
#define BOARD_GD_SPI_SCK_PIN    HW_PIN_NONE
#endif
#ifndef BOARD_GD_SPI_MISO_PIN
#define BOARD_GD_SPI_MISO_PIN   HW_PIN_NONE
#endif
#ifndef BOARD_GD_SPI_MOSI_PIN
#define BOARD_GD_SPI_MOSI_PIN   HW_PIN_NONE
#endif
#ifndef BOARD_GD_SPI_CS_PIN
#define BOARD_GD_SPI_CS_PIN     HW_PIN_NONE
#endif
#ifndef BOARD_GD_DCCAL_PIN
#define BOARD_GD_DCCAL_PIN      HW_PIN_NONE
#endif

/* --- BKIN : cable sur 7/172 cartes seulement (toujours PB12) --- */
#ifndef BOARD_BKIN_PIN
#define BOARD_BKIN_PIN          HW_PIN_NONE
#endif
#ifndef BOARD_BKIN_ACTIVE_HIGH
#define BOARD_BKIN_ACTIVE_HIGH  0
#endif

/* --- Courants : IN10/IN11/IN12 (PC0/PC1/PC2) sur ADC1/2/3, ~134 cartes --- */
#ifndef BOARD_CS_A_CH
#define BOARD_CS_A_CH           10
#endif
#ifndef BOARD_CS_A_ADC
#define BOARD_CS_A_ADC          1
#endif
#ifndef BOARD_CS_B_CH
#define BOARD_CS_B_CH           11
#endif
#ifndef BOARD_CS_B_ADC
#define BOARD_CS_B_ADC          2
#endif
#ifndef BOARD_CS_C_CH
#define BOARD_CS_C_CH           12
#endif
#ifndef BOARD_CS_C_ADC
#define BOARD_CS_C_ADC          3
#endif
#ifndef BOARD_CS_FILTER_PIN
#define BOARD_CS_FILTER_PIN     HW_PIN_NONE
#endif

/* --- Tensions de phase : IN0/IN1/IN2 (PA0/PA1/PA2), ~141 cartes --- */
#ifndef BOARD_VPH_A_CH
#define BOARD_VPH_A_CH          0
#endif
#ifndef BOARD_VPH_A_ADC
#define BOARD_VPH_A_ADC         1
#endif
#ifndef BOARD_VPH_B_CH
#define BOARD_VPH_B_CH          1
#endif
#ifndef BOARD_VPH_B_ADC
#define BOARD_VPH_B_ADC         2
#endif
#ifndef BOARD_VPH_C_CH
#define BOARD_VPH_C_CH          2
#endif
#ifndef BOARD_VPH_C_ADC
#define BOARD_VPH_C_ADC         3
#endif
#ifndef BOARD_PHASE_FILTER_PIN
#define BOARD_PHASE_FILTER_PIN  HW_PIN_NONE
#endif

/* --- Bus : IN13 (PC3) sur ADC3, 141 cartes --- */
#ifndef BOARD_VBUS_CH
#define BOARD_VBUS_CH           13
#endif
#ifndef BOARD_VBUS_ADC
#define BOARD_VBUS_ADC          3
#endif

/* --- Temperatures : TFET IN3 (PA3) 121 cartes ; moteur IN14 (PC4) 119 --- */
#ifndef BOARD_TFET_CH
#define BOARD_TFET_CH           3
#endif
#ifndef BOARD_TFET2_CH
#define BOARD_TFET2_CH          HW_ADC_CH_NONE
#endif
#ifndef BOARD_TFET3_CH
#define BOARD_TFET3_CH          HW_ADC_CH_NONE
#endif
#ifndef BOARD_TMOT_CH
#define BOARD_TMOT_CH           14
#endif
#ifndef BOARD_TMOT_MODEL
#define BOARD_TMOT_MODEL        HW_TEMP_NTC_TO_GND   /* « motor temp sensor on low side » */
#endif
#ifndef BOARD_TMOT_R_FIXED_OHM
#define BOARD_TMOT_R_FIXED_OHM  10000.0f
#endif
#ifndef BOARD_TMOT_NTC_R25_OHM
#define BOARD_TMOT_NTC_R25_OHM  10000.0f
#endif

/* --- ADC du port COMM : EXT1 IN5 (PA5) 121, EXT2 IN6 (PA6) 135 --- */
#ifndef BOARD_ADC_EXT1_CH
#define BOARD_ADC_EXT1_CH       5
#endif
#ifndef BOARD_ADC_EXT2_CH
#define BOARD_ADC_EXT2_CH       6
#endif
#ifndef BOARD_ADC_EXT3_CH
#define BOARD_ADC_EXT3_CH       HW_ADC_CH_NONE
#endif

/* --- UART VESC : USART3 PB10/PB11, 146 cartes --- */
#ifndef BOARD_UART_INST
#define BOARD_UART_INST         3
#endif
#ifndef BOARD_UART_TX_PIN
#define BOARD_UART_TX_PIN       HW_PB10
#endif
#ifndef BOARD_UART_RX_PIN
#define BOARD_UART_RX_PIN       HW_PB11
#endif
/* UART permanent (module NRF/BLE) : aucun par defaut (UART4 PC10/PC11 frequent) */
#ifndef BOARD_UART2_INST
#define BOARD_UART2_INST        0
#endif
#ifndef BOARD_UART2_TX_PIN
#define BOARD_UART2_TX_PIN      HW_PIN_NONE
#endif
#ifndef BOARD_UART2_RX_PIN
#define BOARD_UART2_RX_PIN      HW_PIN_NONE
#endif
/* I2C2 sur les memes broches que l'UART (149 cartes) : usage exclusif */
/* IMU interne (I2C logiciel) : aucune par defaut */
#ifndef BOARD_IMU_SDA_PIN
#define BOARD_IMU_SDA_PIN       HW_PIN_NONE
#endif
#ifndef BOARD_IMU_SCL_PIN
#define BOARD_IMU_SCL_PIN       HW_PIN_NONE
#endif
#ifndef BOARD_IMU_FLIP
#define BOARD_IMU_FLIP          0
#endif
#ifndef BOARD_I2C_SCL_PIN
#define BOARD_I2C_SCL_PIN       BOARD_UART_TX_PIN
#endif
#ifndef BOARD_I2C_SDA_PIN
#define BOARD_I2C_SDA_PIN       BOARD_UART_RX_PIN
#endif

/* --- CAN1 PB8/PB9 (defaut de hw.h VESC, 157 cartes) --- */
#ifndef BOARD_HAS_CAN
#define BOARD_HAS_CAN           1
#endif
#ifndef BOARD_CAN_RX_PIN
#define BOARD_CAN_RX_PIN        HW_PB8    /* CAN1_RX AF9 */
#endif
#ifndef BOARD_CAN_TX_PIN
#define BOARD_CAN_TX_PIN        HW_PB9    /* CAN1_TX AF9 */
#endif

/* --- Halls / codeur ABI : PC6/PC7/PC8, TIM3, 148 cartes --- */
#ifndef BOARD_HALL1_PIN
#define BOARD_HALL1_PIN         HW_PC6
#endif
#ifndef BOARD_HALL2_PIN
#define BOARD_HALL2_PIN         HW_PC7
#endif
#ifndef BOARD_HALL3_PIN
#define BOARD_HALL3_PIN         HW_PC8
#endif
#ifndef BOARD_ENC_TIM
#define BOARD_ENC_TIM           3
#endif

/* --- Entree servo/PPM : PB6 (TIM4_CH1), 139 cartes --- */
#ifndef BOARD_SERVO_PIN
#define BOARD_SERVO_PIN         HW_PB6
#endif

/* --- SPI1 du port capteur : PA5/PA6/PA7 (132), NSS PA4 (95) ou PB11 (66) --- */
#ifndef BOARD_SPI_SCK_PIN
#define BOARD_SPI_SCK_PIN       HW_PA5
#endif
#ifndef BOARD_SPI_MISO_PIN
#define BOARD_SPI_MISO_PIN      HW_PA6
#endif
#ifndef BOARD_SPI_MOSI_PIN
#define BOARD_SPI_MOSI_PIN      HW_PA7
#endif
#ifndef BOARD_SPI_NSS_PIN
#define BOARD_SPI_NSS_PIN       HW_PA4
#endif

/* --- Divers : pas de defaut commun --- */
#ifndef BOARD_LED_GREEN_PIN
#define BOARD_LED_GREEN_PIN     HW_PIN_NONE
#endif
#ifndef BOARD_LED_RED_PIN
#define BOARD_LED_RED_PIN       HW_PIN_NONE
#endif
#ifndef BOARD_AUX_PIN
#define BOARD_AUX_PIN           HW_PIN_NONE
#endif
#ifndef BOARD_PWR_HOLD_PIN
#define BOARD_PWR_HOLD_PIN      HW_PIN_NONE   /* auto-maintien de l'alimentation */
#endif
#ifndef BOARD_PWR_KEY_PIN
#define BOARD_PWR_KEY_PIN       HW_PIN_NONE
#endif
#ifndef BOARD_SENSOR_5V_PIN
#define BOARD_SENSOR_5V_PIN     HW_PIN_NONE
#endif

/* --- Defauts de parametres communs --- */
#ifndef HWLIM_F_PWM_MIN_HZ
#define HWLIM_F_PWM_MIN_HZ      8000
#endif
#ifndef HWLIM_F_PWM_MAX_HZ
#define HWLIM_F_PWM_MAX_HZ      40000
#endif
/* Fenetre ADC minimale : valeur prudente tant qu'elle n'est pas mesuree (doc 04 §4.4) */
#ifndef HWLIM_T_WIN_MIN_NS
#define HWLIM_T_WIN_MIN_NS      3000
#endif

#endif /* BOARD_COMMON_F405_H */
