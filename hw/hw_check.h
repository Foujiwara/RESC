/*
 * hw_check.h — Verifications de coherence d'une description de carte,
 * a la compilation. Inclus uniquement par hw_config.h.
 *
 * Le preprocesseur ne compare que des entiers : les valeurs flottantes
 * (shunt, gain, VREF, resistances, NTC) sont verifiees au demarrage par
 * hw_validate() (a ecrire dans hw/bsp/, doc 09) :
 *   - plage mesurable (VREF/2)/(R_shunt*G) > HWLIM_I_ABS_TRIP_A * 1.1
 *   - pleine echelle Vbus > HWLIM_VBUS_MAX_V * 1.1
 *   - resistances, beta, gain > 0
 * Les doublons de broches sont verifies par tools/hw_import/check_boards.py.
 */
#ifndef HW_CHECK_H
#define HW_CHECK_H

/* ---------------- Presence des definitions obligatoires ---------------- */
#if !defined(BOARD_NAME) || !defined(BOARD_HW_MAJOR) || !defined(BOARD_HW_MINOR) || !defined(BOARD_ORIGIN)
#error "Carte : identite incomplete (BOARD_NAME, BOARD_HW_MAJOR, BOARD_HW_MINOR, BOARD_ORIGIN)"
#endif
#ifndef BOARD_STATUS
#error "Carte : BOARD_STATUS obligatoire"
#endif
#if !defined(BOARD_GATE_DRIVER) || !defined(BOARD_DEADTIME_NS) || !defined(BOARD_HW_OCP)
#error "Carte : BOARD_GATE_DRIVER, BOARD_DEADTIME_NS et BOARD_HW_OCP sont obligatoires"
#endif
#if !defined(BOARD_CS_TOPOLOGY) || !defined(BOARD_CS_SHUNT_OHM) || !defined(BOARD_CS_AMP_GAIN) || !defined(BOARD_CS_INVERTED)
#error "Carte : mesure de courant incomplete (BOARD_CS_TOPOLOGY/SHUNT_OHM/AMP_GAIN/INVERTED)"
#endif
#if !defined(BOARD_VREF_V) || !defined(BOARD_VBUS_R_TOP_OHM) || !defined(BOARD_VBUS_R_BOT_OHM) || !defined(BOARD_VPH_R_TOP_OHM) || !defined(BOARD_VPH_R_BOT_OHM)
#error "Carte : mesures de tension incompletes (BOARD_VREF_V, diviseurs Vbus et phases)"
#endif
#if !defined(BOARD_TFET_MODEL) || !defined(BOARD_TFET_NTC_R25_OHM) || !defined(BOARD_TFET_NTC_BETA) || !defined(BOARD_TFET_R_FIXED_OHM)
#error "Carte : sonde de temperature FET incomplete"
#endif
#if !defined(HWLIM_I_MOTOR_A) || !defined(HWLIM_I_INPUT_A) || !defined(HWLIM_I_ABS_TRIP_A) || !defined(HWLIM_VBUS_MIN_V) || !defined(HWLIM_VBUS_MAX_V) || !defined(HWLIM_ERPM) || !defined(HWLIM_DUTY_MAX_PERMILLE) || !defined(HWLIM_T_FET_C)
#error "Carte : limites absolues HWLIM_* incompletes"
#endif
#if !defined(DEF_F_PWM_HZ) || !defined(DEF_I_ABS_TRIP_A) || !defined(DEF_I_INPUT_MAX_A) || !defined(DEF_I_INPUT_MIN_A) || !defined(DEF_VBUS_MIN_V) || !defined(DEF_VBUS_MAX_V)
#error "Carte : valeurs par defaut DEF_* incompletes"
#endif

/* ---------------- Enumerations ---------------- */
#if BOARD_STATUS < HW_BOARD_STATUS_IMPORTED || BOARD_STATUS > HW_BOARD_STATUS_VALIDATED
#error "BOARD_STATUS invalide"
#endif
#if BOARD_GATE_DRIVER < HW_GD_GENERIC || BOARD_GATE_DRIVER > HW_GD_DRV8313
#error "BOARD_GATE_DRIVER invalide"
#endif
#if BOARD_CS_TOPOLOGY < HW_CS_LOWSIDE_2 || BOARD_CS_TOPOLOGY > HW_CS_INLINE_3
#error "BOARD_CS_TOPOLOGY invalide"
#endif
#if BOARD_HW_OCP < 0 || BOARD_HW_OCP > 3
#error "BOARD_HW_OCP invalide (0..3)"
#endif

/* ---------------- Pont et dead-time ---------------- */
#if BOARD_PWM_AH_PIN != HW_PA8 || BOARD_PWM_BH_PIN != HW_PA9 || BOARD_PWM_CH_PIN != HW_PA10
#error "TIM1 CH1..3 doivent etre sur PA8/PA9/PA10 en LQFP64"
#endif
#if !((BOARD_PWM_AL_PIN == HW_PB13 && BOARD_PWM_BL_PIN == HW_PB14 && BOARD_PWM_CL_PIN == HW_PB15) || \
      (BOARD_PWM_AL_PIN == HW_PA7  && BOARD_PWM_BL_PIN == HW_PB0  && BOARD_PWM_CL_PIN == HW_PB1))
#error "TIM1 CH1N..3N : PB13/PB14/PB15 ou PA7/PB0/PB1 uniquement"
#endif
/* DTG a 168 MHz : 5.95 ns de pas, ~6 us max (RM0090) */
#if BOARD_GD_INTERNAL_DEADTIME
#  if BOARD_DEADTIME_NS < 0 || BOARD_DEADTIME_NS > 6000
#  error "BOARD_DEADTIME_NS hors plage"
#  endif
#else
#  if BOARD_DEADTIME_NS < 50 || BOARD_DEADTIME_NS > 6000
#  error "BOARD_DEADTIME_NS < 50 ns sans temps mort interne au driver : conduction croisee possible"
#  endif
#endif

/* ---------------- Gate driver ---------------- */
#if BOARD_GATE_DRIVER == HW_GD_DRV8301 || BOARD_GATE_DRIVER == HW_GD_DRV8305 || \
    BOARD_GATE_DRIVER == HW_GD_DRV8320S || BOARD_GATE_DRIVER == HW_GD_DRV8323S
#  if BOARD_GD_SPI_SCK_PIN == HW_PIN_NONE || BOARD_GD_SPI_MISO_PIN == HW_PIN_NONE || \
      BOARD_GD_SPI_MOSI_PIN == HW_PIN_NONE || BOARD_GD_SPI_CS_PIN == HW_PIN_NONE
#  error "Driver SPI sans broches SPI"
#  endif
#endif

/* ---------------- Protection ---------------- */
#if BOARD_BKIN_PIN != HW_PIN_NONE && BOARD_BKIN_PIN != HW_PB12 && BOARD_BKIN_PIN != HW_PA6
#error "TIM1_BKIN n'existe que sur PB12 ou PA6 en LQFP64"
#endif
#if BOARD_HW_OCP == 3 && BOARD_BKIN_PIN == HW_PIN_NONE
#error "BOARD_HW_OCP == 3 (defaut sur BKIN) mais BOARD_BKIN_PIN non cable"
#endif
#if BOARD_HW_OCP == 2 && BOARD_GD_FAULT_PIN == HW_PIN_NONE
#error "BOARD_HW_OCP == 2 (defaut sur GPIO) mais BOARD_GD_FAULT_PIN non cable"
#endif
#if BOARD_BKIN_PIN == HW_PA6 && (BOARD_SPI_MISO_PIN == HW_PA6 || BOARD_ADC_EXT2_CH == 6)
#error "BKIN sur PA6 partage avec SPI_MISO ou ADC_EXT2 : redefinir ces fonctions a HW_PIN_NONE / HW_ADC_CH_NONE"
#endif

/* ---------------- ADC : courants ---------------- */
#if BOARD_CS_A_ADC < 1 || BOARD_CS_A_ADC > 3 || BOARD_CS_B_ADC < 1 || BOARD_CS_B_ADC > 3
#error "BOARD_CS_A/B_ADC : 1..3"
#endif
#if BOARD_CS_A_ADC == BOARD_CS_B_ADC
#error "Courants A et B sur le meme ADC : echantillonnage simultane impossible"
#endif
#if (BOARD_CS_A_ADC == 3 && !HW_ADC_CH_ON_ADC3(BOARD_CS_A_CH)) || (BOARD_CS_B_ADC == 3 && !HW_ADC_CH_ON_ADC3(BOARD_CS_B_CH))
#error "Voie de courant non disponible sur ADC3 (seules IN0-3 et IN10-13 le sont)"
#endif
#if BOARD_CS_TOPOLOGY != HW_CS_LOWSIDE_2
#  if BOARD_CS_C_CH == HW_ADC_CH_NONE || BOARD_CS_C_ADC < 1 || BOARD_CS_C_ADC > 3
#  error "3e courant requis par la topologie"
#  endif
#  if BOARD_CS_C_ADC == BOARD_CS_A_ADC || BOARD_CS_C_ADC == BOARD_CS_B_ADC
#  error "Courant C sur le meme ADC que A ou B"
#  endif
#  if BOARD_CS_C_ADC == 3 && !HW_ADC_CH_ON_ADC3(BOARD_CS_C_CH)
#  error "Courant C : voie non disponible sur ADC3"
#  endif
#endif

/* ---------------- ADC : tensions ---------------- */
#if (BOARD_VPH_A_ADC == 3 && !HW_ADC_CH_ON_ADC3(BOARD_VPH_A_CH)) || \
    (BOARD_VPH_B_ADC == 3 && !HW_ADC_CH_ON_ADC3(BOARD_VPH_B_CH)) || \
    (BOARD_VPH_C_ADC == 3 && !HW_ADC_CH_ON_ADC3(BOARD_VPH_C_CH)) || \
    (BOARD_VBUS_ADC == 3 && !HW_ADC_CH_ON_ADC3(BOARD_VBUS_CH))
#error "Tension de phase ou bus : voie non disponible sur ADC3"
#endif
#if BOARD_VPH_A_ADC == BOARD_VPH_B_ADC || BOARD_VPH_A_ADC == BOARD_VPH_C_ADC || BOARD_VPH_B_ADC == BOARD_VPH_C_ADC
#error "Tensions de phase : un ADC par phase requis (mesure simultanee)"
#endif

/* ---------------- Temperatures ---------------- */
#if BOARD_TFET_MODEL == HW_TEMP_MAX_OF_3 && (BOARD_TFET2_CH == HW_ADC_CH_NONE || BOARD_TFET3_CH == HW_ADC_CH_NONE)
#error "HW_TEMP_MAX_OF_3 requiert BOARD_TFET2_CH et BOARD_TFET3_CH"
#endif
#if BOARD_TFET_MODEL == HW_TEMP_MAX_OF_3 && !defined(BOARD_TFET_NTC_POS)
#error "HW_TEMP_MAX_OF_3 requiert BOARD_TFET_NTC_POS (HW_TEMP_NTC_TO_VREF ou HW_TEMP_NTC_TO_GND)"
#endif

/* ---------------- Communications ---------------- */
#if !((BOARD_UART_INST == 3 && ((BOARD_UART_TX_PIN == HW_PB10 && BOARD_UART_RX_PIN == HW_PB11) || (BOARD_UART_TX_PIN == HW_PC10 && BOARD_UART_RX_PIN == HW_PC11))) || \
      (BOARD_UART_INST == 6 && BOARD_UART_TX_PIN == HW_PC6 && BOARD_UART_RX_PIN == HW_PC7) || \
      (BOARD_UART_INST == 1 && ((BOARD_UART_TX_PIN == HW_PA9 && BOARD_UART_RX_PIN == HW_PA10) || (BOARD_UART_TX_PIN == HW_PB6 && BOARD_UART_RX_PIN == HW_PB7))) || \
      (BOARD_UART_INST == 4 && ((BOARD_UART_TX_PIN == HW_PC10 && BOARD_UART_RX_PIN == HW_PC11) || (BOARD_UART_TX_PIN == HW_PA0 && BOARD_UART_RX_PIN == HW_PA1))) || \
      (BOARD_UART_INST == 2 && BOARD_UART_TX_PIN == HW_PA2 && BOARD_UART_RX_PIN == HW_PA3))
#error "UART principal : instance et broches incompatibles"
#endif
#if BOARD_HAS_CAN && !((BOARD_CAN_RX_PIN == HW_PB8 && BOARD_CAN_TX_PIN == HW_PB9) || (BOARD_CAN_RX_PIN == HW_PA11 && BOARD_CAN_TX_PIN == HW_PA12))
#error "CAN1 : PB8/PB9 ou PA11/PA12 uniquement en LQFP64"
#endif
#if BOARD_UART_INST == 1 && BOARD_UART_TX_PIN == HW_PA9
#error "USART1 sur PA9/PA10 entre en conflit avec TIM1 CH2/CH3"
#endif

/* ---------------- Boitier ---------------- */
#if !HW_PIN_ON_LQFP64(BOARD_GD_EN_PIN) || !HW_PIN_ON_LQFP64(BOARD_GD_FAULT_PIN) || !HW_PIN_ON_LQFP64(BOARD_PWR_HOLD_PIN) || \
    !HW_PIN_ON_LQFP64(BOARD_CS_FILTER_PIN) || !HW_PIN_ON_LQFP64(BOARD_PHASE_FILTER_PIN) || !HW_PIN_ON_LQFP64(BOARD_HALL3_PIN)
#error "Broche hors boitier LQFP64 (carte prevue pour un STM32F405 LQFP100 ?)"
#endif

/* ---------------- Limites et defauts ---------------- */
#if HWLIM_VBUS_MIN_V >= HWLIM_VBUS_MAX_V
#error "HWLIM_VBUS_MIN_V >= HWLIM_VBUS_MAX_V"
#endif
#if DEF_VBUS_MIN_V < HWLIM_VBUS_MIN_V || DEF_VBUS_MAX_V > HWLIM_VBUS_MAX_V || DEF_VBUS_MIN_V >= DEF_VBUS_MAX_V
#error "DEF_VBUS_* hors des limites absolues"
#endif
#if HWLIM_I_MOTOR_A > HWLIM_I_ABS_TRIP_A
#error "Le courant moteur max depasse le seuil de coupure absolue"
#endif
#if DEF_I_ABS_TRIP_A > HWLIM_I_ABS_TRIP_A || DEF_I_ABS_TRIP_A <= 0
#error "DEF_I_ABS_TRIP_A hors limites"
#endif
#if DEF_I_INPUT_MAX_A > HWLIM_I_INPUT_A || -(DEF_I_INPUT_MIN_A) > HWLIM_I_INPUT_A
#error "DEF_I_INPUT_* hors limites"
#endif
#if HWLIM_DUTY_MAX_PERMILLE > 1000 || HWLIM_DUTY_MAX_PERMILLE < 500
#error "HWLIM_DUTY_MAX_PERMILLE : 500..1000"
#endif
#if DEF_F_PWM_HZ < HWLIM_F_PWM_MIN_HZ || DEF_F_PWM_HZ > HWLIM_F_PWM_MAX_HZ
#error "DEF_F_PWM_HZ hors de [HWLIM_F_PWM_MIN_HZ, HWLIM_F_PWM_MAX_HZ]"
#endif

/* ---------------- Statut ---------------- */
#if BOARD_STATUS == HW_BOARD_STATUS_IMPORTED && !defined(RESC_ALLOW_UNREVIEWED_BOARD)
#  if defined(__GNUC__)
#  warning "Carte IMPORTED (non relue) : l'etage de puissance restera desactive"
#  endif
#endif

#endif /* HW_CHECK_H */
