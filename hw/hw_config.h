/*
 * hw_config.h — Point d'entree unique de la configuration materielle.
 *
 * Selection de la carte a la compilation (meme principe que HW_HEADER de VESC) :
 *     -DRESC_BOARD_HEADER='"boards/ubox/board_ubox_single_85_200.h"'
 * ou via CMake : -DRESC_BOARD=ubox/board_ubox_single_85_200
 *
 * Ordre d'inclusion :
 *   1. hw_types.h                       constantes (broches, types de driver...)
 *   2. RESC_BOARD_HEADER                description de la carte (variante -> core)
 *   3. board_common_f405.h         brochage commun VESC (#ifndef : la carte gagne)
 *   4. hw_check.h                       verifications de compilation (#error)
 *
 * Aucun algorithme (FOC, observateurs, protections) ne lit autre chose que ces
 * macros : changer de carte = changer RESC_BOARD_HEADER, rien d'autre.
 *
 * ===========================================================================
 * CONTRAT D'UNE DESCRIPTION DE CARTE (prefixes)
 * ===========================================================================
 *  BOARD_*   faits materiels (broches, composants, valeurs). Obligatoires sauf
 *            si un defaut commun existe dans board_common_f405.h.
 *            Les valeurs ANALOGIQUES (shunt, gain, diviseurs, VREF, NTC) et la
 *            protection n'ont JAMAIS de defaut : la carte doit les definir.
 *  HWLIM_*   limites ABSOLUES de la carte. Aucun parametre utilisateur ne peut
 *            les depasser (bornage + refus de configuration).
 *  DEF_*     valeurs par defaut des parametres utilisateur (premier demarrage,
 *            reinitialisation). Toujours a l'interieur des HWLIM_*.
 *
 *  Identite
 *    BOARD_NAME              chaine renvoyee a VESC Tool (COMM_FW_VERSION). Garder
 *                            le HW_NAME VESC pour la compatibilite.
 *    BOARD_HW_MAJOR/MINOR
 *    BOARD_STATUS            HW_BOARD_STATUS_IMPORTED / REVIEWED / VALIDATED
 *    BOARD_ORIGIN            chaine : fichier VESC source ou schema de reference
 *
 *  Etage de puissance
 *    BOARD_PWM_{AH,BH,CH,AL,BL,CL}_PIN, BOARD_PWM_{HS,LS}_ACTIVE_HIGH
 *    BOARD_DEADTIME_NS       dead-time materiel insere par TIM1 (DTG)
 *    BOARD_GD_INTERNAL_DEADTIME  1 si le driver insere lui-meme un temps mort
 *    BOARD_GATE_DRIVER       HW_GD_*
 *    BOARD_GD_EN_PIN, BOARD_GD_EN_ACTIVE_HIGH
 *    BOARD_GD_FAULT_PIN, BOARD_GD_FAULT_ACTIVE_LOW
 *    BOARD_GD_SPI_{SCK,MISO,MOSI,CS}_PIN   (drivers SPI)
 *    BOARD_GD_DCCAL_PIN      (DRV8302/8301)
 *
 *  Protection materielle (doc 07)
 *    BOARD_BKIN_PIN          HW_PB12, HW_PA6 ou HW_PIN_NONE
 *    BOARD_BKIN_ACTIVE_HIGH
 *    BOARD_HW_OCP            resume de la protection independante du MCU :
 *                            0 = aucune connue (logiciel uniquement)
 *                            1 = protection interne du driver (VDS) non remontee
 *                            2 = defaut remonte sur GPIO (EXTI)
 *                            3 = defaut remonte sur BKIN (coupure sans CPU)
 *
 *  Mesure de courant
 *    BOARD_CS_TOPOLOGY       HW_CS_*
 *    BOARD_CS_{A,B,C}_CH     voie ADC (0..15), BOARD_CS_{A,B,C}_ADC (1..3, distincts)
 *    BOARD_CS_SHUNT_OHM, BOARD_CS_AMP_GAIN (V/V), BOARD_CS_INVERTED (0/1)
 *    BOARD_CS_FILTER_PIN     commutation d'un filtre RC (HW_PIN_NONE sinon)
 *
 *  Mesure de tension
 *    BOARD_VREF_V            tension de reference ADC reelle (VDDA)
 *    BOARD_VBUS_CH, BOARD_VBUS_ADC, BOARD_VBUS_R_TOP_OHM, BOARD_VBUS_R_BOT_OHM
 *    BOARD_VPH_{A,B,C}_CH, BOARD_VPH_{A,B,C}_ADC, BOARD_VPH_R_TOP_OHM, BOARD_VPH_R_BOT_OHM
 *    BOARD_PHASE_FILTER_PIN  commutation du filtre des tensions de phase
 *
 *  Temperatures
 *    BOARD_TFET_MODEL        HW_TEMP_*, BOARD_TFET_CH (+ TFET2_CH, TFET3_CH)
 *    BOARD_TFET_NTC_R25_OHM, BOARD_TFET_NTC_BETA, BOARD_TFET_R_FIXED_OHM
 *    BOARD_TMOT_MODEL, BOARD_TMOT_CH, BOARD_TMOT_R_FIXED_OHM, BOARD_TMOT_NTC_R25_OHM
 *    (le beta moteur est un parametre utilisateur)
 *
 *  Entrees/sorties annexes : BOARD_ADC_EXT{1,2,3}_CH, LED, AUX, alimentation
 *  auto-maintenue (BOARD_PWR_HOLD_PIN), bouton (BOARD_PWR_KEY_PIN), etc.
 *
 *  Communications : BOARD_UART_{INST,TX_PIN,RX_PIN}, BOARD_UART2_*,
 *  BOARD_HAS_CAN, BOARD_CAN_{RX,TX}_PIN
 *
 *  Capteurs : BOARD_HALL{1,2,3}_PIN, BOARD_ENC_TIM, BOARD_SERVO_PIN,
 *  BOARD_SPI_{SCK,MISO,MOSI,NSS}_PIN
 *
 *  Limites absolues : HWLIM_I_MOTOR_A, HWLIM_I_INPUT_A, HWLIM_I_ABS_TRIP_A,
 *  HWLIM_VBUS_MIN_V, HWLIM_VBUS_MAX_V, HWLIM_ERPM, HWLIM_DUTY_MAX,
 *  HWLIM_T_FET_C, HWLIM_F_PWM_MIN_HZ, HWLIM_F_PWM_MAX_HZ, HWLIM_T_WIN_MIN_NS
 *
 *  Defauts : DEF_F_PWM_HZ, DEF_I_ABS_TRIP_A, DEF_I_INPUT_MAX_A, DEF_I_INPUT_MIN_A,
 *  DEF_VBUS_MIN_V, DEF_VBUS_MAX_V
 *
 * ===========================================================================
 * POLITIQUE DE SECURITE LIEE AU STATUT (appliquee par safety/, pas ici)
 * ===========================================================================
 *  IMPORTED  : pont INTERDIT, sauf build -DRESC_ALLOW_UNREVIEWED_BOARD=1 (banc).
 *  REVIEWED  : pont autorise, courant plafonne a HW_UNREVIEWED_I_FRACTION
 *              des HWLIM_* tant que BOARD_STATUS != VALIDATED.
 *  VALIDATED : pleine plage.
 */
#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "hw_types.h"

#ifndef RESC_BOARD_HEADER
#error "RESC_BOARD_HEADER non defini : choisir une carte (ex. -DRESC_BOARD_HEADER='\"boards/ubox/board_ubox_single_85_200.h\"')"
#endif

#include RESC_BOARD_HEADER
#include "boards/common/board_common_f405.h"
#include "hw_check.h"

/* Fraction des limites autorisee tant que la carte n'est pas validee au banc */
#ifndef HW_UNREVIEWED_I_FRACTION
#define HW_UNREVIEWED_I_FRACTION  0.25f
#endif

#endif /* HW_CONFIG_H */
