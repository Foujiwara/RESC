/*
 * fault_codes.h — Codes de defaut RESC.
 * Les valeurs numeriques sont celles du protocole (mc_fault_code) pour que les
 * ecrans, maitres CAN et VESC Tool les affichent correctement.
 */
#ifndef FAULT_CODES_H
#define FAULT_CODES_H

#define RESC_FAULT_NONE                 0
#define RESC_FAULT_OVER_VOLTAGE         1
#define RESC_FAULT_UNDER_VOLTAGE        2
#define RESC_FAULT_DRV                  3
#define RESC_FAULT_ABS_OVER_CURRENT     4
#define RESC_FAULT_OVER_TEMP_FET        5
#define RESC_FAULT_OVER_TEMP_MOTOR      6
#define RESC_FAULT_WATCHDOG_RESET       10
#define RESC_FAULT_FLASH_CORRUPTION     14
#define RESC_FAULT_HIGH_OFFSET_CURR_1   15
#define RESC_FAULT_HIGH_OFFSET_CURR_2   16
#define RESC_FAULT_HIGH_OFFSET_CURR_3   17
#define RESC_FAULT_UNBALANCED_CURRENTS  18
#define RESC_FAULT_BRK                  19
#define RESC_FAULT_OVERSPEED            31
#define RESC_FAULT_UNDERSPEED           32   /* RESC : echec du demarrage / perte de synchro */
#define RESC_FAULT_ABS_OVERSPEED        33

#endif /* FAULT_CODES_H */
