/*
 * hw_types.h — Types et constantes communs a la couche materielle.
 *
 * Tout est en macros objet (pas de macros-fonctions) pour etre utilisable
 * dans les #if des verifications de compilation (hw_check.h).
 */
#ifndef HW_TYPES_H
#define HW_TYPES_H

/* ------------------------------------------------------------------------
 * Broches : code = (port << 4) | numero. HW_PIN_NONE = non cablee.
 * LQFP64 (STM32F405RGT6) : PA0..PA15, PB0..PB15, PC0..PC15, PD2 seulement.
 * ---------------------------------------------------------------------- */
#define HW_PIN_NONE   0xFF
#define HW_PIN_PORT(p)  ((p) >> 4)
#define HW_PIN_NUM(p)   ((p) & 0x0F)

/* Port A */
#define HW_PA0   0x00
#define HW_PA1   0x01
#define HW_PA2   0x02
#define HW_PA3   0x03
#define HW_PA4   0x04
#define HW_PA5   0x05
#define HW_PA6   0x06
#define HW_PA7   0x07
#define HW_PA8   0x08
#define HW_PA9   0x09
#define HW_PA10  0x0A
#define HW_PA11  0x0B
#define HW_PA12  0x0C
#define HW_PA13  0x0D
#define HW_PA14  0x0E
#define HW_PA15  0x0F
/* Port B */
#define HW_PB0   0x10
#define HW_PB1   0x11
#define HW_PB2   0x12
#define HW_PB3   0x13
#define HW_PB4   0x14
#define HW_PB5   0x15
#define HW_PB6   0x16
#define HW_PB7   0x17
#define HW_PB8   0x18
#define HW_PB9   0x19
#define HW_PB10  0x1A
#define HW_PB11  0x1B
#define HW_PB12  0x1C
#define HW_PB13  0x1D
#define HW_PB14  0x1E
#define HW_PB15  0x1F
/* Port C */
#define HW_PC0   0x20
#define HW_PC1   0x21
#define HW_PC2   0x22
#define HW_PC3   0x23
#define HW_PC4   0x24
#define HW_PC5   0x25
#define HW_PC6   0x26
#define HW_PC7   0x27
#define HW_PC8   0x28
#define HW_PC9   0x29
#define HW_PC10  0x2A
#define HW_PC11  0x2B
#define HW_PC12  0x2C
#define HW_PC13  0x2D
#define HW_PC14  0x2E
#define HW_PC15  0x2F
/* Port D */
#define HW_PD0   0x30
#define HW_PD1   0x31
#define HW_PD2   0x32
#define HW_PD3   0x33
#define HW_PD4   0x34
#define HW_PD5   0x35
#define HW_PD6   0x36
#define HW_PD7   0x37
#define HW_PD8   0x38
#define HW_PD9   0x39
#define HW_PD10  0x3A
#define HW_PD11  0x3B
#define HW_PD12  0x3C
#define HW_PD13  0x3D
#define HW_PD14  0x3E
#define HW_PD15  0x3F
/* Port E */
#define HW_PE0   0x40
#define HW_PE1   0x41
#define HW_PE2   0x42
#define HW_PE3   0x43
#define HW_PE4   0x44
#define HW_PE5   0x45
#define HW_PE6   0x46
#define HW_PE7   0x47
#define HW_PE8   0x48
#define HW_PE9   0x49
#define HW_PE10  0x4A
#define HW_PE11  0x4B
#define HW_PE12  0x4C
#define HW_PE13  0x4D
#define HW_PE14  0x4E
#define HW_PE15  0x4F

/* Broche presente sur le boitier LQFP64 ? (utilise par hw_check.h) */
#define HW_PIN_ON_LQFP64(p) ((p) == HW_PIN_NONE || (p) <= HW_PC15 || (p) == HW_PD2)

/* ------------------------------------------------------------------------
 * ADC : voies externes IN0..IN15. Voies speciales du seul ADC1.
 * Contrainte [C, DS8626] : IN0-3 et IN10-13 sont sur ADC1/2/3 ;
 * IN4-9, IN14, IN15 seulement sur ADC1/2.
 * ---------------------------------------------------------------------- */
#define HW_ADC_CH_NONE     0xFF
#define HW_ADC_CH_TEMPSENS 16   /* ADC1 uniquement */
#define HW_ADC_CH_VREFINT  17   /* ADC1 uniquement */
#define HW_ADC_CH_ON_ADC3(ch) ((ch) <= 3 || ((ch) >= 10 && (ch) <= 13))

/* ------------------------------------------------------------------------
 * Etat de validation d'une description de carte.
 * Politique (appliquee par le firmware, voir hw_config.h) :
 *   IMPORTED  : genere automatiquement depuis VESC. Etage de puissance
 *               INTERDIT sauf build RESC_ALLOW_UNREVIEWED_BOARD (banc).
 *   REVIEWED  : relu a la main contre les sources VESC/schema. Puissance
 *               autorisee, courant plafonne a HW_UNVALIDATED_I_FRACTION.
 *   VALIDATED : valide au banc (mesures doc 10 phases 3-4). Pleine plage.
 * ---------------------------------------------------------------------- */
#define HW_BOARD_STATUS_IMPORTED   0
#define HW_BOARD_STATUS_REVIEWED   1
#define HW_BOARD_STATUS_VALIDATED  2

/* Gate drivers */
#define HW_GD_GENERIC     0   /* discret ou sans interface : EN/FAULT seulement */
#define HW_GD_DRV8302     1   /* sans SPI, OC_ADJ materiel, DC_CAL */
#define HW_GD_DRV8301     2   /* SPI */
#define HW_GD_DRV8305     3   /* SPI */
#define HW_GD_DRV8320S    4   /* SPI */
#define HW_GD_DRV8323S    5   /* SPI, amplis de shunt integres */
#define HW_GD_DRV8313     6   /* pont integre basse puissance */

/* Topologie de mesure du courant */
#define HW_CS_LOWSIDE_2   0   /* 2 shunts low-side */
#define HW_CS_LOWSIDE_3   1   /* 3 shunts low-side (fenetres, doc 04) */
#define HW_CS_INLINE_3    2   /* shunts de phase en ligne (mesure continue) */

/* Modele de sonde de temperature */
#define HW_TEMP_NONE        0
#define HW_TEMP_NTC_TO_VREF 1  /* NTC entre VREF et l'entree, R fixe vers GND */
#define HW_TEMP_NTC_TO_GND  2  /* R fixe vers VREF, NTC vers GND */
#define HW_TEMP_MAX_OF_3    3  /* max de 3 NTC (TFET, TFET2, TFET3), meme modele */

#endif /* HW_TYPES_H */
