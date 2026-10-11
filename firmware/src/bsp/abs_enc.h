/*
 * abs_enc.h — Codeurs de position absolus (m_sensor_port_mode de VESC).
 *
 * Lus a chaque periode FOC, avant le coeur ; l'angle mecanique est rendu en pas de
 * 1/16384 de tour et traite comme un codeur incremental de 16384 pas par tour (rapport,
 * sens, decalage et detection inchanges). Un echantillon invalide (parite, CRC, aimant,
 * signal hors plage) garde l'angle precedent et compte une erreur.
 *
 *   valeur  capteur                    liaison
 *   2       AS5047 / AS5048            SPI logiciel sur le port Hall (H1 SCK, H2 MISO, H3 CS)
 *   3       AD2S1205 (resolveur)       SPI logiciel sur les broches SPI (SCK, MISO, NSS)
 *   4       sin / cos                  entrees ADC 1 (sin) et ADC 2 (cos)
 *   7       MT6816                     SPI1 materiel (mode 3, 16 bits), CS = NSS
 *   8       AS5x47U                    SPI1 materiel (mode 1, trames 24 bits avec CRC8)
 *   10      TLE5012 (SSC)              logiciel sur le port Hall (H1 SCK, H2 donnees, H3 CS)
 *   13      PWM (AS5047 et autres)     TIM4, entree servo (largeur / periode)
 *   15      MA782                      SPI1 materiel (16 bits)
 *   16      AMT22                      SPI logiciel sur le port Hall
 */
#ifndef ABS_ENC_H
#define ABS_ENC_H

#include <stdbool.h>
#include <stdint.h>

#define ABS_ENC_CPR 16384u

typedef struct {
    float s_gain, s_offset;    /* sin : 1 / amplitude (V), decalage (V) */
    float c_gain, c_offset;
    float filter;              /* filtre passe-bas par echantillon (0..1) */
    float sph, cph;            /* correction de phase : sin et cos de l'angle de correction */
} abs_enc_sincos_t;

/* true si le mode designe un codeur absolu pris en charge */
static inline bool abs_enc_supported(uint8_t m)
{
    return m == 2 || m == 3 || m == 4 || m == 7 || m == 8 || m == 10 || m == 13 || m == 15 || m == 16;
}
/* Configure les broches et le peripherique (moteur arrete) ; 0/1 (Hall, ABI) : rien */
void abs_enc_config(uint8_t port_mode, const abs_enc_sincos_t *sc);
/* ISR FOC : lit le capteur, rend l'angle en pas (0..16383) ; false si aucun codeur absolu */
bool abs_enc_sample(uint32_t *counts);
uint32_t abs_enc_errors(void);
float abs_enc_error_rate(void);     /* proportion filtree d'echantillons invalides */

#endif
