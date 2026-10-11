/*
 * sampler.h — Echantillonnage pour VESC Tool (COMM_SAMPLE_PRINT, page « Sampled Data »).
 *
 * L'ISR FOC enregistre a chaque periode (ou une sur 'decimation') les courants et tensions de
 * phase bruts, le point milieu, le courant filtre, l'etat Hall et l'angle de commande. Modes
 * (debug_sampling_mode du protocole) : immediat, au demarrage du moteur, sur declenchement au
 * demarrage ou sur defaut (avec historique avant l'evenement), sans envoi, puis renvoi des
 * derniers echantillons ou d'un seul.
 */
#ifndef SAMPLER_H
#define SAMPLER_H

#include <stdbool.h>
#include <stdint.h>
#include "foc.h"
#include "packet.h"

#define SAMPLER_MAX 1000

/* Depuis l'ISR FOC, apres foc_isr() */
void sampler_isr(const foc_adc_t *in);
/* COMM_SAMPLE_PRINT : [mode u8][len u16][decimation u8][raw u8 optionnel] */
void sampler_request(const comm_iface_t *reply, const uint8_t *d, uint32_t n);
/* Boucle principale : envoi des echantillons, au rythme de l'interface */
void sampler_poll(void);
/* Parametres de conversion (A et V par LSB, offsets des courants), tenus a jour par la configuration */
void sampler_set_scale(float i_scale, float v_scale, float vph_scale, float f_loop);

#endif
