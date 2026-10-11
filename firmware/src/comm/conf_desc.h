/* conf_desc.h — Description XML de la configuration pour RESC Tool (commande 250). */
#ifndef CONF_DESC_H
#define CONF_DESC_H

#include <stdbool.h>
#include <stdint.h>
#include "vesc_conf.h"

#define COMM_RESC_GET_CONF_XML 250   /* hors de la plage du protocole de reference (<= 159) */

/* Entree des onglets : 'G' groupe, 'S' onglet, 'P' parametre ou separateur (::sep::), 'E' fin.
 * cond : condition d'affichage (0 = toujours), evaluee par vis() sur la configuration en cours. */
typedef struct {
    char kind;
    uint16_t cond;
    const char *text;
} conf_grp_t;

/* Description generee (tools/resc_conf/desc_vis.py) : partie fixe <Params>/<SerOrder> en flux
 * zlib sans bloc final, taille et adler32 de cette partie, onglets et conditions. */
typedef struct {
    const uint8_t *z;
    uint32_t z_len;
    uint32_t raw_len;
    uint32_t adler;
    const conf_grp_t *grp;
    bool (*vis)(uint16_t cond, const vesc_mcconf_t *mc, const vesc_appconf_t *app);
} conf_desc_t;

extern const conf_desc_t conf_desc_mc, conf_desc_app;

/* Configuration consultee pour les conditions (sans source : tout est affiche). */
void conf_desc_source(const vesc_mcconf_t *mc, const vesc_appconf_t *app);

/* d, n : charge utile sans l'octet de commande ; out : au moins 410 octets. Retourne la taille. */
int32_t conf_desc_reply(const uint8_t *d, uint32_t n, uint8_t *out);

#endif
