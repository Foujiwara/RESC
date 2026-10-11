/* conf_desc.h — Description XML de la configuration pour RESC Tool (commande 250). */
#ifndef CONF_DESC_H
#define CONF_DESC_H

#include <stdint.h>

#define COMM_RESC_GET_CONF_XML 250   /* hors de la plage du protocole de reference (<= 159) */

/* d, n : charge utile sans l'octet de commande ; out : au moins 410 octets. Retourne la taille. */
int32_t conf_desc_reply(const uint8_t *d, uint32_t n, uint8_t *out);

#endif
