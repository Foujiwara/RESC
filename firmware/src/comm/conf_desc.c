/*
 * conf_desc.c — Commande COMM_RESC_GET_CONF_XML (250) : description XML de la configuration.
 *
 *   requete : [250][conf u8][len i32][offset i32]          (gros-boutiste)
 *             conf : 0 = mcconf, 1 = appconf, 2 = info
 *   reponse : [250][conf u8][total i32][offset i32][donnees offset .. offset+n-1]
 *             n = min(len, 400, total - offset), 0 si offset hors limites ou len <= 0
 *             total = taille du XML compresse (qCompress : u32 taille + zlib) ; 0 si non fourni
 * Les tableaux sont constants en flash : simple copie du morceau demande.
 */
#include <string.h>
#include "conf_desc.h"
#include "vesc_conf.h"
#include "vesc_buffer.h"

#define CHUNK_MAX 400

int32_t conf_desc_reply(const uint8_t *d, uint32_t n, uint8_t *out)
{
    int32_t o = 0;
    out[o++] = COMM_RESC_GET_CONF_XML;
    if (n < 9) {
        out[o++] = n >= 1 ? d[0] : 0;
        vb_put_i32(out, 0, &o);
        vb_put_i32(out, 0, &o);
        return o;
    }
    int32_t i = 0;
    uint8_t conf = vb_get_u8(d, &i);
    int32_t len = vb_get_i32(d, &i);
    int32_t off = vb_get_i32(d, &i);
    const uint8_t *data = 0;
    int32_t total = 0;
    if (conf == 0) { data = conf_xml_mc; total = (int32_t)conf_xml_mc_len; }
    else if (conf == 1) { data = conf_xml_app; total = (int32_t)conf_xml_app_len; }
    out[o++] = conf;
    vb_put_i32(out, total, &o);
    vb_put_i32(out, off, &o);
    if (data && len > 0 && off >= 0 && off < total) {
        int32_t k = len < CHUNK_MAX ? len : CHUNK_MAX;
        if (k > total - off) k = total - off;
        memcpy(out + o, data + off, (size_t)k);
        o += k;
    }
    return o;
}
