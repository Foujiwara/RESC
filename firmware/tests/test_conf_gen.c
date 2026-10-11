/*
 * test_conf_gen.c — Tests de la configuration generee depuis les XML :
 *  - profil VESC700 identique au serialiseur de reference (octets, defauts, lecture)
 *  - aller-retour serialisation / deserialisation des deux profils
 *  - commande 250 : reconstitution par morceaux de 400 octets, bornes ; les blocs
 *    reconstitues sont ecrits dans build/ pour la verification de decompression (Python)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vesc_conf.h"
#include "conf_desc.h"
#include "vesc_buffer.h"

int32_t ref_mcconf_serialize(uint8_t *buf, const vesc_mcconf_t *c);
bool ref_mcconf_deserialize(const uint8_t *buf, int32_t len, vesc_mcconf_t *c);
void ref_mcconf_defaults(vesc_mcconf_t *c);
int32_t ref_appconf_serialize(uint8_t *buf, const vesc_appconf_t *c);
bool ref_appconf_deserialize(const uint8_t *buf, int32_t len, vesc_appconf_t *c);
void ref_appconf_defaults(vesc_appconf_t *c);
uint8_t plat_default_controller_id(void) { return 42; }

static int fails;
static void check(const char *w, int ok) { printf("  [%s] %s\n", ok ? " OK " : "ECHEC", w); if (!ok) fails++; }

/* valeurs aleatoires plausibles par type, dans les plages des encodages */
__attribute__((unused)) static void randomize(void *s, const conf_field_t *f, int n, unsigned seed)
{
    srand(seed);
    for (int k = 0; k < n; k++) {
        uint8_t *p = (uint8_t *)s + f[k].offset;
        int r = rand();
        switch (f[k].ctype) {
        case 0: { float v = (float)((r % 20001) - 10000) / 7.0f; memcpy(p, &v, 4); break; }
        case 1: case 2: *p = (uint8_t)r; break;
        case 3: case 4: { uint16_t v = (uint16_t)r; memcpy(p, &v, 2); break; }
        default: { uint32_t v = (uint32_t)r * 2654435761u; memcpy(p, &v, 4); break; }
        }
    }
}

int main(void)
{
    static uint8_t a[1024], b[1024];
    static vesc_mcconf_t m1, m2;
    static vesc_appconf_t p1, p2;
    int32_t n1, n2;

#ifndef RESC_CONF_PROFILE_RESC
    /* la reference ecrit la signature du profil actif : comparaison dans le profil VESC700 seulement */
    printf("== Profil VESC700 : identique au serialiseur de reference\n");
    int same = 1;
    for (unsigned seed = 1; seed <= 50; seed++) {
        randomize(&m1, vesc_mcconf_fields, vesc_mcconf_nfields, seed);
        n1 = vesc700_mcconf_serialize(a, &m1); n2 = ref_mcconf_serialize(b, &m1);
        if (n1 != n2 || memcmp(a, b, (size_t)n1)) same = 0;
        randomize(&p1, vesc_appconf_fields, vesc_appconf_nfields, seed);
        n1 = vesc700_appconf_serialize(a, &p1); n2 = ref_appconf_serialize(b, &p1);
        if (n1 != n2 || memcmp(a, b, (size_t)n1)) same = 0;
    }
    check("50 configurations aleatoires : octets identiques (mcconf et appconf)", same);
    vesc_mcconf_defaults(&m1); ref_mcconf_defaults(&m2);
    n1 = vesc700_mcconf_serialize(a, &m1); n2 = ref_mcconf_serialize(b, &m2);
    check("defauts mcconf identiques", n1 == n2 && !memcmp(a, b, (size_t)n1));
    vesc_appconf_defaults(&p1); ref_appconf_defaults(&p2);
    n1 = vesc700_appconf_serialize(a, &p1); n2 = ref_appconf_serialize(b, &p2);
    check("defauts appconf identiques", n1 == n2 && !memcmp(a, b, (size_t)n1));
    check("taille mcconf 7.00 = 488 octets", vesc700_mcconf_serialize(a, &m1) == 488);
    randomize(&m1, vesc_mcconf_fields, vesc_mcconf_nfields, 7);
    n1 = ref_mcconf_serialize(a, &m1);
    memset(&m2, 0, sizeof m2);
    vesc_mcconf_t m3;
    memset(&m3, 0, sizeof m3);
    check("lecture identique a la reference", vesc700_mcconf_deserialize(a, n1, &m2) && ref_mcconf_deserialize(a, n1, &m3) &&
          !memcmp(&m2, &m3, sizeof m2));
    a[0] ^= 0xFF;
    check("signature fausse refusee", !vesc700_mcconf_deserialize(a, n1, &m2));
    check("taille insuffisante refusee", !vesc700_mcconf_deserialize(b, 10, &m2));
#endif

    printf("== Profil RESC : aller-retour\n");
    vesc_mcconf_defaults(&m1);
    n1 = rescp_mcconf_serialize(a, &m1);
    memset(&m2, 0, sizeof m2);
    check("deserialisation mcconf RESC", rescp_mcconf_deserialize(a, n1, &m2));
    n2 = rescp_mcconf_serialize(b, &m2);
    check("octets identiques apres aller-retour", n1 == n2 && !memcmp(a, b, (size_t)n1) && n1 == RESCP_MCCONF_SER_SIZE);
    vesc_appconf_defaults(&p1);
    n1 = rescp_appconf_serialize(a, &p1);
    check("appconf RESC aller-retour", rescp_appconf_deserialize(a, n1, &p2) && rescp_appconf_serialize(b, &p2) == n1 &&
          !memcmp(a, b, (size_t)n1));
    check("signatures RESC differentes des signatures 7.00",
          RESCP_MCCONF_SIGNATURE != VESC700_MCCONF_SIGNATURE && RESCP_APPCONF_SIGNATURE != VESC700_APPCONF_SIGNATURE);
    check("une configuration 7.00 n'est pas lue comme RESC", !rescp_mcconf_deserialize(a, n1, &m2));

    printf("== Commande 250\n");
    for (int conf = 0; conf < 2; conf++) {
        uint8_t req[9], out[512];
        int32_t total = -1, off = 0;
        static uint8_t blob[64 * 1024];
        int ok = 1;
        for (int guard = 0; guard < 1000; guard++) {
            int32_t i = 0;
            req[i++] = (uint8_t)conf;
            vb_put_i32(req, off == 0 && total < 0 ? 10 : 400, &i);
            vb_put_i32(req, off, &i);
            int32_t n = conf_desc_reply(req, 9, out);
            int32_t j = 2;
            int32_t t = vb_get_i32(out, &j), o2 = vb_get_i32(out, &j);
            if (out[0] != COMM_RESC_GET_CONF_XML || out[1] != conf || o2 != off) { ok = 0; break; }
            int32_t k = n - 10;
            if (k < 0 || k > 400) { ok = 0; break; }
            total = t;
            memcpy(blob + off, out + 10, (size_t)k);
            off += k;
            if (off >= total) break;
        }
        char nm[64];
        snprintf(nm, sizeof nm, "build/cmd250_%s.bin", conf ? "app" : "mc");
        FILE *fh = fopen(nm, "wb");
        if (fh) { fwrite(blob, 1, (size_t)off, fh); fclose(fh); }
        char t[96];
        snprintf(t, sizeof t, "conf %d : %d octets reconstitues par morceaux <= 400", conf, (int)off);
        check(t, ok && off == total && total > 1000);
    }
    {
        uint8_t req[9], out[512];
        int32_t i = 0, j;
        req[i++] = 2; vb_put_i32(req, 400, &i); vb_put_i32(req, 0, &i);
        int32_t n = conf_desc_reply(req, 9, out);
        j = 2;
        check("conf 2 (info) : total 0, pas de donnees", n == 10 && vb_get_i32(out, &j) == 0);
        i = 0; req[i++] = 0; vb_put_i32(req, -5, &i); vb_put_i32(req, 0, &i);
        check("longueur negative : pas de donnees", conf_desc_reply(req, 9, out) == 10);
        i = 0; req[i++] = 0; vb_put_i32(req, 400, &i); vb_put_i32(req, 0x7FFFFFF0, &i);
        check("offset hors limites : pas de donnees", conf_desc_reply(req, 9, out) == 10);
        i = 0; req[i++] = 0; vb_put_i32(req, 400, &i); vb_put_i32(req, -1, &i);
        check("offset negatif : pas de donnees", conf_desc_reply(req, 9, out) == 10);
        i = 0; req[i++] = 0; vb_put_i32(req, 100000, &i); vb_put_i32(req, 0, &i);
        check("longueur enorme : 400 octets au plus", conf_desc_reply(req, 9, out) == 410);
        check("paquet trop court : reponse vide", conf_desc_reply(req, 3, out) == 10);
    }

    printf("\n%s (%d echec(s))\n", fails ? "ECHEC" : "TOUS LES TESTS PASSENT", fails);
    return fails ? 1 : 0;
}
