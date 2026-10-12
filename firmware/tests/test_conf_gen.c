/*
 * test_conf_gen.c — Tests de la configuration generee depuis les XML :
 *  - ancien format 7.00 identique au serialiseur de reference (octets, lecture)
 *  - aller-retour serialisation / deserialisation du format RESC
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

/* telechargement complet de la description 'conf' par morceaux, comme RESC Tool */
static int32_t fetch250(int conf, uint8_t *blob, int32_t *total_out, int *ok_out)
{
    uint8_t req[9], out[512];
    int32_t total = -1, off = 0;
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
        if (k < 0 || k > 400 || t > 64 * 1024) { ok = 0; break; }
        total = t;
        memcpy(blob + off, out + 10, (size_t)k);
        off += k;
        if (off >= total) break;
    }
    *total_out = total;
    *ok_out = ok;
    return off;
}

static void save(const char *nm, const uint8_t *b, int32_t n)
{
    FILE *fh = fopen(nm, "wb");
    if (fh) { fwrite(b, 1, (size_t)n, fh); fclose(fh); }
}

/* les onglets sont en blocs non compresses : recherche directe de <param>nom</param>, d'un nom
 * d'onglet, ou d'un texte XML */
__attribute__((unused)) static int has(const uint8_t *b, int32_t n, const char *name)
{
    char pat[160];
    if (strchr(name, '<')) snprintf(pat, sizeof pat, "%s", name);
    else if (strncmp(name, "foc_", 4) && strncmp(name, "app_", 4)) snprintf(pat, sizeof pat, "<subgroupName>%s<", name);
    else snprintf(pat, sizeof pat, "<param>%s</param>", name);
    size_t m = strlen(pat);
    for (int32_t i = 0; i + (int32_t)m <= n; i++) if (!memcmp(b + i, pat, m)) return 1;
    return 0;
}

/* recherche litterale (variante complete : attributs visibleIf) */
static int has_raw(const uint8_t *b, int32_t n, const char *pat)
{
    size_t m = strlen(pat);
    for (int32_t i = 0; i + (int32_t)m <= n; i++) if (!memcmp(b + i, pat, m)) return 1;
    return 0;
}

int main(void)
{
    static uint8_t a[1024], b[1024];
    static vesc_mcconf_t m1, m2;
    static vesc_appconf_t p1, p2;
    int32_t n1, n2;

    printf("== Ancien format 7.00 : identique au serialiseur de reference\n");
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
    vesc_mcconf_defaults(&m1);
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

    printf("== Profil RESC : listes reduites (position affichee <-> valeur interne)\n");
    {
        vesc_mcconf_defaults(&m1);
        m1.foc_hfi_amb_mode = 2;          /* double impulsion : 2e choix affiche */
        m1.m_out_aux_mode = 5;            /* 'On when running' : 5e choix affiche (4 'Unused' retire) */
        m1.l_additional_faults = 0x0B;    /* glissement, survitesse, survitesse absolue : bits 0, 1, 2 affiches */
        m1.sp_pid_loop_rate = 5;          /* 1000 Hz */
        rescp_mcconf_serialize(a, &m1);
        memset(&m2, 0, sizeof m2);
        rescp_mcconf_deserialize(a, RESCP_MCCONF_SER_SIZE, &m2);
        check("valeurs internes retrouvees apres aller-retour",
              m2.foc_hfi_amb_mode == 2 && m2.m_out_aux_mode == 5 && m2.l_additional_faults == 0x0B && m2.sp_pid_loop_rate == 5);
        /* position ecrite par l'outil -> valeur interne */
        const conf_field_t *f = 0;
        for (int k = 0; k < vesc_mcconf_nfields; k++) if (!strcmp(vesc_mcconf_fields[k].name, "m_out_aux_mode")) f = &vesc_mcconf_fields[k];
        (void)f;
        check("motor_type garde sa liste complete (FOC = 2 pour RESC Tool)", m2.motor_type == 2);
    }

    printf("== Commande 250\n");
    static uint8_t blob[64 * 1024];
    for (int conf = 0; conf < 2; conf++) {
        int ok;
        int32_t total, got = fetch250(conf, blob, &total, &ok);
        save(conf ? "build/cmd250_app.bin" : "build/cmd250_mc.bin", blob, got);
        char t[96];
        snprintf(t, sizeof t, "conf %d : %d octets reconstitues par morceaux <= 400", conf, (int)got);
        check(t, ok && got == total && total > 1000);
    }
    printf("== Commande 250 : onglets selon la configuration\n");
    {
        static vesc_mcconf_t mc;
        static vesc_appconf_t ap;
        int ok;
        int32_t total, got;
        vesc_mcconf_defaults(&mc);
        vesc_appconf_defaults(&ap);
        conf_desc_source(&mc, &ap);
        mc.foc_motor_kind = 0;
        mc.foc_sensor_mode = 0;
        got = fetch250(0, blob, &total, &ok);
        save("build/cmd250_mc_pm.bin", blob, got);
        check("aimants : flux de l'aimant visible, parametres d'induction absents",
              ok && has(blob, got, "foc_motor_flux_linkage") && !has(blob, got, "foc_acim_lm") &&
              has(blob, got, "foc_observer_type") && has(blob, got, "foc_sl_openloop_time_lock"));
        check("sans capteur : table Hall, codeur et injection absents",
              !has(blob, got, "foc_hall_table__0") && !has(blob, got, "foc_encoder_offset") &&
              !has(blob, got, "foc_hfi_voltage_run") && !has(blob, got, "Hall Sensors"));
        check("type de moteur en tete de General / General",
              has(blob, got, "<subgroupName>General</subgroupName><subgroupParams><param>foc_motor_kind</param>"));
        mc.foc_motor_kind = 1;
        got = fetch250(0, blob, &total, &ok);
        save("build/cmd250_mc_acim.bin", blob, got);
        check("induction : parametres d'induction visibles ; flux, saillance, MTPA, defluxage, type d'observateur absents",
              ok && has(blob, got, "foc_acim_lm") && has(blob, got, "foc_acim_base_erpm") &&
              !has(blob, got, "foc_motor_flux_linkage") && !has(blob, got, "foc_motor_ld_lq_diff") &&
              !has(blob, got, "foc_mtpa_mode") && !has(blob, got, "foc_observer_type") &&
              !has(blob, got, "foc_fw_current_max") && !has(blob, got, "foc_sl_openloop_time_lock"));
        mc.foc_sensor_mode = 2;     /* Hall : sans effet en induction */
        got = fetch250(0, blob, &total, &ok);
        check("induction + Hall demande : pas de table Hall", ok && !has(blob, got, "foc_hall_table__0"));
        mc.foc_motor_kind = 0;
        got = fetch250(0, blob, &total, &ok);
        check("aimants + Hall : table Hall visible, I/f absent", ok && has(blob, got, "foc_hall_table__7") &&
              has(blob, got, "foc_sl_erpm") && !has(blob, got, "foc_openloop_rpm"));
        mc.foc_sensor_mode = 1;
        mc.l_additional_faults = 0;
        got = fetch250(0, blob, &total, &ok);
        check("aimants + codeur seul : observateur et I/f absents, codeur visible",
              ok && has(blob, got, "foc_encoder_ratio") && !has(blob, got, "foc_observer_gain") &&
              !has(blob, got, "foc_openloop_rpm"));
        ap.app_ppm_conf_ctrl_type = 9;
        ap.app_ppm_conf_multi_esc = 0;
        got = fetch250(1, blob, &total, &ok);
        save("build/cmd250_app_cond.bin", blob, got);
        check("PPM marche arriere intelligente : ses reglages visibles, PID et TC absents",
              ok && has(blob, got, "app_ppm_conf.smart_rev_max_duty") && !has(blob, got, "app_ppm_conf.pid_max_erpm") &&
              !has(blob, got, "app_ppm_conf.tc_max_diff"));

        printf("== Commande 250 : variante complete (bit 7, conditions evaluees par RESC Tool)\n");
        vesc_mcconf_defaults(&mc);
        vesc_appconf_defaults(&ap);
        static uint8_t blob2[64 * 1024];
        int32_t t2, got2;
        int ok2;
        /* fichiers de test pour RESC Tool (fake_resc_device.py --raw) : sans bit / avec bit */
        got = fetch250(0, blob, &total, &ok);
        save("build/conf_xml_mc.bin", blob, got);
        got2 = fetch250(0x80, blob2, &t2, &ok2);
        save("build/vis_conf_xml_mc.bin", blob2, got2);
        check("mc, 0x80 : reponse avec le meme octet, plus longue que la variante filtree", ok2 && got2 == t2 && got2 > got);
        check("mc, 0x80 : tous les parametres (aimants et induction), avec leurs conditions",
              has_raw(blob2, got2, "foc_acim_lm</param>") && has_raw(blob2, got2, "foc_motor_flux_linkage</param>") &&
              has_raw(blob2, got2, "foc_hall_table__0</param>") && has_raw(blob2, got2, "foc_hfi_voltage_run</param>") &&
              has_raw(blob2, got2, "<param visibleIf=\"foc_motor_kind == 1\">foc_acim_lm</param>"));
        check("mc, 0x80 : !, parentheses, in [...], & sur un champ de bits, condition d'onglet, <Pages>",
              has_raw(blob2, got2, "visibleIf=\"!(foc_sensor_mode == 1 &amp;&amp; (foc_motor_kind == 1 || !(l_additional_faults &amp; 1)))\"") &&
              has_raw(blob2, got2, "m_sensor_port_mode in [") && has_raw(blob2, got2, "(bms.limit_mode &amp; 1)") &&
              has_raw(blob2, got2, "<subgroup visibleIf=\"foc_sensor_mode == 1\"><subgroupName>Sensors</subgroupName>") &&
              has_raw(blob2, got2, "<Pages><page name=\"motor_bldc\" visibleIf=\"foc_motor_kind &lt; 0\"/>"));
        got = fetch250(1, blob, &total, &ok);
        save("build/conf_xml_app.bin", blob, got);
        got2 = fetch250(0x81, blob2, &t2, &ok2);
        save("build/vis_conf_xml_app.bin", blob2, got2);
        check("app, 0x81 : condition de groupe et de page (application utilisee)",
              ok2 && got2 == t2 && has_raw(blob2, got2, "<group visibleIf=\"app_to_use in [2, 5, 10]\"><groupName>ADC</groupName>") &&
              has_raw(blob2, got2, "<page name=\"app_adc\" visibleIf=\"app_to_use in [2, 5, 10]\"/>"));
        check("app, 0x81 : options RESC de l'ADC avec leurs conditions",
              has_raw(blob2, got2, "visibleIf=\"app_adc_conf.ctrl_type in [1, 2, 3, 4, 5, 6, 7, 8] &amp;&amp; app_adc_conf.resc_hybrid == 1\">app_adc_conf.resc_hybrid_band</param>") &&
              has_raw(blob2, got2, "app_adc_conf.resc_smart_rev_ramp_time</param>"));
        /* la variante complete ne depend pas de la configuration */
        int32_t got3 = fetch250(0x80, blob, &total, &ok);
        mc.foc_motor_kind = 1; mc.foc_sensor_mode = 3;
        int32_t got4 = fetch250(0x80, blob2, &t2, &ok2);
        check("variante complete identique quelle que soit la configuration", got3 == got4 && !memcmp(blob, blob2, (size_t)got3));
        conf_desc_source(0, 0);
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
