/*
 * test_migrate.c — Stockage par nom et migration de la configuration (compile dans les deux profils) :
 *  - ancien format (serialisation 7.00 + en-tete 'RESC') relu, valeurs conservees, reecrit par nom
 *  - format par nom relu a l'identique ; champs inconnus ignores, champs nouveaux aux defauts
 *  - flash illisible : defauts, moteur non identifie (pont interdit)
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "resc_conf.h"
#include "vesc_conf.h"

static int fails;
static void check(const char *w, int ok) { printf("  [%s] %s\n", ok ? " OK " : "ECHEC", w); if (!ok) fails++; }

/* flash simulee : deux emplacements */
static uint8_t flash[2][16384];
static uint32_t flash_len[2];
static int n_writes;
bool plat_conf_write(uint32_t s, const uint8_t *d, uint32_t n)
{
    if (s > 1 || n > sizeof(flash[0])) return false;
    memcpy(flash[s], d, n);
    flash_len[s] = n;
    n_writes++;
    return true;
}
const uint8_t *plat_conf_read(uint32_t s, uint32_t *n)
{
    if (s > 1 || !flash_len[s]) { *n = 0; return 0; }
    *n = flash_len[s];
    return flash[s];
}
uint8_t plat_default_controller_id(void) { return 42; }
void plat_pwm_set_freq(float f) { (void)f; }
void plat_sensor_config(uint8_t m, uint32_t c) { (void)m; (void)c; }
void plat_hall_samples(uint8_t e) { (void)e; }
void foc_hw_phase_filter(bool on) { (void)on; }
void foc_hw_kill(void) { }

static uint32_t crc32(const uint8_t *d, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        c ^= d[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(c & 1));
    }
    return ~c;
}

/* ecrit une configuration dans l'ancien format (RESC 0.x) */
static void write_old(uint32_t slot, const uint8_t *ser, int32_t n)
{
    uint32_t h[4] = { 0x52455343u, (uint32_t)n, crc32(ser, (uint32_t)n), slot };
    memcpy(flash[slot], h, 16);
    memcpy(flash[slot] + 16, ser, (size_t)n);
    flash_len[slot] = 16u + (uint32_t)n;
}

int main(void)
{
    static uint8_t buf[1024];
    printf("== Migration depuis l'ancien format (configuration 7.00)\n");
    vesc_mcconf_t m;
    vesc_mcconf_defaults(&m);
    m.foc_motor_r = 0.0421f; m.foc_motor_l = 61.5e-6f; m.foc_motor_flux_linkage = 0.0163f;
    m.l_current_max = 55.0f; m.l_current_min = -40.0f; m.foc_observer_type = 0; m.foc_sensor_mode = 2;
    m.foc_hall_table_3 = 77.0f;
    write_old(0, buf, vesc700_mcconf_serialize(buf, &m));
    vesc_appconf_t a;
    vesc_appconf_defaults(&a);
    a.controller_id = 17.0f; a.app_to_use = 1; a.app_ppm_conf_ctrl_type = 3;
    write_old(1, buf, vesc700_appconf_serialize(buf, &a));
    n_writes = 0;
    resc_conf_init();
    check("etat : migree (moteur et application)",
          resc_conf_load_status(false) == RESC_LOAD_MIGRATED && resc_conf_load_status(true) == RESC_LOAD_MIGRATED);
    check("R, L, flux conserves", g_mcconf.foc_motor_r == 0.0421f && g_mcconf.foc_motor_l == 61.5e-6f &&
          g_mcconf.foc_motor_flux_linkage == 0.0163f);
    check("limites, capteur et table Hall conserves", g_mcconf.l_current_max == 55.0f && g_mcconf.l_current_min == -40.0f &&
          g_mcconf.foc_sensor_mode == 2 && g_mcconf.foc_hall_table_3 == 77.0f);
    check("application conservee", g_appconf.controller_id == 17.0f && g_appconf.app_to_use == 1 &&
          g_appconf.app_ppm_conf_ctrl_type == 3);
    check("nouveaux champs RESC a leur defaut", g_mcconf.foc_motor_kind == 0 && g_mcconf.foc_resc_cc_bw == 1000.0f &&
          fabsf(g_mcconf.foc_resc_rl_frac - 0.5f) < 1e-6f);
    check("moteur considere identifie", resc_motor_configured());
    check("reecrit aussitot dans le format par nom", n_writes == 2 && ((uint32_t *)flash[0])[0] == 0x32534552u);

    printf("== Relecture du format par nom\n");
    g_mcconf.foc_acim_rr = 0.123f;
    g_mcconf.foc_resc_cc_bw = 1500.0f;
    resc_conf_store_mc();
    vesc_mcconf_t before = g_mcconf;
    memset(&g_mcconf, 0, sizeof g_mcconf);
    resc_conf_init();
    check("etat : OK", resc_conf_load_status(false) == RESC_LOAD_OK);
    {
        static uint8_t s1[1024], s2[1024];
        int32_t a1 = vesc700_mcconf_serialize(s1, &before), a2 = vesc700_mcconf_serialize(s2, &g_mcconf);
        int32_t b1 = rescp_mcconf_serialize(s1 + 512, &before), b2 = rescp_mcconf_serialize(s2 + 512, &g_mcconf);
        check("configuration identique apres relecture (champs des deux profils)",
              a1 == a2 && b1 == b2 && !memcmp(s1, s2, (size_t)a1) && !memcmp(s1 + 512, s2 + 512, (size_t)b1));
    }

    printf("== Champ inconnu et type change\n");
    {
        /* enregistrement d'un champ inexistant, puis l_current_max stocke en entier signe */
        uint8_t *d = flash[0] + 16;
        uint32_t n = ((uint32_t *)flash[0])[1];
        uint32_t unk = 0xDEADBEEFu;
        memcpy(d + n, &unk, 4); d[n + 4] = 1; memset(d + n + 5, 0, 4);
        const conf_field_t *f = resc_conf_field(false, "l_current_max");
        int32_t v = 33;
        memcpy(d + n + 9, &f->hash, 4); d[n + 13] = 6; memcpy(d + n + 14, &v, 4);
        n += 18;
        uint32_t h[4] = { 0x32534552u, n, crc32(d, n), 0 };
        memcpy(flash[0], h, 16);
        flash_len[0] = 16 + n;
        resc_conf_init();
        check("champ inconnu ignore, conversion de type (int32 33 -> float 33)",
              resc_conf_load_status(false) == RESC_LOAD_OK && g_mcconf.l_current_max == 33.0f);
    }

    printf("== Flash illisible\n");
    flash[0][40] ^= 0x55;
    resc_conf_init();
    check("etat : illisible, defauts", resc_conf_load_status(false) == RESC_LOAD_FAILED);
    check("moteur non identifie : pont interdit", !resc_motor_configured());

    printf("\n%s (%d echec(s))\n", fails ? "ECHEC" : "TOUS LES TESTS PASSENT", fails);
    return fails ? 1 : 0;
}
