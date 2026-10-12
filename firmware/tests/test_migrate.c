/*
 * test_migrate.c — Stockage par nom et migration de la configuration  :
 *  - ancien format (serialisation 7.00 + en-tete 'RESC') relu, valeurs conservees, reecrit par nom
 *  - format par nom relu a l'identique ; champs inconnus ignores, champs nouveaux aux defauts
 *  - odometre enregistre avec la configuration des applications, relu, ecrit seulement s'il change
 *  - flash illisible : defauts, moteur non identifie (pont interdit)
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "resc_conf.h"
#include "vesc_conf.h"
#include "stats.h"

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
/* zone de copie pendant la mise a jour (fin du tampon), simulee */
static uint8_t upd[0x2000];
bool flash_upd_backup_free(void) { for (int k = 0; k < 0x2000; k++) if (upd[k] != 0xFF) return false; return true; }
bool flash_upd_backup_write(uint32_t o, const uint8_t *d, uint32_t n) { if (o + n > sizeof upd) return false; memcpy(upd + o, d, n); return true; }
bool flash_upd_backup_erase(void) { memset(upd, 0xFF, sizeof upd); return true; }
const uint8_t *flash_upd_backup_ptr(void) { return upd; }
uint8_t plat_default_controller_id(void) { return 42; }
void plat_pwm_set_freq(float f, uint8_t m) { (void)f; (void)m; }
void plat_sensor_config(uint8_t m, uint32_t c, uint8_t p, const abs_enc_sincos_t *s) { (void)m; (void)c; (void)p; (void)s; }
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
    check("comportement de RESC 0.x conserve (observateur 0, gain plein, anticipation, sans filtre de phase)",
          g_mcconf.foc_observer_type == 7 && g_mcconf.foc_observer_gain_slow == 1.0f && g_mcconf.foc_cc_decoupling == 3 &&
          g_mcconf.foc_phase_filter_enable == 0);
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

    printf("== Odometre enregistre avec la configuration des applications\n");
    {
        stats_set_odometer_m(123456u);
        resc_conf_store_app();
        int w0 = n_writes;
        resc_conf_store_backup();
        check("odometre inchange : pas de nouvelle ecriture", n_writes == w0);
        stats_set_odometer_m(0u);
        resc_conf_init();
        check("odometre relu a la mise sous tension (123456 m)", stats_odometer_m() == 123456u &&
              resc_conf_load_status(true) == RESC_LOAD_OK);
        stats_set_odometer_m(123500u);
        resc_conf_store_backup();
        check("odometre change : enregistre a l'arret", n_writes == w0 + 1);
    }

    printf("== Mise a jour du firmware : le bootloader efface la configuration\n");
    {
        memset(upd, 0xFF, sizeof upd);
        g_mcconf.l_current_max = 187.0f;
        g_appconf.app_adc_conf_ctrl_type = 15;
        resc_conf_store_mc();
        resc_conf_store_app();
        check("copie faite avant le bootloader", resc_conf_backup_for_update());
        flash_len[0] = flash_len[1] = 0;                 /* secteurs effaces par le bootloader */
        memset(&g_mcconf, 0, sizeof g_mcconf);
        resc_conf_init();
        check("configuration restauree (moteur 187 A, ADC type 15)", g_mcconf.l_current_max == 187.0f &&
              g_appconf.app_adc_conf_ctrl_type == 15 && resc_conf_restored_after_update());
        check("reecrite dans les secteurs, copie effacee", flash_len[0] > 0 && flash_len[1] > 0 && upd[0] == 0xFF);
        flash_len[0] = flash_len[1] = 0;
        resc_conf_init();
        check("sans copie : defauts (pas de vieille restauration)", resc_conf_load_status(false) == RESC_LOAD_EMPTY);
        resc_conf_store_mc();
        resc_conf_store_app();
        /* zone flash occupee (ou effacee par le bootloader) : la copie en RAM CCM suffit */
        upd[0] = 0x00;
        g_mcconf.l_current_max = 155.0f;
        resc_conf_store_mc();
        check("zone flash occupee : copie faite en RAM", resc_conf_backup_for_update());
        memset(upd, 0xFF, sizeof upd);                   /* le bootloader efface aussi le tampon */
        flash_len[0] = flash_len[1] = 0;
        memset(&g_mcconf, 0, sizeof g_mcconf);
        resc_conf_init();
        check("restauree depuis la RAM malgre le tampon efface (155 A)", g_mcconf.l_current_max == 155.0f &&
              resc_conf_restored_after_update());
        flash_len[0] = flash_len[1] = 0;
        resc_conf_init();
        check("copie RAM consommee : pas de seconde restauration", resc_conf_load_status(false) == RESC_LOAD_EMPTY);
        resc_conf_store_mc();
        resc_conf_store_app();
    }

    printf("== Flash illisible\n");
    flash[0][40] ^= 0x55;
    resc_conf_init();
    check("etat : illisible, defauts", resc_conf_load_status(false) == RESC_LOAD_FAILED);
    check("moteur non identifie : pont interdit", !resc_motor_configured());

    printf("\n%s (%d echec(s))\n", fails ? "ECHEC" : "TOUS LES TESTS PASSENT", fails);
    return fails ? 1 : 0;
}
