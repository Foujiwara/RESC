/*
 * test_conf.c — Tests PC de la configuration RESC sur la carte reelle (Ubox 85_200) :
 *  - serialisation MCCONF/APPCONF aller-retour identique, signature, taille
 *  - echelles ADC et signe du courant (polarite inversee de l'Ubox)
 *  - plafonds de courant tant que la carte n'est pas validee au banc
 *  - bornage des valeurs hors limites recues de l'outil
 *  - CRC16 du protocole (vecteur de reference CRC-16/XMODEM)
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "resc_conf.h"
#include "vesc_conf.h"
#include "vesc_defaults.h"
#include "packet.h"
#include "hw_config.h"

static int fails;
static void check(const char *w, int ok) { printf("  [%s] %s\n", ok ? " OK " : "ECHEC", w); if (!ok) fails++; }

/* bouchons plateforme */
bool plat_conf_write(uint32_t s, const uint8_t *d, uint32_t n) { (void)s; (void)d; (void)n; return true; }
const uint8_t *plat_conf_read(uint32_t s, uint32_t *n) { (void)s; *n = 0; return 0; }
bool flash_upd_backup_free(void) { return false; }
bool flash_upd_backup_write(uint32_t o, const uint8_t *d, uint32_t n) { (void)o; (void)d; (void)n; return false; }
bool flash_upd_backup_erase(void) { return true; }
const uint8_t *flash_upd_backup_ptr(void) { return 0; }
uint8_t plat_default_controller_id(void) { return 42; }
void plat_pwm_set_freq(float f, uint8_t m) { (void)f; (void)m; }
void plat_sensor_config(uint8_t m, uint32_t c, uint8_t p, const abs_enc_sincos_t *s) { (void)m; (void)c; (void)p; (void)s; }
void plat_hall_samples(uint8_t e) { (void)e; }
void foc_hw_phase_filter(bool on) { (void)on; }
void foc_hw_kill(void) { }
void commands_process(const comm_iface_t *s, uint8_t *d, uint32_t n) { (void)s; (void)d; (void)n; }
void can_vesc_send_buffer(uint8_t t, const uint8_t *d, uint32_t n, uint8_t m) { (void)t; (void)d; (void)n; (void)m; }

int main(void)
{
    static uint8_t a[1024], b[1024];
    printf("== Serialisation\n");
    resc_conf_init();
    int32_t n1 = vesc_mcconf_serialize(a, &g_mcconf);
    vesc_mcconf_t m2;
    check("taille MCCONF = taille attendue", n1 == VESC_MCCONF_SER_MAX);
    check("deserialisation MCCONF", vesc_mcconf_deserialize(a, n1, &m2));
    int32_t n2 = vesc_mcconf_serialize(b, &m2);
    check("aller-retour MCCONF identique octet a octet", n1 == n2 && !memcmp(a, b, (size_t)n1));
    a[0] ^= 1;
    check("signature fausse refusee", !vesc_mcconf_deserialize(a, n1, &m2));
    int32_t na = vesc_appconf_serialize(a, &g_appconf);
    vesc_appconf_t ap2;
    check("deserialisation APPCONF", na == VESC_APPCONF_SER_MAX && vesc_appconf_deserialize(a, na, &ap2));
    int32_t nb = vesc_appconf_serialize(b, &ap2);
    check("aller-retour APPCONF identique", na == nb && !memcmp(a, b, (size_t)na));
    check("identifiant CAN par defaut issu de l'UID", (int)g_appconf.controller_id == 42);

    printf("== Traduction vers le coeur (Ubox 85_200)\n");
    foc_params_t fp;
    ctrl_limits_t cl;
    resc_conf_build(&g_mcconf, &g_appconf, &fp, &cl);
    printf("  f_pwm=%.0f i_scale=%.4f A/LSB v_scale=%.4f V/LSB i_max=%.1f i_abs=%.1f vbus=%.0f..%.0f\n",
           fp.f_pwm, fp.i_scale, fp.v_scale, cl.i_max, fp.i_abs_trip, fp.vbus_trip_min, fp.vbus_trip_max);
    check("f_PWM = 15 kHz", fabsf(fp.f_pwm - 15000.0f) < 1.0f);
    check("echelle courant -0.252 A/LSB (polarite inversee)", fabsf(fp.i_scale + 0.2521f) < 0.001f);
    check("echelle Vbus 27.6 mV/LSB", fabsf(fp.v_scale - 0.02765f) < 0.0002f);
    check("courant max = 30 A (defaut prudent)", fabsf(cl.i_max - 30.0f) < 0.01f);
    check("coupure absolue = l_abs_current_max (carte validee, 100 %)", fabsf(fp.i_abs_trip - fminf(g_mcconf.l_abs_current_max, 350.0f)) < 0.01f);
    check("Vbus 12..82 V", fabsf(fp.vbus_trip_min - 12.0f) < 0.01f && fabsf(fp.vbus_trip_max - 82.0f) < 0.01f);
    check("shunts low-side, 3 phases", fp.cs_lowside && !fp.cs_two_shunts);
    check("sensorless par defaut", fp.sensor_mode == 0);
    check("moteur non identifie au depart (pont interdit)", !resc_motor_configured());

    printf("== Bornage des valeurs recues\n");
    vesc_mcconf_t m = g_mcconf;
    m.l_current_max = 1000.0f;
    m.l_abs_current_max = 5000.0f;
    m.l_max_vin = 200.0f;
    m.foc_f_zv = 200000.0f;
    m.foc_sensor_mode = FOC_SENSOR_MODE_HALL;     /* table Hall vide */
    resc_conf_clamp_mc(&m);
    check("l_current_max <= 300 A (HWLIM)", m.l_current_max <= 300.0f);
    check("l_abs_current_max <= 350 A (HWLIM abaisse, doc 15)", m.l_abs_current_max <= 350.0f);
    check("l_max_vin <= 85 V", m.l_max_vin <= 85.0f);
    check("f_zv <= 2 x 40 kHz", m.foc_f_zv <= 80000.0f);
    resc_conf_build(&m, &g_appconf, &fp, &cl);
    /* plafond : min(HWLIM_I_MOTOR_A = 300 A, 80 % de la coupure = 0.8 x 350 A) */
    check("courant effectif = 280 A (80 % de la coupure, sous le plafond HWLIM)", fabsf(cl.i_max - 280.0f) < 0.01f);
    check("courant moteur toujours sous 80 % de la coupure", cl.i_max <= 0.8f * fp.i_abs_trip + 0.01f &&
                                                              -cl.i_min <= 0.8f * fp.i_abs_trip + 0.01f);
    check("table Hall vide -> reste en sensorless", fp.sensor_mode == 0);
    m.foc_motor_r = 0.06f; m.foc_motor_l = 100e-6f; m.foc_motor_flux_linkage = 0.018f;
    g_mcconf = m;
    check("moteur considere identifie apres saisie R/L/flux", resc_motor_configured());

    printf("== Garde-fous\n");
    {
        vesc_mcconf_t z = g_mcconf;
        z.l_current_max = NAN;
        z.foc_motor_r = INFINITY;
        z.foc_f_zv = NAN;
        resc_conf_clamp_mc(&z);
        check("NaN / infini remplaces par des valeurs finies", isfinite(z.l_current_max) && isfinite(z.foc_motor_r) && isfinite(z.foc_f_zv));
        z.foc_f_zv = 80000.0f;            /* PWM 40 kHz */
        resc_conf_clamp_mc(&z);
        foc_params_t f2;
        ctrl_limits_t c2;
        resc_conf_build(&z, &g_appconf, &f2, &c2);
        float m_lim = (0.5f - 3e-6f * 40000.0f) * (4.0f / 1.7320508f);
        check("modulation bornee pour garder 2 phases mesurables a 40 kHz", f2.m_max <= m_lim + 1e-4f);
    }

    printf("== Valeurs derivees apres detection\n");
    resc_conf_derive_current_gains(&g_mcconf, RESC_CURRENT_BW_DEFAULT);
    resc_conf_derive_observer_gain(&g_mcconf);
    check("kp = L x 1000 = 0.1", fabsf(g_mcconf.foc_current_kp - 0.1f) < 1e-5f);
    check("ki = R x 1000 = 60", fabsf(g_mcconf.foc_current_ki - 60.0f) < 1e-3f);
    check("gain observateur = 1000 / lambda^2", fabsf(g_mcconf.foc_observer_gain / (1e3f / (0.018f * 0.018f)) - 1.0f) < 1e-5f);

    printf("== CRC16 du protocole\n");
    check("CRC-16/XMODEM(\"123456789\") = 0x31C3", crc16((const uint8_t *)"123456789", 9) == 0x31C3);

    printf("\n%s (%d echec(s))\n", fails ? "ECHEC" : "TOUS LES TESTS PASSENT", fails);
    return fails ? 1 : 0;
}
