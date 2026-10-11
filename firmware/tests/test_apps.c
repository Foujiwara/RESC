/*
 * test_apps.c — Tests PC des applications (apps.c) : entrees simulees, consignes enregistrees.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "apps.h"
#include "inputs.h"
#include "ctrl.h"
#include "foc.h"
#include "can_vesc.h"
#include "resc_conf.h"
#include "vesc_conf.h"
#include "vesc_defaults.h"

static int fails;
static void check(const char *w, int ok) { printf("  [%s] %s\n", ok ? " OK " : "ECHEC", w); if (!ok) fails++; }

/* ------------------------------------------------------------ bouchons */
vesc_mcconf_t g_mcconf;
vesc_appconf_t g_appconf;
foc_telem_t g_foc_telem;
static float in_servo_ms = 1.5f;
static bool in_servo_ok = true, in_servo_lvl = true, in_cc, in_rev;
static float in_adc[3];
static int32_t in_pas;
void inputs_init(bool servo, in_uart_use_t u, bool o) { (void)servo; (void)u; (void)o; }
void inputs_servo_out_init(void) { }
void inputs_servo_out_set(float p) { (void)p; }
bool inputs_servo_ms(float *ms, uint32_t now) { (void)now; *ms = in_servo_ms; return in_servo_ok; }
bool inputs_servo_level(void) { return in_servo_lvl; }
float inputs_adc_v(int n) { return in_adc[n]; }
bool inputs_btn_cruise(void) { return in_cc; }
bool inputs_btn_reverse(void) { return in_rev; }
int32_t inputs_pas_poll(void) { int32_t d = in_pas; in_pas = 0; return d; }
void uart_attach_pins(int i, uint8_t a, uint8_t b) { (void)i; (void)a; (void)b; }
uint8_t plat_default_controller_id(void) { return 1; }

static ctrl_limits_t L;
static char last_kind[16];
static float last_val;
static int n_cmd;
static void rec(const char *k, float v) { strcpy(last_kind, k); last_val = v; n_cmd++; }
void ctrl_set_current_rel(float r) { rec("cur_rel", r); }
void ctrl_set_brake_rel(float r) { rec("brake_rel", r); }
void ctrl_set_duty(float d) { rec("duty", d); }
void ctrl_set_rpm(float e) { rec("rpm", e); }
void ctrl_set_pos(float p) { rec("pos", p); }
void ctrl_release(void) { rec("release", 0); }
const ctrl_limits_t *ctrl_limits(void) { return &L; }

static can_peer_t peers[2];
static int npeers;
static int can_n;
static float can_last[4];
const can_peer_t *can_vesc_peers(int *n) { *n = npeers; return peers; }
void can_vesc_send_current_rel(uint8_t id, float r) { can_last[id & 3] = r; can_n++; }
void can_vesc_send_current(uint8_t id, float a) { (void)id; (void)a; can_n++; }
void can_vesc_send_brake(uint8_t id, float a) { (void)id; (void)a; can_n++; }
void can_vesc_send_duty(uint8_t id, float d) { (void)id; (void)d; can_n++; }
void can_vesc_send_rpm(uint8_t id, float e) { (void)id; (void)e; can_n++; }

static uint32_t now = 1000;
static void tick(int ms) { for (int k = 0; k < ms; k++) apps_tick_1khz(now++); }
static void set_erpm(float e) { g_foc_telem.omega = e * 6.2831853f / 60.0f; }

int main(void)
{
    vesc_appconf_defaults(&g_appconf);
    L.i_max = 40; L.i_min = -40; L.duty_max = 0.95f; L.erpm_max = 50000; L.erpm_min = -50000;
    g_appconf.timeout_msec = 1000;

    printf("== PPM, courant, rampes nulles, demarrage securise\n");
    g_appconf.app_to_use = APP_PPM;
    g_appconf.app_ppm_conf_ctrl_type = 1;
    g_appconf.app_ppm_conf_pulse_start = 1.0f; g_appconf.app_ppm_conf_pulse_end = 2.0f; g_appconf.app_ppm_conf_pulse_center = 1.5f;
    g_appconf.app_ppm_conf_hyst = 0.05f; g_appconf.app_ppm_conf_ramp_time_pos = 0; g_appconf.app_ppm_conf_ramp_time_neg = 0;
    g_appconf.app_ppm_conf_throttle_exp = 0; g_appconf.app_ppm_conf_throttle_exp_brake = 0;
    g_appconf.app_ppm_conf_safe_start = 1; g_appconf.app_ppm_conf_median_filter = 0; g_appconf.app_ppm_conf_multi_esc = 0;
    apps_configure();
    in_servo_ms = 2.0f; n_cmd = 0;
    tick(300);
    check("gaz a fond au demarrage : aucune commande (demarrage securise)", n_cmd == 0);
    in_servo_ms = 1.5f; tick(300);
    in_servo_ms = 2.0f; tick(10);
    check("neutre puis gaz : courant relatif +1", !strcmp(last_kind, "cur_rel") && fabsf(last_val - 1.0f) < 1e-3f);
    in_servo_ms = 1.25f; tick(10);
    check("1.25 ms : courant relatif -0.47 (zone morte 5 %)", fabsf(last_val + 0.4737f) < 0.01f);
    check("decodage pour l'outil : -0.5", fabsf(apps_ppm_decoded() + 0.5f) < 0.01f);

    printf("== PPM, frein sans marche arriere, rampe 0.5 s\n");
    g_appconf.app_ppm_conf_ctrl_type = 3; g_appconf.app_ppm_conf_ramp_time_pos = 0.5f;
    in_servo_ms = 1.5f; tick(50);
    in_servo_ms = 2.0f; tick(250);
    check("rampe : ~50 % apres 0.25 s", fabsf(last_val - 0.5f) < 0.03f);
    g_appconf.app_ppm_conf_ramp_time_pos = 0.0f;
    in_servo_ms = 1.0f; tick(5);
    check("manette en arriere : frein", !strcmp(last_kind, "brake_rel") && last_val > 0.9f);

    printf("== PPM, perte du signal\n");
    in_servo_ok = false; n_cmd = 0; tick(100);
    check("signal perdu : plus aucune commande (le timeout agit)", n_cmd == 0);
    in_servo_ok = true;

    printf("== PPM, multi-controleurs avec antipatinage\n");
    g_appconf.app_ppm_conf_ctrl_type = 1; g_appconf.app_ppm_conf_multi_esc = 1; g_appconf.app_ppm_conf_tc = 1;
    g_appconf.app_ppm_conf_tc_max_diff = 1000;
    npeers = 2;
    peers[0].id = 1; peers[0].erpm = 5000;
    peers[1].id = 2; peers[1].erpm = 7500;           /* patine : 2500 ERPM de plus que le plus lent */
    set_erpm(5000);
    in_servo_ms = 1.5f; tick(300);
    in_servo_ms = 2.0f;
    for (int k = 0; k < 20; k++) { peers[0].ms = peers[1].ms = now; tick(1); }
    check("consigne recopiee aux deux autres controleurs", can_n > 0 && fabsf(can_last[1] - 1.0f) < 1e-3f);
    check("controleur qui patine : courant reduit a 0", fabsf(can_last[2]) < 1e-3f);
    npeers = 0; g_appconf.app_ppm_conf_multi_esc = 0; set_erpm(0);

    printf("== ADC, gaz ADC1, frein ADC2, boutons\n");
    g_appconf.app_to_use = APP_ADC;
    g_appconf.app_adc_conf_ctrl_type = 8;
    g_appconf.app_adc_conf_voltage_start = 1.0f; g_appconf.app_adc_conf_voltage_end = 3.0f;
    g_appconf.app_adc_conf_voltage_min = 0.5f; g_appconf.app_adc_conf_voltage_max = 3.2f;
    g_appconf.app_adc_conf_voltage2_start = 1.0f; g_appconf.app_adc_conf_voltage2_end = 3.0f;
    g_appconf.app_adc_conf_hyst = 0.05f; g_appconf.app_adc_conf_use_filter = 0; g_appconf.app_adc_conf_safe_start = 0;
    g_appconf.app_adc_conf_ramp_time_pos = 0; g_appconf.app_adc_conf_ramp_time_neg = 0;
    g_appconf.app_adc_conf_throttle_exp = 0; g_appconf.app_adc_conf_throttle_exp_brake = 0;
    g_appconf.app_adc_conf_update_rate_hz = 500; g_appconf.app_adc_conf_multi_esc = 0; g_appconf.app_adc_conf_buttons = 1;
    apps_configure();
    in_adc[1] = 2.0f; in_adc[2] = 1.0f; tick(20);
    check("ADC1 a mi-course : courant relatif ~0.47", !strcmp(last_kind, "cur_rel") && fabsf(last_val - 0.4737f) < 0.02f);
    in_adc[2] = 3.0f; tick(20);
    check("ADC2 a fond : frein", !strcmp(last_kind, "brake_rel") && last_val > 0.95f);
    in_adc[1] = 0.2f; in_adc[2] = 1.0f; n_cmd = 0; tick(20);
    check("ADC1 hors plage (fil coupe) : aucune commande", n_cmd == 0);
    in_adc[1] = 1.0f; set_erpm(3000); in_cc = true; tick(20);
    check("regulateur de vitesse : maintien de 3000 ERPM", !strcmp(last_kind, "rpm") && fabsf(last_val - 3000.0f) < 1.0f);
    in_cc = false; set_erpm(0);
    g_appconf.app_adc_conf_ctrl_type = 3; in_adc[1] = 3.0f; in_rev = true; tick(20);
    check("bouton marche arriere : courant negatif", !strcmp(last_kind, "cur_rel") && last_val < -0.95f);
    in_rev = false;

    printf("== VESC Remote (manette)\n");
    g_appconf.app_to_use = APP_UART;
    g_appconf.app_chuk_conf_ctrl_type = 1; g_appconf.app_chuk_conf_hyst = 0.1f;
    g_appconf.app_chuk_conf_ramp_time_pos = 0; g_appconf.app_chuk_conf_ramp_time_neg = 0;
    g_appconf.app_chuk_conf_throttle_exp = 0; g_appconf.app_chuk_conf_throttle_exp_brake = 0;
    g_appconf.app_chuk_conf_multi_esc = 0; g_appconf.app_chuk_conf_coast_brake_level = 0;
    g_appconf.app_chuk_conf_use_smart_rev = 0;
    apps_configure();
    uint8_t ch[4] = { 128, 255, 0, 0 };
    apps_set_chuck(ch, 4, now); tick(10);
    check("manette en avant : courant +1", !strcmp(last_kind, "cur_rel") && fabsf(last_val - 1.0f) < 0.01f);
    ch[1] = 0; apps_set_chuck(ch, 4, now); tick(10);
    check("manette en arriere : frein", !strcmp(last_kind, "brake_rel"));
    ch[1] = 255; ch[3] = 1; apps_set_chuck(ch, 4, now); tick(10);
    check("bouton Z, moteur arrete : marche arriere", !strcmp(last_kind, "cur_rel") && last_val < -0.99f);
    g_appconf.app_chuk_conf_use_smart_rev = 1; g_appconf.app_chuk_conf_smart_rev_max_duty = 0.1f;
    g_appconf.app_chuk_conf_smart_rev_ramp_time = 0.5f;
    for (int k = 0; k < 6; k++) { apps_set_chuck(ch, 4, now); tick(100); }
    check("marche arriere intelligente : duty -0.1 en rampe", !strcmp(last_kind, "duty") && fabsf(last_val + 0.1f) < 0.005f);
    ch[3] = 0; apps_set_chuck(ch, 4, now); tick(5);
    n_cmd = 0; tick(1200);
    check("plus de donnees depuis plus que le timeout : silence", n_cmd < 1000);

    printf("== PAS (cadence)\n");
    g_appconf.app_to_use = APP_PAS;
    g_appconf.app_pas_conf_ctrl_type = 1; g_appconf.app_pas_conf_magnets = 12;
    g_appconf.app_pas_conf_pedal_rpm_start = 10; g_appconf.app_pas_conf_pedal_rpm_end = 60;
    g_appconf.app_pas_conf_current_scaling = 0.5f; g_appconf.app_pas_conf_use_filter = 0;
    g_appconf.app_pas_conf_ramp_time_pos = 0; g_appconf.app_pas_conf_ramp_time_neg = 0;
    g_appconf.app_pas_conf_update_rate_hz = 50; g_appconf.app_pas_conf_invert_pedal_direction = 0;
    apps_configure();
    /* 35 tr/min, 12 aimants, 4 fronts : 28 fronts/s */
    for (int k = 0; k < 2000; k++) { if (k % 36 == 0) in_pas = 1; tick(1); }
    check("35 tr/min : 50 % du courant x (35-10)/(60-10) = 0.25", !strcmp(last_kind, "cur_rel") && fabsf(last_val - 0.25f) < 0.04f);

    printf("== Interrupteur d'arret et sortie coupee\n");
    g_appconf.app_to_use = APP_PPM; g_appconf.app_ppm_conf_ctrl_type = 1; g_appconf.app_ppm_conf_safe_start = 0;
    g_appconf.kill_sw_mode = 1;
    apps_configure();
    in_servo_lvl = false; tick(5);
    check("arret d'urgence (niveau bas) : relache", !strcmp(last_kind, "release"));
    in_servo_lvl = true; g_appconf.kill_sw_mode = 0;
    apps_disable_output(-1, now); n_cmd = 0; tick(10);
    check("sortie coupee par l'outil : aucune commande", n_cmd == 0);
    apps_disable_output(0, now); tick(10);
    check("sortie reactivee", !strcmp(last_kind, "cur_rel"));

    printf("\n%s (%d echec(s))\n", fails ? "ECHEC" : "TOUS LES TESTS PASSENT", fails);
    return fails ? 1 : 0;
}
