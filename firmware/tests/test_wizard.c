/*
 * test_wizard.c — Detection automatique (assistant de VESC Tool) de bout en bout sur PC :
 * configuration RESC de la carte, detections et coeur FOC reels, moteur simule (sim_motor).
 *  - R, L et lambda identifies, courant max deduit de la puissance de pertes (sqrt(P / 1.5 R))
 *  - reponse COMM_DETECT_APPLY_ALL_FOC >= 0, configuration moteur renvoyee a l'outil
 *  - gains et gain d'observateur calcules ; puis demarrage et marche sur la configuration trouvee
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "resc_conf.h"
#include "detect_all.h"
#include "detect_app.h"
#include "foc.h"
#include "ctrl.h"
#include "sim_motor.h"
#include "can_vesc.h"
#include "vesc_buffer.h"
#include "vesc_defaults.h"
#include "hw_config.h"

static int fails;
static void check(const char *w, int ok) { printf("  [%s] %s\n", ok ? " OK " : "ECHEC", w); if (!ok) fails++; }

/* ---------------------------------------------------------------- plateforme simulee */
static uint32_t s_ms;
uint32_t systime_ms(void) { return s_ms; }
bool plat_conf_write(uint32_t s, const uint8_t *d, uint32_t n) { (void)s; (void)d; (void)n; return true; }
const uint8_t *plat_conf_read(uint32_t s, uint32_t *n) { (void)s; *n = 0; return 0; }
uint8_t plat_default_controller_id(void) { return 42; }
void plat_pwm_set_freq(float f, uint8_t m) { (void)f; (void)m; }
void plat_sensor_config(uint8_t m, uint32_t c, uint8_t p, const abs_enc_sincos_t *s) { (void)m; (void)c; (void)p; (void)s; }
void plat_hall_samples(uint8_t e) { (void)e; }
static sim_motor_t M;
static foc_pwm_t OUT;
static foc_adc_t ADC;
void foc_hw_phase_filter(bool on) { M.pf_on = on; }
void foc_hw_kill(void) { OUT.enable = false; }
static bool s_verbose;
void commands_printf(const char *fmt, ...)
{
    if (!s_verbose) return;
    va_list ap;
    va_start(ap, fmt);
    printf("    | ");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}
void commands_set_print_iface(const comm_iface_t *i) { (void)i; }
/* reponses vers l'outil */
static int s_res = 9999, s_mcconf_rx;
void packet_send(const comm_iface_t *i, const uint8_t *d, uint32_t n)
{
    (void)i;
    if (d[0] == COMM_DETECT_APPLY_ALL_FOC && n >= 3) { int32_t k = 1; s_res = vb_get_i16(d, &k); }
    if (d[0] == COMM_GET_MCCONF) s_mcconf_rx++;
}
const can_peer_t *can_vesc_peers(int *n) { *n = 0; return 0; }
void can_vesc_conf_in_limits(uint8_t id, float a, float b) { (void)id; (void)a; (void)b; }
void can_vesc_conf_foc_erpms(uint8_t id, float a, float b) { (void)id; (void)a; (void)b; }
void can_vesc_detect_apply_all_foc(uint8_t id, bool a, float p) { (void)id; (void)a; (void)p; }
void can_vesc_detect_result(uint8_t m, int8_t r) { (void)m; (void)r; }

static void run_ms(int ms)
{
    int per = (int)(M.f_pwm / 1000.0 + 0.5);
    for (int k = 0; k < ms; k++) {
        for (int j = 0; j < per; j++) {
            foc_isr(&ADC, &OUT);
            foc_pwm_t h;
            M.hn_ok = foc_isr_half(&h);
            for (int x = 0; x < 3; x++) M.duty_hn[x] = h.duty[x];
            sim_step(&M, &OUT, &ADC);
        }
        ctrl_update(s_ms, 0.001f);
        detect_app_poll(s_ms);
        detect_all_poll(s_ms);
        resc_conf_poll();
        s_ms++;
    }
}

int main(int argc, char **argv)
{
    (void)argv;
    s_verbose = argc > 1;
    printf("== Detection automatique (50 W de pertes) sur le moteur simule\n");
    resc_conf_init();
    foc_params_t fp;
    ctrl_limits_t cl;
    resc_conf_build(&g_mcconf, &g_appconf, &fp, &cl);

    memset(&M, 0, sizeof(M));
    M.r = 0.06; M.ld = 90e-6; M.lq = 110e-6; M.lambda = 0.018; M.pp = 7;
    M.j = 0.004; M.b = 2e-4; M.t_load = 0.02;
    M.vbus = 48.0; M.t_dead = 300e-9; M.f_pwm = fp.f_pwm;
    M.i_lsb = fp.i_scale;
    M.v_lsb = fp.v_scale;
    M.vph_lsb = fp.vph_scale;
    M.off[0] = 2041; M.off[1] = 2055; M.off[2] = 2049;
    M.noise_lsb = 1.0;
    M.pf_tau = 100e-6;
    M.hall_off = 0.4;
    sim_init(&M);

    foc_init(&fp);
    ctrl_init(&cl);
    ctrl_set_power_allowed(true);
    run_ms(500);
    check("calibration des offsets terminee", foc_calibrated() && foc_state() == FOC_ST_IDLE);

    uint8_t req[21];
    int32_t i = 0;
    req[i++] = 0;                       /* pas de CAN */
    vb_put_f32(req, 50.0f, 1e3f, &i);
    vb_put_f32(req, -20.0f, 1e3f, &i);
    vb_put_f32(req, 30.0f, 1e3f, &i);
    vb_put_f32(req, 0.0f, 1e3f, &i);
    vb_put_f32(req, 0.0f, 1e3f, &i);
    static const comm_iface_t tool = { 0, 0, 0, false, 0 };
    detect_all_request(&tool, req, 21);
    for (int k = 0; k < 120 && detect_all_busy(); k++) run_ms(500);

    const vesc_mcconf_t *c = &g_mcconf;
    float imax = sqrtf(50.0f / (float)M.r / 1.5f);
    float cap = (float)HWLIM_I_MOTOR_A * resc_current_cap_frac();
    if (imax > cap) imax = cap;
    printf("  resultat %d ; R=%.4f (vrai %.4f) L=%.1f uH (vrai %.1f) dL=%.1f uH lambda=%.5f (vrai %.5f)\n", s_res,
           c->foc_motor_r, M.r, c->foc_motor_l * 1e6, 0.5 * (M.ld + M.lq) * 1e6, c->foc_motor_ld_lq_diff * 1e6,
           c->foc_motor_flux_linkage, M.lambda);
    printf("  courant max %.1f A (attendu %.1f A), abs %.1f A, kp=%.4f ki=%.2f gain obs %.3g, capteur %u\n",
           c->l_current_max, imax, c->l_abs_current_max, c->foc_current_kp, c->foc_current_ki, c->foc_observer_gain,
           c->foc_sensor_mode);
    check("assistant termine, resultat >= 0", !detect_all_busy() && s_res >= 0 && s_res <= 2);
    check("configuration moteur renvoyee a l'outil", s_mcconf_rx >= 1);
    check("R a +/-10 %", fabs(c->foc_motor_r / M.r - 1) < 0.10);
    check("L a +/-20 %", fabs(c->foc_motor_l / (0.5 * (M.ld + M.lq)) - 1) < 0.20);
    check("lambda a +/-8 %", fabs(c->foc_motor_flux_linkage / M.lambda - 1) < 0.08);
    check("courant max = sqrt(P / 1.5 R) (+/-12 %)", fabsf(c->l_current_max / imax - 1.0f) < 0.12f && c->l_current_min == -c->l_current_max);
    check("gains derives (kp = L bw, ki = R bw)", c->foc_current_kp > 0.0f && fabsf(c->foc_current_ki / (c->foc_motor_r * c->foc_resc_cc_bw) - 1.0f) < 0.01f);
    check("sondes Hall du moteur simule detectees (resultat 1)", s_res == 1 && c->foc_sensor_mode == 2);

    printf("== Marche sur la configuration trouvee (courant 10 A)\n");
    M.b = 0.02; M.t_load = 0.1;
    ctrl_set_current(10.0f);
    for (int k = 0; k < 30; k++) { ctrl_alive(); run_ms(100); }
    printf("  etat %d, erpm %.0f, iq %.1f A\n", foc_state(), g_foc_telem.omega * 60 / 6.2831853, g_foc_telem.iq);
    check("RUN, iq = 10 A (+/-1.5)", foc_state() == FOC_ST_RUN && fabsf(g_foc_telem.iq - 10.0f) < 1.5f);

    printf("\n%s (%d echec(s))\n", fails ? "ECHEC" : "TOUS LES TESTS PASSENT", fails);
    return fails ? 1 : 0;
}
