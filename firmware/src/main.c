/*
 * main.c — Firmware RESC.
 *
 * Niveaux d'execution :
 *   ISR ADC (prio 1)      : boucle FOC complete, en RAM, a chaque periode PWM
 *   ISR break / EXTI (0)  : coupure du pont sur defaut materiel
 *   SysTick (prio 4)      : controle 1 kHz, temperatures, LED, watchdog
 *   UART / CAN / USB      : remplissage des tampons uniquement
 *   boucle principale     : decodage des paquets, reponses, flash
 */
#include <math.h>
#include "stm32.h"
#include "hw_config.h"
#include "platform.h"
#include "clock.h"
#include "board.h"
#include "gpio.h"
#include "systime.h"
#include "motor_hw.h"
#include "uart.h"
#include "usb_cdc.h"
#include "can.h"
#include "flash.h"
#include "wdg.h"
#include "foc.h"
#include "ctrl.h"
#include "resc_conf.h"
#include "packet.h"
#include "commands.h"
#include "can_vesc.h"
#include "fault_codes.h"
#include "apps.h"
#include "vesc_defaults.h"
#include "sampler.h"
#include "stats.h"
#include "imu.h"
#include "bms.h"
#include "abs_enc.h"

/* ------------------------------------------------------------ interfaces */
static uart_t *s_uart1, *s_uart2;
static uint32_t w_usb(const uint8_t *d, uint32_t n) { return usb_cdc_write(d, n); }
static uint32_t w_uart1(const uint8_t *d, uint32_t n) { return uart_write(s_uart1, d, n); }
static uint32_t w_uart2(const uint8_t *d, uint32_t n) { return uart_write(s_uart2, d, n); }
static uint32_t f_usb(void) { return usb_cdc_tx_free(); }
static uint32_t f_uart1(void) { return uart_tx_free(s_uart1); }
static uint32_t f_uart2(void) { return uart_tx_free(s_uart2); }
static const comm_iface_t IF_USB = { w_usb, 0, 0, false, f_usb };
static const comm_iface_t IF_UART1 = { w_uart1, 0, 0, false, f_uart1 };
static const comm_iface_t IF_UART2 = { w_uart2, 0, 0, false, f_uart2 };
static packet_dec_t s_dec_usb, s_dec_u1, s_dec_u2;

uint8_t plat_default_controller_id(void) { return board_uid_hash(); }

/* ------------------------------------------------------------ temperatures */
static float ntc_temp(uint16_t raw, int pos, float r_fixed, float r25, float beta)
{
    float a = (float)raw;
    if (a < 1.0f) a = 1.0f;
    if (a > 4094.0f) a = 4094.0f;
    float r = (pos == HW_TEMP_NTC_TO_VREF) ? r_fixed * (4095.0f - a) / a : r_fixed * a / (4095.0f - a);
    return 1.0f / (logf(r / r25) / beta + 1.0f / 298.15f) - 273.15f;
}

/* Resistance de la sonde moteur a partir de la mesure (pont diviseur de la carte) */
static float sensor_res(uint16_t raw, int pos, float r_fixed)
{
    float a = (float)raw;
    if (a < 1.0f) a = 1.0f;
    if (a > 4094.0f) a = 4094.0f;
    return (pos == HW_TEMP_NTC_TO_VREF) ? r_fixed * (4095.0f - a) / a : r_fixed * a / (4095.0f - a);
}

/* Temperature moteur selon m_motor_temp_sens_type :
 *  NTC (beta), PTC 1k a 100 degC et PTC personnalisee (coefficient %/K), KTY83/122 et
 *  KTY84/130 (polynomes du 2e degre, approximation des tables), PT1000 (IEC 60751). */
static float motor_temp(uint16_t raw)
{
    const vesc_mcconf_t *c = &g_mcconf;
    float r = sensor_res(raw, BOARD_TMOT_MODEL, BOARD_TMOT_R_FIXED_OHM);
    float beta = c->m_ntc_motor_beta > 100.0f ? c->m_ntc_motor_beta : 3380.0f;
    switch (c->m_motor_temp_sens_type) {
    case TEMP_SENSOR_NTC_10K_25C:  return 1.0f / (logf(r / 10000.0f) / beta + 1.0f / 298.15f) - 273.15f;
    case TEMP_SENSOR_NTC_100K_25C: return 1.0f / (logf(r / 100000.0f) / beta + 1.0f / 298.15f) - 273.15f;
    case TEMP_SENSOR_NTCX: {
        float r0 = c->m_ntcx_ptcx_res > 1.0f ? c->m_ntcx_ptcx_res : 10000.0f;
        return 1.0f / (logf(r / r0) / beta + 1.0f / (c->m_ntcx_ptcx_temp_base + 273.15f)) - 273.15f;
    }
    case TEMP_SENSOR_PTC_1K_100C: {
        float k = c->m_ptc_motor_coeff > 0.01f ? c->m_ptc_motor_coeff : 0.61f;
        return 100.0f + (r - 1000.0f) / (1000.0f * k * 0.01f);
    }
    case TEMP_SENSOR_PTCX: {
        float r0 = c->m_ntcx_ptcx_res > 1.0f ? c->m_ntcx_ptcx_res : 1000.0f;
        float k = c->m_ptc_motor_coeff > 0.01f ? c->m_ptc_motor_coeff : 0.61f;
        return c->m_ntcx_ptcx_temp_base + (r - r0) / (r0 * k * 0.01f);
    }
    case TEMP_SENSOR_KTY83_122: {
        /* R = 1000 (1 + 7.874e-3 x + 1.874e-5 x^2), x = T - 25 */
        float a2 = 1.874e-5f, a1 = 7.874e-3f, c0 = 1.0f - r / 1000.0f;
        return 25.0f + (-a1 + sqrtf(fmaxf(a1 * a1 - 4.0f * a2 * c0, 0.0f))) / (2.0f * a2);
    }
    case TEMP_SENSOR_KTY84_130: {
        /* R = 1000 (1 + 6.12e-3 x + 1.1e-5 x^2), x = T - 100 */
        float a2 = 1.1e-5f, a1 = 6.12e-3f, c0 = 1.0f - r / 1000.0f;
        return 100.0f + (-a1 + sqrtf(fmaxf(a1 * a1 - 4.0f * a2 * c0, 0.0f))) / (2.0f * a2);
    }
    case TEMP_SENSOR_PT1000:       return (r / 1000.0f - 1.0f) / 0.00385f;
    default:                       return 25.0f;
    }
}

static void update_temps(void)
{
#if BOARD_TFET_MODEL == HW_TEMP_MAX_OF_3
    const int pos = BOARD_TFET_NTC_POS;
#else
    const int pos = BOARD_TFET_MODEL;
#endif
    float t1 = ntc_temp(motor_hw_slow_raw(BOARD_TFET_CH), pos, BOARD_TFET_R_FIXED_OHM, BOARD_TFET_NTC_R25_OHM, BOARD_TFET_NTC_BETA);
    float t2 = t1, t3 = t1;
#if BOARD_TFET_MODEL == HW_TEMP_MAX_OF_3
    t2 = ntc_temp(motor_hw_slow_raw(BOARD_TFET2_CH), pos, BOARD_TFET_R_FIXED_OHM, BOARD_TFET_NTC_R25_OHM, BOARD_TFET_NTC_BETA);
    t3 = ntc_temp(motor_hw_slow_raw(BOARD_TFET3_CH), pos, BOARD_TFET_R_FIXED_OHM, BOARD_TFET_NTC_R25_OHM, BOARD_TFET_NTC_BETA);
#endif
    float tm = 25.0f;
#if BOARD_TMOT_CH != HW_ADC_CH_NONE
    tm = motor_temp(motor_hw_slow_raw(BOARD_TMOT_CH));
#endif
    ctrl_set_temps(t1, t2, t3, tm);
    foc_set_motor_temp(ctrl_stats()->tmot);
}

/* ------------------------------------------------------------ bouton (cartes a alimentation maintenue) */
#if BOARD_PWR_KEY_PIN != HW_PIN_NONE
static uint32_t s_key_ms, s_rel_ms;
static void power_key_tick(void)
{
    /* Le niveau bas du bouton (~1 V) n'est pas toujours lu comme 0 : on tire la ligne a 0
     * (open-drain), on la relache, et on lit si la resistance de tirage a pu la remonter. */
    static uint8_t phase;
    static uint32_t div;
    if (++div < 10) return;
    div = 0;
    if (phase == 0) {
        gpio_write(BOARD_PWR_KEY_PIN, true);       /* relache */
        phase = 1;
    } else {
        bool pressed = !gpio_read(BOARD_PWR_KEY_PIN);
        gpio_write(BOARD_PWR_KEY_PIN, false);      /* pre-tire a 0 */
        phase = 0;
        s_key_ms = pressed ? s_key_ms + 20u : 0u;
        s_rel_ms = pressed ? 0u : s_rel_ms + 20u;
        /* shutdown_mode : 0 interrupteur (coupe des que relache), 1 jamais coupe par le firmware,
         * 2 appui long, 3..9 appui long ou inactivite (10 s, 1 min, 5, 10, 30 min, 1 h, 5 h) */
        static const uint32_t idle_s[10] = { 0, 0, 0, 10, 60, 300, 600, 1800, 3600, 18000 };
        uint8_t mode = g_appconf.shutdown_mode;
        uint32_t now = systime_ms();
        bool stopped = foc_state() != FOC_ST_RUN && foc_state() != FOC_ST_START && foc_state() != FOC_ST_DETECT;
        bool off = false;
        if (mode == 0) off = s_rel_ms > 500u;
        else if (mode >= 2) off = s_key_ms > 2000u;
        if (mode >= 3 && mode <= 9 && now - apps_last_activity_ms() > idle_s[mode] * 1000u) off = true;
        if (off && stopped) g_req_poweroff = true;     /* boucle principale : odometre enregistre, puis coupure */
    }
}
#endif

/* ------------------------------------------------------------ sortie AUX (m_out_aux_mode) */
static void aux_tick(uint32_t now)
{
#if BOARD_AUX_PIN != HW_PIN_NONE
    const ctrl_stats_t *s = ctrl_stats();
    bool running = g_foc_telem.state == FOC_ST_RUN || g_foc_telem.state == FOC_ST_START;
    bool on;
    switch (g_mcconf.m_out_aux_mode) {
    case 1:  on = now > 2000u; break;
    case 2:  on = now > 5000u; break;
    case 3:  on = now > 10000u; break;
    case 5:  on = running; break;
    case 6:  on = !running; break;
    case 7:  on = s->tmot > 50.0f; break;
    case 8:  on = s->tfet > 50.0f; break;
    case 9:  on = s->tmot > 70.0f; break;
    case 10: on = s->tfet > 70.0f; break;
    case 11: on = s->tmot > 50.0f || s->tfet > 50.0f; break;
    case 12: on = s->tmot > 70.0f || s->tfet > 70.0f; break;
    default: on = false; break;
    }
    gpio_write(BOARD_AUX_PIN, on);
#else
    (void)now;
#endif
}

/* ------------------------------------------------------------ chronometrage (terminal prof) */
#define PROF_N 14
static const char *const s_prof_name[PROF_N] = {
    "usb rx", "uart rx", "codeur abs", "uart2 rx", "can", "commandes", "sampler", "imu", "stats/bms",
    "conf", "usb poll", "awd", "boucle principale (tour)", "boucle 1 kHz (ecart)" };
static volatile uint32_t s_prof_max[PROF_N], s_prof_ms[PROF_N], s_prof_n_slow[PROF_N];
static void prof_note(int i, uint32_t cyc, uint32_t now)
{
    if (cyc > s_prof_max[i]) { s_prof_max[i] = cyc; s_prof_ms[i] = now; }
    if (cyc > 168000u * 5u) s_prof_n_slow[i]++;       /* > 5 ms */
}
#define PROF(i, stmt) do { uint32_t _t0 = DWT->CYCCNT; stmt; prof_note((i), DWT->CYCCNT - _t0, now); } while (0)

int main_prof_count(void) { return PROF_N; }
const char *main_prof_get(int i, uint32_t *max_us, uint32_t *at_ms, uint32_t *n_slow)
{
    if (i < 0 || i >= PROF_N) return 0;
    *max_us = s_prof_max[i] / 168u;
    *at_ms = s_prof_ms[i];
    *n_slow = s_prof_n_slow[i];
    return s_prof_name[i];
}
void main_prof_reset(void)
{
    for (int i = 0; i < PROF_N; i++) { s_prof_max[i] = 0; s_prof_ms[i] = 0; s_prof_n_slow[i] = 0; }
}

/* ------------------------------------------------------------ 1 kHz */
static volatile uint32_t s_main_alive;
static uint32_t s_last_alive, s_alive_ms, s_last_foc_ticks;

void app_tick_1khz(void)
{
    uint32_t now = systime_ms();
    {
        static uint32_t last_cyc;
        uint32_t cyc = DWT->CYCCNT;
        if (last_cyc) prof_note(13, cyc - last_cyc, now);
        last_cyc = cyc;
    }
    ctrl_set_power_allowed(resc_power_allowed() && resc_motor_configured());
    apps_tick_1khz(now);
    ctrl_update(now, 0.001f);
    if ((now & 15u) == 0) update_temps();
    if ((now & 63u) == 0) aux_tick(now);
#if BOARD_PWR_KEY_PIN != HW_PIN_NONE
    power_key_tick();
#endif

    /* LED : defaut = rouge clignotant (2 Hz), marche = vert 100 %,
     * attente = vert a 1/3 (PWM logiciel 333 Hz, sans scintillement visible) */
    foc_state_t st = g_foc_telem.state;
    bool fault = st == FOC_ST_FAULT;
    bool run = st == FOC_ST_RUN || st == FOC_ST_START || st == FOC_ST_DETECT;
    bool dim = (now % 3u) == 0u;
    board_led(!fault && (run || dim), fault && ((now >> 8) & 1u));

    /* Echeance de la boucle FOC : plus de 10 depassements en 1 s -> defaut (une ISR trop longue degrade
     * la regulation sans rien signaler) */
    {
        static uint32_t ov_ref, ov_ms;
        uint32_t ov = motor_hw_overruns();
        if (now - ov_ms >= 1000u || ov < ov_ref) { ov_ms = now; ov_ref = ov; }      /* ov < ref : compteur remis a 0 */
        else if (ov - ov_ref > 10u) { foc_raise_fault(RESC_FAULT_ISR_OVERRUN); ov_ref = ov; }
    }

    /* Watchdog : rafraichi seulement si la boucle FOC et la boucle principale avancent */
    if (s_main_alive != s_last_alive) { s_last_alive = s_main_alive; s_alive_ms = now; }
    bool foc_ok = g_foc_telem.ticks != s_last_foc_ticks;
    s_last_foc_ticks = g_foc_telem.ticks;
    if (foc_ok && now - s_alive_ms < 200u) wdg_kick();
}

/* ------------------------------------------------------------ main */
int main(void)
{
    clock_init();
    board_init();
    SCB->VTOR = 0x08000000u;
    NVIC_SetPriorityGrouping(3);       /* 4 bits de preemption */

    resc_conf_init();
    foc_params_t fp;
    ctrl_limits_t cl;
    resc_conf_build(&g_mcconf, &g_appconf, &fp, &cl);
    foc_init(&fp);
    ctrl_init(&cl);
    ctrl_set_power_allowed(false);

    resc_conf_sensor_hw(&g_mcconf, &fp);
    motor_hw_init(fp.f_pwm, fp.ctl_mode);           /* TIM1 + ADC : la calibration des offsets demarre */
    board_gate_enable(true);           /* driver actif, MOE = 0 : MOSFET bloques */

    systime_init();
    s_uart1 = uart_open(BOARD_UART_INST, BOARD_UART_TX_PIN, BOARD_UART_RX_PIN,
                        g_appconf.app_uart_baudrate ? g_appconf.app_uart_baudrate : 115200u);
#if BOARD_UART2_INST != 0
    s_uart2 = uart_open(BOARD_UART2_INST, BOARD_UART2_TX_PIN, BOARD_UART2_RX_PIN, 115200u);
#endif
    packet_dec_init(&s_dec_usb, &IF_USB);
    packet_dec_init(&s_dec_u1, &IF_UART1);
    packet_dec_init(&s_dec_u2, &IF_UART2);
    usb_cdc_init();
    can_vesc_init();
    apps_configure();
    apps_note_activity(systime_ms());
    stats_reset(systime_ms());
    imu_configure();
    bms_set_fwd_iface(0, &IF_USB);
    wdg_init();
    if (wdg_was_reset()) foc_raise_fault(RESC_FAULT_WATCHDOG_RESET);

    bool awd_set = false;
    uint32_t awd_ms = 0;
    for (;;) {
        s_main_alive++;
        uint32_t now = systime_ms();
        uint32_t loop_t0 = DWT->CYCCNT;
        int c;
        PROF(0, for (int k = 0; k < 256 && (c = usb_cdc_getc()) >= 0; k++) packet_dec_byte(&s_dec_usb, (uint8_t)c, now));
        /* UART du port COMM : paquets, sauf s'il sert au codeur TS5700N8501 */
        if (!abs_enc_uart_owned())
            PROF(1, for (int k = 0; k < 256 && (c = uart_getc(s_uart1)) >= 0; k++) packet_dec_byte(&s_dec_u1, (uint8_t)c, now));
        PROF(2, abs_enc_poll(DWT->CYCCNT / 168u));
        /* UART permanent (module BLE / NRF) : permanent_uart_enabled */
        if (g_appconf.permanent_uart_enabled)
            PROF(3, for (int k = 0; k < 256 && (c = uart_getc(s_uart2)) >= 0; k++) packet_dec_byte(&s_dec_u2, (uint8_t)c, now));
        PROF(4, can_vesc_poll(now));
        PROF(5, commands_poll(now));
        PROF(6, sampler_poll());
        PROF(7, imu_poll(DWT->CYCCNT / 168u));
        {
            static uint32_t st_ms;
            if (now - st_ms >= 10u) {
                st_ms = now;
                PROF(8, stats_update(now); bms_update(now));
                const foc_params_t *fpp = foc_params();
                sampler_set_scale(fpp->i_scale, fpp->v_scale, fpp->vph_scale, fpp->f_pwm);
            }
        }
        PROF(9, resc_conf_poll());
        PROF(10, usb_cdc_poll());

        /* Watchdog analogique : seuils a partir des offsets mesures, mis a jour toutes les 100 ms */
        if (foc_calibrated() && (!awd_set || now - awd_ms > 100u)) {
            const foc_params_t *p = foc_params();
            PROF(11, motor_hw_set_awd(g_foc_telem.off_cs, p->i_abs_trip, p->i_scale));
            awd_set = true;
            awd_ms = now;
        }

        prof_note(12, DWT->CYCCNT - loop_t0, now);
        bool idle = foc_state() == FOC_ST_IDLE || foc_state() == FOC_ST_FAULT || foc_state() == FOC_ST_CALIB;
        if (g_req_reboot && idle) {
            resc_conf_store_backup();                       /* odometre */
            for (volatile int i = 0; i < 200000; i++) { }   /* laisse partir la derniere reponse */
            NVIC_SystemReset();
        }
        if (g_req_poweroff && idle) {
            g_req_poweroff = false;
            resc_conf_store_backup();
            board_power_off();
        }
        if (g_req_bootloader && idle) {
            resc_conf_store_backup();                       /* odometre */
            resc_conf_backup_for_update();                  /* la configuration survit au bootloader */
            for (volatile int i = 0; i < 200000; i++) { }
            flash_jump_to_bootloader();
            g_req_bootloader = false;                       /* pas de bootloader : on reste */
        }
    }
}
