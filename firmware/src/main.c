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

/* ------------------------------------------------------------ interfaces */
static uart_t *s_uart1, *s_uart2;
static uint32_t w_usb(const uint8_t *d, uint32_t n) { return usb_cdc_write(d, n); }
static uint32_t w_uart1(const uint8_t *d, uint32_t n) { return uart_write(s_uart1, d, n); }
static uint32_t w_uart2(const uint8_t *d, uint32_t n) { return uart_write(s_uart2, d, n); }
static const comm_iface_t IF_USB = { w_usb, 0, 0, false };
static const comm_iface_t IF_UART1 = { w_uart1, 0, 0, false };
static const comm_iface_t IF_UART2 = { w_uart2, 0, 0, false };
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
    tm = ntc_temp(motor_hw_slow_raw(BOARD_TMOT_CH), BOARD_TMOT_MODEL, BOARD_TMOT_R_FIXED_OHM, BOARD_TMOT_NTC_R25_OHM,
                  g_mcconf.m_ntc_motor_beta > 100.0f ? g_mcconf.m_ntc_motor_beta : 3380.0f);
#endif
    ctrl_set_temps(t1, t2, t3, tm);
}

/* ------------------------------------------------------------ bouton (cartes a alimentation maintenue) */
#if BOARD_PWR_KEY_PIN != HW_PIN_NONE
static uint32_t s_key_ms;
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
        if (s_key_ms > 2000u && foc_state() != FOC_ST_RUN && foc_state() != FOC_ST_START) board_power_off();
    }
}
#endif

/* ------------------------------------------------------------ 1 kHz */
static volatile uint32_t s_main_alive;
static uint32_t s_last_alive, s_alive_ms, s_last_foc_ticks;

void app_tick_1khz(void)
{
    uint32_t now = systime_ms();
    ctrl_set_power_allowed(resc_power_allowed() && resc_motor_configured());
    ctrl_update(now, 0.001f);
    if ((now & 15u) == 0) update_temps();
#if BOARD_PWR_KEY_PIN != HW_PIN_NONE
    power_key_tick();
#endif

    /* LED : vert = pret, clignotant en marche ; rouge = defaut */
    bool fault = g_foc_telem.state == FOC_ST_FAULT;
    bool run = g_foc_telem.state == FOC_ST_RUN || g_foc_telem.state == FOC_ST_START;
    board_led(run ? ((now >> 7) & 1u) : !fault, fault ? ((now >> 8) & 1u) : false);

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

    motor_hw_sensor_config(fp.sensor_mode, fp.enc.cpr);
    motor_hw_init(fp.f_pwm);           /* TIM1 + ADC : la calibration des offsets demarre */
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
    wdg_init();
    if (wdg_was_reset()) foc_raise_fault(RESC_FAULT_WATCHDOG_RESET);

    bool awd_set = false;
    uint32_t awd_ms = 0;
    for (;;) {
        s_main_alive++;
        uint32_t now = systime_ms();
        int c;
        for (int k = 0; k < 256 && (c = usb_cdc_getc()) >= 0; k++) packet_dec_byte(&s_dec_usb, (uint8_t)c, now);
        for (int k = 0; k < 256 && (c = uart_getc(s_uart1)) >= 0; k++) packet_dec_byte(&s_dec_u1, (uint8_t)c, now);
        for (int k = 0; k < 256 && (c = uart_getc(s_uart2)) >= 0; k++) packet_dec_byte(&s_dec_u2, (uint8_t)c, now);
        can_vesc_poll(now);
        commands_poll(now);
        resc_conf_poll();
        usb_cdc_poll();

        /* Watchdog analogique : seuils a partir des offsets mesures, mis a jour toutes les 100 ms */
        if (foc_calibrated() && (!awd_set || now - awd_ms > 100u)) {
            const foc_params_t *p = foc_params();
            motor_hw_set_awd(g_foc_telem.off_cs, p->i_abs_trip, p->i_scale);
            awd_set = true;
            awd_ms = now;
        }

        bool idle = foc_state() == FOC_ST_IDLE || foc_state() == FOC_ST_FAULT || foc_state() == FOC_ST_CALIB;
        if (g_req_reboot && idle) {
            for (volatile int i = 0; i < 200000; i++) { }   /* laisse partir la derniere reponse */
            NVIC_SystemReset();
        }
        if (g_req_bootloader && idle) {
            for (volatile int i = 0; i < 200000; i++) { }
            flash_jump_to_bootloader();
            g_req_bootloader = false;                       /* pas de bootloader : on reste */
        }
    }
}
