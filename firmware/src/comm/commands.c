/*
 * commands.c — Traitement des paquets du protocole (USB, UART, BLE, CAN).
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "commands.h"
#include "can_vesc.h"
#include "terminal.h"
#include "detect_app.h"
#include "ctrl.h"
#include "foc.h"
#include "resc_conf.h"
#include "vesc_buffer.h"
#include "vesc_defaults.h"
#include "hw_config.h"
#include "platform.h"
#include "board.h"
#include "flash.h"
#include "systime.h"
#include "apps.h"
#include "conf_desc.h"

volatile bool g_req_reboot, g_req_bootloader;

static uint8_t s_tx[PACKET_MAX_PL + 16];
static const comm_iface_t *s_term_iface;

void commands_init(void) { }

static void reply(const comm_iface_t *src, const uint8_t *d, uint32_t n) { packet_send(src, d, n); }

void commands_printf(const char *fmt, ...)
{
    if (!s_term_iface) return;
    char buf[256];
    buf[0] = (char)COMM_PRINT;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + 1, sizeof(buf) - 1, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 2) n = (int)sizeof(buf) - 2;
    packet_send(s_term_iface, (const uint8_t *)buf, (uint32_t)n + 1);
}

/* ------------------------------------------------------ valeurs temps reel */
static float temp_fet(void) { return ctrl_stats()->tfet; }

static int32_t put_values(uint8_t *b, int32_t i, uint32_t mask)
{
    const ctrl_stats_t *s = ctrl_stats();
    const foc_telem_t *t = &g_foc_telem;
    if (mask & (1u << 0))  vb_put_f16(b, temp_fet(), 1e1f, &i);
    if (mask & (1u << 1))  vb_put_f16(b, s->tmot, 1e1f, &i);
    if (mask & (1u << 2))  vb_put_f32(b, s->i_motor, 1e2f, &i);
    if (mask & (1u << 3))  vb_put_f32(b, s->i_in, 1e2f, &i);
    if (mask & (1u << 4))  vb_put_f32(b, t->id, 1e2f, &i);
    if (mask & (1u << 5))  vb_put_f32(b, t->iq, 1e2f, &i);
    if (mask & (1u << 6))  vb_put_f16(b, s->duty, 1e3f, &i);
    if (mask & (1u << 7))  vb_put_f32(b, s->erpm, 1e0f, &i);
    if (mask & (1u << 8))  vb_put_f16(b, s->vbus, 1e1f, &i);
    if (mask & (1u << 9))  vb_put_f32(b, s->ah, 1e4f, &i);
    if (mask & (1u << 10)) vb_put_f32(b, s->ah_chg, 1e4f, &i);
    if (mask & (1u << 11)) vb_put_f32(b, s->wh, 1e4f, &i);
    if (mask & (1u << 12)) vb_put_f32(b, s->wh_chg, 1e4f, &i);
    if (mask & (1u << 13)) vb_put_i32(b, s->tacho, &i);
    if (mask & (1u << 14)) vb_put_i32(b, s->tacho_abs, &i);
    if (mask & (1u << 15)) b[i++] = (uint8_t)t->fault;
    if (mask & (1u << 16)) vb_put_f32(b, 0.0f, 1e6f, &i);
    if (mask & (1u << 17)) b[i++] = resc_controller_id();
    if (mask & (1u << 18)) {
        vb_put_f16(b, s->tfet1, 1e1f, &i);
        vb_put_f16(b, s->tfet2, 1e1f, &i);
        vb_put_f16(b, s->tfet3, 1e1f, &i);
    }
    if (mask & (1u << 19)) vb_put_f32(b, t->vd, 1e3f, &i);
    if (mask & (1u << 20)) vb_put_f32(b, t->vq, 1e3f, &i);
    if (mask & (1u << 21)) b[i++] = (uint8_t)(s->timeout ? 1 : 0);
    return i;
}

static float speed_ms(void)
{
    float pp = g_mcconf.si_motor_poles > 1.0f ? g_mcconf.si_motor_poles * 0.5f : 1.0f;
    float gear = g_mcconf.si_gear_ratio > 0.0f ? g_mcconf.si_gear_ratio : 1.0f;
    float rpm = ctrl_stats()->erpm / pp / gear;
    return rpm * 3.14159265f * g_mcconf.si_wheel_diameter / 60.0f;
}

static float battery_level(void)
{
    float cells = g_mcconf.si_battery_cells > 0.0f ? g_mcconf.si_battery_cells : 1.0f;
    float v = ctrl_stats()->vbus / cells;
    float l = (v - 3.0f) / (4.2f - 3.0f);
    return l < 0.0f ? 0.0f : (l > 1.0f ? 1.0f : l);
}

static int32_t put_setup_values(uint8_t *b, int32_t i, uint32_t mask)
{
    const ctrl_stats_t *s = ctrl_stats();
    float dist = (float)s->tacho / (6.0f * (g_mcconf.si_motor_poles > 1.0f ? g_mcconf.si_motor_poles * 0.5f : 1.0f)) /
                 (g_mcconf.si_gear_ratio > 0.0f ? g_mcconf.si_gear_ratio : 1.0f) * 3.14159265f * g_mcconf.si_wheel_diameter;
    float dist_abs = dist * ((float)s->tacho_abs / (s->tacho != 0 ? (float)(s->tacho < 0 ? -s->tacho : s->tacho) : 1.0f));
    if (mask & (1u << 0))  vb_put_f16(b, temp_fet(), 1e1f, &i);
    if (mask & (1u << 1))  vb_put_f16(b, s->tmot, 1e1f, &i);
    if (mask & (1u << 2))  vb_put_f32(b, s->i_motor, 1e2f, &i);
    if (mask & (1u << 3))  vb_put_f32(b, s->i_in, 1e2f, &i);
    if (mask & (1u << 4))  vb_put_f16(b, s->duty, 1e3f, &i);
    if (mask & (1u << 5))  vb_put_f32(b, s->erpm, 1e0f, &i);
    if (mask & (1u << 6))  vb_put_f32(b, speed_ms(), 1e3f, &i);
    if (mask & (1u << 7))  vb_put_f16(b, s->vbus, 1e1f, &i);
    if (mask & (1u << 8))  vb_put_f16(b, battery_level(), 1e3f, &i);
    if (mask & (1u << 9))  vb_put_f32(b, s->ah, 1e4f, &i);
    if (mask & (1u << 10)) vb_put_f32(b, s->ah_chg, 1e4f, &i);
    if (mask & (1u << 11)) vb_put_f32(b, s->wh, 1e4f, &i);
    if (mask & (1u << 12)) vb_put_f32(b, s->wh_chg, 1e4f, &i);
    if (mask & (1u << 13)) vb_put_f32(b, dist, 1e3f, &i);
    if (mask & (1u << 14)) vb_put_f32(b, dist_abs, 1e3f, &i);
    if (mask & (1u << 15)) vb_put_f32(b, 0.0f, 1e6f, &i);
    if (mask & (1u << 16)) b[i++] = (uint8_t)g_foc_telem.fault;
    if (mask & (1u << 17)) b[i++] = resc_controller_id();
    if (mask & (1u << 18)) b[i++] = 1;
    if (mask & (1u << 19)) vb_put_f32(b, 0.0f, 1e3f, &i);
    if (mask & (1u << 20)) vb_put_u32(b, (uint32_t)(dist_abs > 0.0f ? dist_abs : 0.0f), &i);
    if (mask & (1u << 21)) vb_put_u32(b, systime_ms(), &i);
    return i;
}

/* ------------------------------------------------------ traitement */
void commands_process(const comm_iface_t *src, uint8_t *data, uint32_t len)
{
    if (len < 1) return;
    uint8_t id = data[0];
    uint8_t *d = data + 1;
    uint32_t n = len - 1;
    int32_t i = 0;
    int32_t o = 0;
    uint8_t *b = s_tx;

    switch (id) {
    case COMM_SET_DUTY: case COMM_SET_CURRENT: case COMM_SET_CURRENT_BRAKE: case COMM_SET_RPM:
    case COMM_SET_HANDBRAKE: case COMM_SET_CURRENT_REL: case COMM_SET_POS: case COMM_ALIVE:
    case COMM_SET_CHUCK_DATA:
        apps_note_activity(systime_ms());       /* arret automatique : activite */
        break;
    default:
        break;
    }

    switch (id) {
    case COMM_FW_VERSION: {
        b[o++] = COMM_FW_VERSION;
        b[o++] = RESC_FW_MAJOR;
        b[o++] = RESC_FW_MINOR;
        size_t hn = strlen(BOARD_NAME);
        memcpy(b + o, BOARD_NAME, hn + 1);
        o += (int32_t)hn + 1;
        memcpy(b + o, board_uid(), 12);
        o += 12;
        b[o++] = 0;                  /* pairing_done */
        b[o++] = 0;                  /* version de test */
        b[o++] = HW_TYPE_VESC;
        b[o++] = 0;                  /* configurations personnalisees */
        b[o++] = BOARD_PHASE_FILTER_PIN != HW_PIN_NONE ? 1 : 0;
        b[o++] = 0;                  /* QML materiel */
        b[o++] = 0;                  /* QML application */
        b[o++] = 0;                  /* nrf */
        memcpy(b + o, RESC_FW_NAME, sizeof(RESC_FW_NAME));
        o += (int32_t)sizeof(RESC_FW_NAME);
        vb_put_u32(b, 0, &o);        /* crc materiel */
        reply(src, b, (uint32_t)o);
        break;
    }

    case COMM_GET_VALUES:
    case COMM_GET_VALUES_SELECTIVE: {
        uint32_t mask = 0xFFFFFFFFu;
        b[o++] = id;
        if (id == COMM_GET_VALUES_SELECTIVE && n >= 4) {
            mask = vb_get_u32(d, &i);
            vb_put_u32(b, mask, &o);
        }
        o = put_values(b, o, mask);
        reply(src, b, (uint32_t)o);
        break;
    }

    case COMM_GET_VALUES_SETUP:
    case COMM_GET_VALUES_SETUP_SELECTIVE: {
        uint32_t mask = 0xFFFFFFFFu;
        b[o++] = id;
        if (id == COMM_GET_VALUES_SETUP_SELECTIVE && n >= 4) {
            mask = vb_get_u32(d, &i);
            vb_put_u32(b, mask, &o);
        }
        o = put_setup_values(b, o, mask);
        reply(src, b, (uint32_t)o);
        break;
    }

    case COMM_SET_DUTY:          if (n >= 4) ctrl_set_duty((float)vb_get_i32(d, &i) / 1e5f); break;
    case COMM_SET_CURRENT:       if (n >= 4) ctrl_set_current((float)vb_get_i32(d, &i) / 1e3f); break;
    case COMM_SET_CURRENT_BRAKE: if (n >= 4) ctrl_set_brake((float)vb_get_i32(d, &i) / 1e3f); break;
    case COMM_SET_RPM:           if (n >= 4) ctrl_set_rpm((float)vb_get_i32(d, &i)); break;
    case COMM_SET_HANDBRAKE:     if (n >= 4) ctrl_set_handbrake((float)vb_get_i32(d, &i) / 1e3f); break;
    case COMM_SET_CURRENT_REL:   if (n >= 4) ctrl_set_current_rel((float)vb_get_i32(d, &i) / 1e5f); break;
    case COMM_SET_POS:           if (n >= 4) ctrl_set_pos((float)vb_get_i32(d, &i) / 1e6f); break;
    case COMM_ALIVE:             ctrl_alive(); break;
    case COMM_MOTOR_ESTOP:       ctrl_estop(); break;

    case COMM_GET_MCCONF:
    case COMM_GET_MCCONF_DEFAULT: {
        static vesc_mcconf_t tmp;
        const vesc_mcconf_t *c = &g_mcconf;
        if (id == COMM_GET_MCCONF_DEFAULT) { resc_conf_defaults_mc(&tmp); c = &tmp; }
        b[o++] = id;
        o += vesc_mcconf_serialize(b + o, c);
        reply(src, b, (uint32_t)o);
        break;
    }
    case COMM_SET_MCCONF: {
        static vesc_mcconf_t tmp;
        if (!vesc_mcconf_deserialize(d, (int32_t)n, &tmp)) {
            commands_printf("SET_MCCONF refuse : signature ou taille incorrecte");
            break;
        }
        resc_conf_clamp_mc(&tmp);
        g_mcconf = tmp;
        resc_conf_apply();
        resc_conf_store_mc();
        b[o++] = COMM_SET_MCCONF;
        reply(src, b, 1);
        break;
    }
    case COMM_GET_APPCONF:
    case COMM_GET_APPCONF_DEFAULT: {
        static vesc_appconf_t tmp;
        const vesc_appconf_t *c = &g_appconf;
        if (id == COMM_GET_APPCONF_DEFAULT) { resc_conf_defaults_app(&tmp); c = &tmp; }
        b[o++] = id;
        o += vesc_appconf_serialize(b + o, c);
        reply(src, b, (uint32_t)o);
        break;
    }
    case COMM_SET_APPCONF:
    case COMM_SET_APPCONF_NO_STORE: {
        static vesc_appconf_t tmp;
        if (!vesc_appconf_deserialize(d, (int32_t)n, &tmp)) {
            commands_printf("SET_APPCONF refuse : signature ou taille incorrecte");
            break;
        }
        resc_conf_clamp_app(&tmp);
        g_appconf = tmp;
        resc_conf_apply();
        apps_configure();
        if (id == COMM_SET_APPCONF) resc_conf_store_app();
        b[o++] = id;
        reply(src, b, 1);
        break;
    }

    case COMM_TERMINAL_CMD:
    case COMM_TERMINAL_CMD_SYNC: {
        char line[200];
        uint32_t k = n < sizeof(line) - 1 ? n : sizeof(line) - 1;
        memcpy(line, d, k);
        line[k] = 0;
        s_term_iface = src;
        terminal_process(line);
        break;
    }

    case COMM_DETECT_MOTOR_R_L:
        detect_app_start_rl(src, 0.0f);   /* la requete ne porte pas de courant */
        break;
    case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP: {
        if (n < 16) break;
        float cur = vb_get_f32(d, 1e3f, &i);
        float erpm_s = vb_get_f32(d, 1e3f, &i);
        float duty = vb_get_f32(d, 1e3f, &i);
        float r = vb_get_f32(d, 1e6f, &i);
        float l = n >= 20 ? vb_get_f32(d, 1e8f, &i) : 0.0f;
        detect_app_start_lambda(src, cur, erpm_s, duty, r, l);
        break;
    }
    case COMM_DETECT_APPLY_ALL_FOC:
        /* Pas d'auto-detection ni d'auto-configuration dans RESC : detections manuelles uniquement */
        s_term_iface = src;
        commands_printf("RESC : detection automatique non supportee. Utiliser les detections manuelles "
                        "(offsets, R/L, flux, Hall, codeur).");
        b[o++] = COMM_DETECT_APPLY_ALL_FOC;
        vb_put_i16(b, -1, &o);
        reply(src, b, (uint32_t)o);
        break;

    case COMM_DETECT_HALL_FOC:
        detect_app_start_hall(src, n >= 4 ? vb_get_f32(d, 1e3f, &i) : 0.0f);
        break;
    case COMM_DETECT_ENCODER:
        detect_app_start_encoder(src, n >= 4 ? vb_get_f32(d, 1e3f, &i) : 0.0f);
        break;

    case COMM_FORWARD_CAN:
        if (n >= 2) {
            can_vesc_set_forward_iface(src);
            can_vesc_send_buffer(d[0], d + 1, n - 1, 0);
        }
        break;
    case COMM_PING_CAN:
        can_vesc_start_ping_scan(src);
        break;

    case COMM_REBOOT:
        g_req_reboot = true;
        break;

    case COMM_JUMP_TO_BOOTLOADER:
        g_req_bootloader = true;
        break;
    case COMM_ERASE_NEW_APP: {
        bool ok = false;
        if (n >= 4 && foc_state() != FOC_ST_RUN && foc_state() != FOC_ST_START && foc_state() != FOC_ST_DETECT) {
            uint32_t sz = vb_get_u32(d, &i);
            ctrl_release();
            ok = sz <= FLASH_NEWAPP_SIZE && flash_erase_new_app(sz);
        }
        b[o++] = COMM_ERASE_NEW_APP;
        b[o++] = ok ? 1 : 0;
        reply(src, b, (uint32_t)o);
        break;
    }
    case COMM_WRITE_NEW_APP_DATA: {
        bool ok = false;
        uint32_t off = 0;
        if (n >= 4) {
            off = vb_get_u32(d, &i);
            ok = foc_state() != FOC_ST_RUN && flash_write_new_app(off, d + 4, n - 4);
        }
        b[o++] = COMM_WRITE_NEW_APP_DATA;
        b[o++] = ok ? 1 : 0;
        vb_put_u32(b, off, &o);
        reply(src, b, (uint32_t)o);
        break;
    }
    case COMM_WRITE_NEW_APP_DATA_LZO: {
        /* compression non supportee : l'outil reessaie sans compression */
        b[o++] = COMM_WRITE_NEW_APP_DATA_LZO;
        b[o++] = 0;
        vb_put_u32(b, n >= 4 ? vb_get_u32(d, &i) : 0, &o);
        reply(src, b, (uint32_t)o);
        break;
    }

    case COMM_GET_DECODED_PPM: {
        b[o++] = id;
        vb_put_i32(b, (int32_t)(apps_ppm_decoded() * 1e6f), &o);
        vb_put_i32(b, (int32_t)(apps_ppm_pulse_ms() * 1e6f), &o);
        reply(src, b, (uint32_t)o);
        break;
    }
    case COMM_GET_DECODED_ADC: {
        b[o++] = id;
        vb_put_i32(b, (int32_t)(apps_adc_decoded(1) * 1e6f), &o);
        vb_put_i32(b, (int32_t)(apps_adc_voltage(1) * 1e6f), &o);
        vb_put_i32(b, (int32_t)(apps_adc_decoded(2) * 1e6f), &o);
        vb_put_i32(b, (int32_t)(apps_adc_voltage(2) * 1e6f), &o);
        reply(src, b, (uint32_t)o);
        break;
    }
    case COMM_GET_DECODED_CHUK: {
        b[o++] = id;
        vb_put_i32(b, (int32_t)(apps_chuk_decoded() * 1e6f), &o);
        reply(src, b, (uint32_t)o);
        break;
    }
    case COMM_SET_CHUCK_DATA:
        apps_set_chuck(d, n, systime_ms());
        break;
    case COMM_RESC_GET_CONF_XML:
        o = conf_desc_reply(d, n, b);
        reply(src, b, (uint32_t)o);
        break;
    case COMM_APP_DISABLE_OUTPUT:
        if (n >= 4) apps_disable_output(vb_get_i32(d, &i), systime_ms());
        break;

    default:
        break;    /* commandes non prises en charge : ignorees */
    }
}

void commands_poll(uint32_t now_ms) { detect_app_poll(now_ms); }
