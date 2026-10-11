/*
 * can_vesc.c — Protocole CAN : EID = (commande << 8) | identifiant.
 */
#include <string.h>
#include "can_vesc.h"
#include "can.h"
#include "commands.h"
#include "ctrl.h"
#include "foc.h"
#include "resc_conf.h"
#include "vesc_buffer.h"
#include "vesc_defaults.h"

#define EID(cmd, id) (((uint32_t)(cmd) << 8) | (uint8_t)(id))

static uint8_t s_rxbuf[PACKET_MAX_PL + 8];
static uint32_t s_rxbuf_fill;
static const comm_iface_t *s_fwd_iface;
static uint32_t s_last_status_ms;
static uint32_t s_last_fwd_ms;

/* scan PING */
static const comm_iface_t *s_ping_reply;
static uint16_t s_ping_next;
static uint32_t s_ping_end_ms;
static uint8_t s_ping_found[256];
static uint16_t s_ping_n;
static uint32_t s_ping_last_ms;

uint32_t can_baud_from_enum(uint8_t e)
{
    switch (e) {
    case CAN_BAUD_125K: return 125000;
    case CAN_BAUD_250K: return 250000;
    case CAN_BAUD_1M:   return 1000000;
    case CAN_BAUD_100K: return 100000;
    case CAN_BAUD_50K:  return 50000;
    default:            return 500000;
    }
}

void can_vesc_init(void)
{
    can_init(can_baud_from_enum(g_appconf.can_baud_rate));
}

void can_vesc_set_forward_iface(const comm_iface_t *iface)
{
    s_fwd_iface = iface;
    s_last_fwd_ms = 0;
}

void can_vesc_send_buffer(uint8_t target, const uint8_t *d, uint32_t n, uint8_t mode)
{
    uint8_t me = resc_controller_id();
    uint8_t f[8];
    if (n <= 6) {
        f[0] = me;
        f[1] = mode;
        memcpy(f + 2, d, n);
        can_send(EID(CAN_PACKET_PROCESS_SHORT_BUFFER, target), f, (uint8_t)(n + 2));
        return;
    }
    uint32_t off = 0;
    while (off < n && off < 256) {
        uint32_t k = n - off < 7 ? n - off : 7;
        f[0] = (uint8_t)off;
        memcpy(f + 1, d + off, k);
        can_send(EID(CAN_PACKET_FILL_RX_BUFFER, target), f, (uint8_t)(k + 1));
        off += k;
    }
    while (off < n) {
        uint32_t k = n - off < 6 ? n - off : 6;
        f[0] = (uint8_t)(off >> 8);
        f[1] = (uint8_t)off;
        memcpy(f + 2, d + off, k);
        can_send(EID(CAN_PACKET_FILL_RX_BUFFER_LONG, target), f, (uint8_t)(k + 2));
        off += k;
    }
    uint16_t c = crc16(d, n);
    f[0] = me;
    f[1] = mode;
    f[2] = (uint8_t)(n >> 8);
    f[3] = (uint8_t)n;
    f[4] = (uint8_t)(c >> 8);
    f[5] = (uint8_t)c;
    can_send(EID(CAN_PACKET_PROCESS_RX_BUFFER, target), f, 6);
}

/* Traitement d'un paquet complet recu par CAN */
static void process_packet(uint8_t sender, uint8_t mode, uint8_t *d, uint32_t n)
{
    static comm_iface_t reply;
    static const comm_iface_t silent = { 0, 0, 0, false };
    if (mode == 1) {
        /* reponse a une requete relayee : vers l'interface d'origine */
        if (s_fwd_iface) packet_send(s_fwd_iface, d, n);
        return;
    }
    if (mode == 2) {
        commands_process(&silent, d, n);
        return;
    }
    reply.write = 0;
    reply.is_can = true;
    reply.can_target = sender;
    reply.can_send_mode = 1;
    commands_process(&reply, d, n);
}

static void handle_frame(const can_frame_t *fr)
{
    uint8_t id = (uint8_t)(fr->id & 0xFF);
    uint32_t cmd = fr->id >> 8;
    uint8_t me = resc_controller_id();
    const uint8_t *d = fr->data;
    int32_t i = 0;

    if (cmd == CAN_PACKET_PONG) {
        if (s_ping_reply && id == me && fr->len >= 1 && s_ping_n < 255) s_ping_found[s_ping_n++] = d[0];
        return;
    }
    if (id != me && !(id == 255 && cmd <= CAN_PACKET_SET_CURRENT_HANDBRAKE_REL)) return;

    /* Longueur minimale de chaque commande : une trame tronquee est ignoree */
    switch (cmd) {
    case CAN_PACKET_SET_DUTY: case CAN_PACKET_SET_CURRENT: case CAN_PACKET_SET_CURRENT_BRAKE:
    case CAN_PACKET_SET_RPM: case CAN_PACKET_SET_CURRENT_REL: case CAN_PACKET_SET_CURRENT_BRAKE_REL:
    case CAN_PACKET_SET_CURRENT_HANDBRAKE: case CAN_PACKET_SET_CURRENT_HANDBRAKE_REL: case CAN_PACKET_SET_POS:
        if (fr->len < 4) return;
        break;
    case CAN_PACKET_CONF_CURRENT_LIMITS: case CAN_PACKET_CONF_STORE_CURRENT_LIMITS:
    case CAN_PACKET_CONF_CURRENT_LIMITS_IN: case CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN:
    case CAN_PACKET_CONF_FOC_ERPMS: case CAN_PACKET_CONF_STORE_FOC_ERPMS:
    case CAN_PACKET_CONF_BATTERY_CUT: case CAN_PACKET_CONF_STORE_BATTERY_CUT:
        if (fr->len < 8) return;
        break;
    default:
        break;
    }

    switch (cmd) {
    case CAN_PACKET_SET_DUTY:
        ctrl_set_duty((float)vb_get_i32(d, &i) / 1e5f);
        break;
    case CAN_PACKET_SET_CURRENT:
        ctrl_set_current((float)vb_get_i32(d, &i) / 1e3f);
        if (fr->len >= 6) ctrl_set_off_delay((float)vb_get_i16(d, &i) / 1e3f);
        break;
    case CAN_PACKET_SET_CURRENT_BRAKE:
        ctrl_set_brake((float)vb_get_i32(d, &i) / 1e3f);
        break;
    case CAN_PACKET_SET_RPM:
        ctrl_set_rpm((float)vb_get_i32(d, &i));
        break;
    case CAN_PACKET_SET_POS:
        ctrl_set_pos((float)vb_get_i32(d, &i) / 1e6f);
        break;
    case CAN_PACKET_SET_CURRENT_REL:
        ctrl_set_current_rel((float)vb_get_i32(d, &i) / 1e5f);
        if (fr->len >= 6) ctrl_set_off_delay((float)vb_get_i16(d, &i) / 1e3f);
        break;
    case CAN_PACKET_SET_CURRENT_BRAKE_REL:
        ctrl_set_brake_rel((float)vb_get_i32(d, &i) / 1e5f);
        break;
    case CAN_PACKET_SET_CURRENT_HANDBRAKE:
        ctrl_set_handbrake((float)vb_get_i32(d, &i) / 1e3f);
        break;
    case CAN_PACKET_SET_CURRENT_HANDBRAKE_REL:
        ctrl_set_handbrake((float)vb_get_i32(d, &i) / 1e5f * ctrl_limits()->i_max);
        break;

    case CAN_PACKET_FILL_RX_BUFFER:
        if (fr->len >= 2) {
            uint32_t off = d[0];
            memcpy(s_rxbuf + off, d + 1, fr->len - 1u);
            s_rxbuf_fill = off + fr->len - 1u;
        }
        break;
    case CAN_PACKET_FILL_RX_BUFFER_LONG:
        if (fr->len >= 3) {
            uint32_t off = (uint32_t)d[0] << 8 | d[1];
            if (off + fr->len - 2u <= sizeof(s_rxbuf)) {
                memcpy(s_rxbuf + off, d + 2, fr->len - 2u);
                s_rxbuf_fill = off + fr->len - 2u;
            }
        }
        break;
    case CAN_PACKET_PROCESS_RX_BUFFER:
        if (fr->len >= 6) {
            uint8_t sender = d[0], mode = d[1];
            uint32_t n = (uint32_t)d[2] << 8 | d[3];
            uint16_t c = (uint16_t)((uint16_t)d[4] << 8 | d[5]);
            if (n <= sizeof(s_rxbuf) && crc16(s_rxbuf, n) == c) process_packet(sender, mode, s_rxbuf, n);
            s_rxbuf_fill = 0;
        }
        break;
    case CAN_PACKET_PROCESS_SHORT_BUFFER:
        if (fr->len >= 3) {
            uint8_t tmp[8];
            memcpy(tmp, d + 2, fr->len - 2u);
            process_packet(d[0], d[1], tmp, fr->len - 2u);
        }
        break;

    case CAN_PACKET_PING: {
        uint8_t r[2] = { me, HW_TYPE_VESC };
        if (fr->len >= 1) can_send(EID(CAN_PACKET_PONG, d[0]), r, 2);
        break;
    }

    case CAN_PACKET_CONF_CURRENT_LIMITS:
    case CAN_PACKET_CONF_STORE_CURRENT_LIMITS:
        g_mcconf.l_current_min = vb_get_f32(d, 1e3f, &i);
        g_mcconf.l_current_max = vb_get_f32(d, 1e3f, &i);
        resc_conf_clamp_mc(&g_mcconf);
        resc_conf_apply();
        if (cmd == CAN_PACKET_CONF_STORE_CURRENT_LIMITS) resc_conf_store_mc();
        break;
    case CAN_PACKET_CONF_CURRENT_LIMITS_IN:
    case CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN:
        g_mcconf.l_in_current_min = vb_get_f32(d, 1e3f, &i);
        g_mcconf.l_in_current_max = vb_get_f32(d, 1e3f, &i);
        resc_conf_clamp_mc(&g_mcconf);
        resc_conf_apply();
        if (cmd == CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN) resc_conf_store_mc();
        break;
    case CAN_PACKET_CONF_FOC_ERPMS:
    case CAN_PACKET_CONF_STORE_FOC_ERPMS:
        g_mcconf.foc_openloop_rpm = vb_get_f32(d, 1e3f, &i);
        g_mcconf.foc_sl_erpm = vb_get_f32(d, 1e3f, &i);
        resc_conf_apply();
        if (cmd == CAN_PACKET_CONF_STORE_FOC_ERPMS) resc_conf_store_mc();
        break;
    case CAN_PACKET_CONF_BATTERY_CUT:
    case CAN_PACKET_CONF_STORE_BATTERY_CUT:
        g_mcconf.l_battery_cut_start = vb_get_f32(d, 1e3f, &i);
        g_mcconf.l_battery_cut_end = vb_get_f32(d, 1e3f, &i);
        resc_conf_apply();
        if (cmd == CAN_PACKET_CONF_STORE_BATTERY_CUT) resc_conf_store_mc();
        break;
    default:
        break;
    }
}

static void send_status(void)
{
    const ctrl_stats_t *s = ctrl_stats();
    uint8_t me = resc_controller_id();
    uint8_t m = g_appconf.can_status_msgs_r1;
    uint8_t b[8];
    int32_t i;
    if (m & 0x01) {
        i = 0;
        vb_put_i32(b, (int32_t)s->erpm, &i);
        vb_put_f16(b, s->i_motor, 1e1f, &i);
        vb_put_f16(b, s->duty, 1e3f, &i);
        can_send(EID(CAN_PACKET_STATUS, me), b, 8);
    }
    if (m & 0x02) {
        i = 0;
        vb_put_f32(b, s->ah, 1e4f, &i);
        vb_put_f32(b, s->ah_chg, 1e4f, &i);
        can_send(EID(CAN_PACKET_STATUS_2, me), b, 8);
    }
    if (m & 0x04) {
        i = 0;
        vb_put_f32(b, s->wh, 1e4f, &i);
        vb_put_f32(b, s->wh_chg, 1e4f, &i);
        can_send(EID(CAN_PACKET_STATUS_3, me), b, 8);
    }
    if (m & 0x08) {
        i = 0;
        vb_put_f16(b, s->tfet, 1e1f, &i);
        vb_put_f16(b, s->tmot, 1e1f, &i);
        vb_put_f16(b, s->i_in, 1e1f, &i);
        vb_put_f16(b, 0.0f, 50.0f, &i);
        can_send(EID(CAN_PACKET_STATUS_4, me), b, 8);
    }
    if (m & 0x10) {
        i = 0;
        vb_put_i32(b, s->tacho, &i);
        vb_put_f16(b, s->vbus, 1e1f, &i);
        vb_put_i16(b, 0, &i);
        can_send(EID(CAN_PACKET_STATUS_5, me), b, 8);
    }
}

void can_vesc_start_ping_scan(const comm_iface_t *reply)
{
    s_ping_reply = reply;
    s_ping_next = 0;
    s_ping_n = 0;
    s_ping_end_ms = 0;
}

void can_vesc_poll(uint32_t now)
{
    can_frame_t f;
    for (int k = 0; k < 32 && can_recv(&f); k++) handle_frame(&f);

    uint16_t rate = g_appconf.can_status_rate_1;
    if (rate > 0 && now - s_last_status_ms >= 1000u / rate) {
        s_last_status_ms = now;
        send_status();
    }

    /* Scan PING : 8 requetes par passage, puis 100 ms d'attente des PONG */
    if (s_ping_reply && now != s_ping_last_ms) {
        s_ping_last_ms = now;
        uint8_t me = resc_controller_id();
        for (int k = 0; k < 8 && s_ping_next < 255; k++, s_ping_next++) {
            if (s_ping_next == me) continue;
            can_send(EID(CAN_PACKET_PING, s_ping_next), &me, 1);
        }
        if (s_ping_next >= 255 && s_ping_end_ms == 0) s_ping_end_ms = now + 100;
        if (s_ping_end_ms && (int32_t)(now - s_ping_end_ms) >= 0) {
            uint8_t r[257];
            r[0] = COMM_PING_CAN;
            memcpy(r + 1, s_ping_found, s_ping_n);
            packet_send(s_ping_reply, r, 1u + s_ping_n);
            s_ping_reply = 0;
        }
    }
}
