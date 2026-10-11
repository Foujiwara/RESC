/*
 * bms.c — BMS VESC sur le bus CAN : valeurs, limites du courant batterie, relais.
 */
#include <string.h>
#include "bms.h"
#include "ctrl.h"
#include "can_vesc.h"
#include "resc_conf.h"
#include "vesc_buffer.h"
#include "vesc_defaults.h"

#define MAX_AGE_MS   2000u
#define MAX_CELLS    50
#define MAX_TEMPS    50

typedef struct {
    int id;
    uint32_t t;
    float v_cell_min, v_cell_max, soc, soh, t_cell_max;
    bool charging, balancing, charge_allowed;
    uint8_t data_version;
} stat_t;

static struct {
    int can_id;
    uint32_t t;
    float v_tot, v_charge, i_in, i_in_ic, ah, wh;
    uint8_t cell_num;
    float v_cell[MAX_CELLS];
    bool bal[MAX_CELLS];
    uint8_t temp_num;
    float temps[MAX_TEMPS];
    float temp_ic, temp_hum, hum, pressure, temp_max_cell, soc, soh;
    float ah_chg_tot, wh_chg_tot, ah_dis_tot, wh_dis_tot;
    uint8_t data_version;
    bool charging, balancing, charge_allowed;
    char status[41];
} V = { .can_id = -1 };

static stat_t s_tmax = { .id = -1 }, s_socmin = { .id = -1 }, s_vmin = { .id = -1 }, s_vmax = { .id = -1 };
static const comm_iface_t *s_fwd_any, *s_fwd_usb;

void bms_set_fwd_iface(const comm_iface_t *any, const comm_iface_t *usb)
{
    if (any) s_fwd_any = any;
    if (usb) s_fwd_usb = usb;
}

static bool owns(int id, uint32_t now) { return id == V.can_id || V.can_id < 0 || now - V.t > MAX_AGE_MS; }

static void keep(stat_t *s, const stat_t *m, bool better, uint32_t now)
{
    if (s->id < 0 || now - s->t > MAX_AGE_MS || better || s->id == m->id) *s = *m;
}

bool bms_can_frame(uint8_t cmd, uint8_t id, const uint8_t *d, uint8_t len, uint32_t now)
{
    if (g_mcconf.bms_type != 1) return false;
    bool fwd = cmd == CAN_PACKET_BMS_SOC_SOH_TEMP_STAT || cmd == CAN_PACKET_BMS_V_TOT || cmd == CAN_PACKET_BMS_I ||
               cmd == CAN_PACKET_BMS_AH_WH || cmd == CAN_PACKET_BMS_V_CELL || cmd == CAN_PACKET_BMS_BAL ||
               cmd == CAN_PACKET_BMS_TEMPS || cmd == CAN_PACKET_BMS_HUM;
    if (fwd && g_mcconf.bms_fwd_can_mode) {
        const comm_iface_t *to = g_mcconf.bms_fwd_can_mode == 1 ? s_fwd_usb : s_fwd_any;
        uint8_t b[11];
        b[0] = COMM_BMS_FWD_CAN_RX;
        b[1] = id;
        b[2] = cmd;
        memcpy(b + 3, d, len);
        if (to) packet_send(to, b, 3u + len);
    }
    int32_t i = 0;
    switch (cmd) {
    case CAN_PACKET_BMS_SOC_SOH_TEMP_STAT: {
        stat_t m;
        m.id = id;
        m.t = now;
        m.v_cell_min = vb_get_f16(d, 1e3f, &i);
        m.v_cell_max = vb_get_f16(d, 1e3f, &i);
        m.soc = (float)d[i++] / 255.0f;
        m.soh = (float)d[i++] / 255.0f;
        m.t_cell_max = (float)(int8_t)d[i++];
        uint8_t st = d[i++];
        m.charging = st & 1u;
        m.balancing = (st >> 1) & 1u;
        m.charge_allowed = (st >> 2) & 1u;
        m.data_version = (uint8_t)((st >> 4) & 0x0Fu);
        if (owns(id, now)) {
            V.can_id = id; V.t = now;
            V.soc = m.soc; V.soh = m.soh; V.temp_max_cell = m.t_cell_max;
            V.charging = m.charging; V.balancing = m.balancing; V.charge_allowed = m.charge_allowed;
            V.data_version = m.data_version;
        }
        keep(&s_tmax, &m, m.t_cell_max > s_tmax.t_cell_max, now);
        keep(&s_socmin, &m, m.soc < s_socmin.soc, now);
        keep(&s_vmin, &m, m.v_cell_min < s_vmin.v_cell_min, now);
        keep(&s_vmax, &m, m.v_cell_max > s_vmax.v_cell_max, now);
        return true;
    }
    case CAN_PACKET_BMS_V_TOT:
        if (owns(id, now)) { V.can_id = id; V.t = now; V.v_tot = vb_get_f32_auto(d, &i); V.v_charge = vb_get_f32_auto(d, &i); }
        return true;
    case CAN_PACKET_BMS_I:
        if (owns(id, now)) { V.can_id = id; V.t = now; V.i_in = vb_get_f32_auto(d, &i); V.i_in_ic = vb_get_f32_auto(d, &i); }
        return true;
    case CAN_PACKET_BMS_AH_WH:
        if (owns(id, now)) { V.can_id = id; V.t = now; V.ah = vb_get_f32_auto(d, &i); V.wh = vb_get_f32_auto(d, &i); }
        return true;
    case CAN_PACKET_BMS_V_CELL:
        if (owns(id, now) && len >= 2) {
            V.can_id = id; V.t = now;
            unsigned ofs = d[0];
            V.cell_num = d[1] > MAX_CELLS ? MAX_CELLS : d[1];
            for (i = 2; i + 1 < len && ofs < MAX_CELLS; ofs++) V.v_cell[ofs] = vb_get_f16(d, 1e3f, &i);
        }
        return true;
    case CAN_PACKET_BMS_BAL:
        if (owns(id, now) && len >= 8) {
            V.can_id = id; V.t = now;
            int n = d[0];
            uint64_t b0 = vb_get_u32(d, &i) & 0x00FFFFFFu, b1 = vb_get_u32(d, &i);
            uint64_t st = b0 << 32 | b1;
            for (int k = 0; k < MAX_CELLS && k < n; k++) V.bal[k] = (st >> k) & 1u;
        }
        return true;
    case CAN_PACKET_BMS_TEMPS:
        if (owns(id, now) && len >= 2) {
            V.can_id = id; V.t = now;
            unsigned ofs = d[0];
            V.temp_num = d[1] > MAX_TEMPS ? MAX_TEMPS : d[1];
            for (i = 2; i + 1 < len && ofs < MAX_TEMPS; ofs++) V.temps[ofs] = vb_get_f16(d, 1e2f, &i);
        }
        return true;
    case CAN_PACKET_BMS_HUM:
        if (owns(id, now)) {
            V.can_id = id; V.t = now;
            V.temp_hum = vb_get_f16(d, 1e2f, &i);
            V.hum = vb_get_f16(d, 1e2f, &i);
            V.temp_ic = vb_get_f16(d, 1e2f, &i);
            if (len == 8) V.pressure = vb_get_f16(d, 1e-1f, &i);
        }
        return true;
    case CAN_PACKET_BMS_AH_WH_CHG_TOTAL:
        if (owns(id, now)) { V.can_id = id; V.t = now; V.ah_chg_tot = vb_get_f32_auto(d, &i); V.wh_chg_tot = vb_get_f32_auto(d, &i); }
        return true;
    case CAN_PACKET_BMS_AH_WH_DIS_TOTAL:
        if (owns(id, now)) { V.can_id = id; V.t = now; V.ah_dis_tot = vb_get_f32_auto(d, &i); V.wh_dis_tot = vb_get_f32_auto(d, &i); }
        return true;
    case CAN_PACKET_BMS_STATUS_1: case CAN_PACKET_BMS_STATUS_2: case CAN_PACKET_BMS_STATUS_3:
    case CAN_PACKET_BMS_STATUS_4: case CAN_PACKET_BMS_STATUS_5:
        if (owns(id, now)) {
            V.can_id = id; V.t = now;
            unsigned o = (unsigned)(cmd - CAN_PACKET_BMS_STATUS_1) * 8u;
            memcpy(V.status + o, d, len);
            V.status[40] = 0;
        }
        return true;
    default:
        return false;
    }
}

static float lin(float x, float x0, float x1, float y0, float y1) { return y0 + (x - x0) * (y1 - y0) / (x1 - x0); }

void bms_update(uint32_t now)
{
    const vesc_mcconf_t *c = &g_mcconf;
    float hi = 1e9f, lo = -1e9f;
    if (c->bms_type == 1) {
        float imax = c->l_in_current_max, imin = c->l_in_current_min;
        uint8_t m = c->bms_limit_mode;
        /* temperature : decharge et regeneration */
        if ((m & 1u) && s_tmax.id >= 0 && now - s_tmax.t <= MAX_AGE_MS) {
            float t = s_tmax.t_cell_max, mc = imax > -imin ? imax : -imin;
            if (t > c->bms_t_limit_end - 0.1f) { hi = 0.0f; lo = 0.0f; }
            else if (t >= c->bms_t_limit_start + 0.1f) {
                float k = lin(t, c->bms_t_limit_start, c->bms_t_limit_end, mc, 0.0f);
                if (k < hi) hi = k;
                if (-k > lo) lo = -k;
            }
        }
        /* etat de charge : decharge */
        if ((m & 2u) && s_socmin.id >= 0 && now - s_socmin.t <= MAX_AGE_MS) {
            float s = s_socmin.soc;
            if (s < c->bms_soc_limit_end + 0.001f) hi = 0.0f;
            else if (s <= c->bms_soc_limit_start - 0.001f) {
                float k = lin(s, c->bms_soc_limit_start, c->bms_soc_limit_end, imax, 0.0f);
                if (k < hi) hi = k;
            }
        }
        /* cellule la plus basse : decharge */
        if ((m & 4u) && s_vmin.id >= 0 && now - s_vmin.t <= MAX_AGE_MS) {
            float v = s_vmin.v_cell_min;
            if (v < c->bms_vmin_limit_end + 0.001f) hi = 0.0f;
            else if (v <= c->bms_vmin_limit_start - 0.001f) {
                float k = lin(v, c->bms_vmin_limit_start, c->bms_vmin_limit_end, imax, 0.0f);
                if (k < hi) hi = k;
            }
        }
        /* cellule la plus haute : regeneration */
        if ((m & 8u) && s_vmax.id >= 0 && now - s_vmax.t <= MAX_AGE_MS) {
            float v = s_vmax.v_cell_max;
            if (v > c->bms_vmax_limit_end - 0.001f) lo = 0.0f;
            else if (v >= c->bms_vmax_limit_start + 0.001f) {
                float k = lin(v, c->bms_vmax_limit_start, c->bms_vmax_limit_end, imin, 0.0f);
                if (k > lo) lo = k;
            }
        }
        if (hi < 0.0f) hi = 0.0f;
        if (lo > 0.0f) lo = 0.0f;
    }
    ctrl_set_bms_limits(lo, hi);
}

void bms_command(const comm_iface_t *src, const uint8_t *data, uint32_t len, uint32_t now)
{
    if (len < 1) return;
    uint8_t id = data[0];
    if (id == COMM_BMS_GET_VALUES) {
        static uint8_t b[300];
        int32_t o = 0;
        b[o++] = id;
        vb_put_f32(b, V.v_tot, 1e6f, &o);
        vb_put_f32(b, V.v_charge, 1e6f, &o);
        vb_put_f32(b, V.i_in, 1e6f, &o);
        vb_put_f32(b, V.i_in_ic, 1e6f, &o);
        vb_put_f32(b, V.ah, 1e3f, &o);
        vb_put_f32(b, V.wh, 1e3f, &o);
        b[o++] = V.cell_num;
        for (int k = 0; k < V.cell_num; k++) vb_put_f16(b, V.v_cell[k], 1e3f, &o);
        for (int k = 0; k < V.cell_num; k++) b[o++] = V.bal[k] ? 1 : 0;
        b[o++] = V.temp_num;
        for (int k = 0; k < V.temp_num; k++) vb_put_f16(b, V.temps[k], 1e2f, &o);
        vb_put_f16(b, V.temp_ic, 1e2f, &o);
        vb_put_f16(b, V.temp_hum, 1e2f, &o);
        vb_put_f16(b, V.hum, 1e2f, &o);
        vb_put_f16(b, V.temp_max_cell, 1e2f, &o);
        vb_put_f16(b, V.soc, 1e3f, &o);
        vb_put_f16(b, V.soh, 1e3f, &o);
        b[o++] = (uint8_t)V.can_id;
        vb_put_f32_auto(b, V.ah_chg_tot, &o);
        vb_put_f32_auto(b, V.wh_chg_tot, &o);
        vb_put_f32_auto(b, V.ah_dis_tot, &o);
        vb_put_f32_auto(b, V.wh_dis_tot, &o);
        vb_put_f16(b, V.pressure, 1e-1f, &o);
        b[o++] = V.data_version;
        size_t sl = strnlen(V.status, sizeof(V.status) - 1);
        memcpy(b + o, V.status, sl);
        o += (int32_t)sl;
        b[o++] = 0;
        packet_send(src, b, (uint32_t)o);
        return;
    }
    /* commandes relayees au BMS actif */
    if (g_mcconf.bms_type == 1 && V.can_id >= 0 && now - V.t < MAX_AGE_MS &&
        (id == COMM_BMS_SET_CHARGE_ALLOWED || id == COMM_BMS_SET_BALANCE_OVERRIDE || id == COMM_BMS_RESET_COUNTERS ||
         id == COMM_BMS_FORCE_BALANCE || id == COMM_BMS_ZERO_CURRENT_OFFSET))
        can_vesc_send_buffer((uint8_t)V.can_id, data, len, 0);
}
