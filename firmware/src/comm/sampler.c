/*
 * sampler.c — Echantillonnage pour VESC Tool (COMM_SAMPLE_PRINT).
 *
 * Reponse, un paquet par echantillon (format du protocole) :
 *   [COMM_SAMPLE_PRINT][index i16][i0 i1 i2 v1 v2 v3 vzero i_filtre : f32 auto][f_sw f32 auto]
 *   [etat u8 : hall << 3][angle u8 : 0..250 pour 0..360 deg][index i32]
 * Mode brut : courants en LSB ADC, tensions en LSB rapportees au point milieu.
 * Les tampons sont en RAM CCM (pas de DMA dessus) ; l'ISR n'ecrit que des entiers.
 */
#include <string.h>
#include "sampler.h"
#include "vesc_buffer.h"
#include "vesc_defaults.h"
#include "platform.h"

enum {
    MODE_OFF = 0, MODE_NOW, MODE_START, MODE_TRIGGER_START, MODE_TRIGGER_FAULT,
    MODE_TRIGGER_START_NOSEND, MODE_TRIGGER_FAULT_NOSEND, MODE_SEND_LAST, MODE_SEND_SINGLE
};

static CCMBSS struct {
    uint16_t cs[SAMPLER_MAX][3];
    uint16_t vph[SAMPLER_MAX][3];
    int16_t  i_f[SAMPLER_MAX];          /* courant filtre, 0.02 A */
    uint8_t  status[SAMPLER_MAX];
    uint8_t  phase[SAMPLER_MAX];
} B;

static volatile struct {
    uint8_t mode, mode_last;
    int len, dec, now, trigger, div;
    bool raw;
    int offset_last;
    bool send_req;
} S;

static struct {
    const comm_iface_t *iface;
    int i, len, offset;
    bool active;
    bool single;
} T;

static float s_i_scale = 1.0f, s_v_scale = 1.0f, s_vph_scale = 1.0f, s_f_loop = 15000.0f;

void sampler_set_scale(float i_scale, float v_scale, float vph_scale, float f_loop)
{
    s_i_scale = i_scale;
    s_v_scale = v_scale;
    s_vph_scale = vph_scale;
    s_f_loop = f_loop;
}

RAMFUNC void sampler_isr(const foc_adc_t *in)
{
    uint8_t mode = S.mode;
    if (mode == MODE_OFF) return;
    bool sample = false;
    bool running = g_foc_telem.state == FOC_ST_RUN || g_foc_telem.state == FOC_ST_START;

    switch (mode) {
    case MODE_NOW:
        if (S.now == S.len) { S.mode = MODE_OFF; S.mode_last = MODE_NOW; S.send_req = true; }
        else sample = true;
        break;
    case MODE_START:
        if (running || S.now > 0) sample = true;
        if (S.now == S.len) { S.mode_last = mode; S.mode = MODE_OFF; S.send_req = true; sample = false; }
        break;
    case MODE_TRIGGER_START: case MODE_TRIGGER_START_NOSEND:
    case MODE_TRIGGER_FAULT: case MODE_TRIGGER_FAULT_NOSEND: {
        sample = true;
        int last = -1;
        if (S.trigger >= 0) {
            last = S.trigger - S.len;
            if (last < 0) last += SAMPLER_MAX;
        }
        if (S.now == last) {
            S.mode_last = mode;
            sample = false;
            if (mode == MODE_TRIGGER_START || mode == MODE_TRIGGER_FAULT) S.send_req = true;
            S.mode = MODE_OFF;
        }
        bool start = mode == MODE_TRIGGER_START || mode == MODE_TRIGGER_START_NOSEND;
        bool ev = start ? running : g_foc_telem.fault != 0;
        if (ev && S.trigger < 0) S.trigger = S.now;
    } break;
    default:
        break;
    }

    if (!sample) return;
    if (++S.div < S.dec) return;
    S.div = 0;
    int k = S.now;
    if (k >= SAMPLER_MAX) k = 0;
    for (int x = 0; x < 3; x++) {
        B.cs[k][x] = in->cs[x];
        B.vph[k][x] = in->vph[x];
    }
    float i = (g_foc_telem.iq < 0.0f ? -1.0f : 1.0f) * g_foc_telem.i_abs;
    B.i_f[k] = (int16_t)(i * 50.0f);
    B.status[k] = (uint8_t)((in->hall & 7u) << 3);
    float th = g_foc_telem.theta;
    if (th < 0.0f) th += 6.2831853f;
    B.phase[k] = (uint8_t)(th * (250.0f / 6.2831853f));
    S.now = k + 1;
}

void sampler_request(const comm_iface_t *reply, const uint8_t *d, uint32_t n)
{
    if (n < 4) return;
    int32_t i = 0;
    uint8_t mode = vb_get_u8(d, &i);
    int len = vb_get_u16(d, &i);
    int dec = vb_get_u8(d, &i);
    bool raw = n > 4 ? d[4] != 0 : false;
    if (len > SAMPLER_MAX) len = SAMPLER_MAX;
    if (len < 1) len = 1;
    if (dec < 1) dec = 1;
    T.iface = reply;
    if (mode == MODE_SEND_LAST) {
        S.send_req = true;
        return;
    }
    if (mode == MODE_SEND_SINGLE) {
        /* un echantillon : 'len' est son index */
        T.active = true;
        T.single = true;
        T.i = len;
        T.len = len + 1;
        T.offset = S.offset_last;
        return;
    }
    S.mode = MODE_OFF;
    S.trigger = -1;
    S.now = 0;
    S.div = 0;
    S.len = len;
    S.dec = dec;
    S.raw = raw;
    S.mode = mode;
}

static void send_block(int ind, int offset)
{
    int k = ind + offset;
    while (k >= SAMPLER_MAX) k -= SAMPLER_MAX;
    while (k < 0) k += SAMPLER_MAX;
    uint8_t b[60];
    int32_t o = 0;
    b[o++] = COMM_SAMPLE_PRINT;
    vb_put_i16(b, (int16_t)ind, &o);
    const float *off = g_foc_telem.off_cs;
    float zr = ((float)B.vph[k][0] + (float)B.vph[k][1] + (float)B.vph[k][2]) * (1.0f / 3.0f);
    if (S.raw) {
        for (int x = 0; x < 3; x++) vb_put_f32_auto(b, (float)B.cs[k][x], &o);
        for (int x = 0; x < 3; x++) vb_put_f32_auto(b, (float)B.vph[k][x] - zr, &o);
        vb_put_f32_auto(b, zr, &o);
        vb_put_f32_auto(b, (float)B.i_f[k] * 0.02f / s_i_scale, &o);
    } else {
        for (int x = 0; x < 3; x++) vb_put_f32_auto(b, ((float)B.cs[k][x] - off[x]) * s_i_scale, &o);
        for (int x = 0; x < 3; x++) vb_put_f32_auto(b, ((float)B.vph[k][x] - zr) * s_vph_scale, &o);
        vb_put_f32_auto(b, zr * s_vph_scale, &o);
        vb_put_f32_auto(b, (float)B.i_f[k] * 0.02f, &o);
    }
    vb_put_f32_auto(b, s_f_loop / (float)(S.dec > 0 ? S.dec : 1), &o);
    b[o++] = B.status[k];
    b[o++] = B.phase[k];
    vb_put_i32(b, ind, &o);
    packet_send(T.iface, b, (uint32_t)o);
}

void sampler_poll(void)
{
    if (S.send_req && !T.active) {
        S.send_req = false;
        int len = 0, offset = 0;
        switch (S.mode_last) {
        case MODE_NOW: case MODE_START:
            len = S.len;
            break;
        case MODE_TRIGGER_START: case MODE_TRIGGER_FAULT:
        case MODE_TRIGGER_START_NOSEND: case MODE_TRIGGER_FAULT_NOSEND:
            len = SAMPLER_MAX;
            offset = S.trigger - S.len;
            break;
        default:
            break;
        }
        S.offset_last = offset;
        T.active = len > 0;
        T.single = false;
        T.i = 0;
        T.len = len;
        T.offset = offset;
    }
    if (!T.active || !T.iface) return;
    /* au rythme de l'interface : quelques paquets par passage, seulement s'ils tiennent dans le tampon */
    for (int n = 0; n < 8 && T.i < T.len; n++) {
        if (T.iface->tx_free && T.iface->tx_free() < 64) break;
        send_block(T.i, T.offset);
        T.i++;
    }
    if (T.i >= T.len) T.active = false;
}
