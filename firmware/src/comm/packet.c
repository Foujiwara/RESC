#include <string.h>
#include "packet.h"
#include "commands.h"
#include "can_vesc.h"

uint16_t crc16(const uint8_t *d, uint32_t n)
{
    uint16_t c = 0;
    for (uint32_t i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int k = 0; k < 8; k++) c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
    }
    return c;
}

void packet_dec_init(packet_dec_t *p, const comm_iface_t *iface)
{
    p->n = 0;
    p->iface = iface;
    p->last_ms = 0;
}

/* Essaie de decoder une trame en tete du tampon.
 * Retour : >0 octets consommes (trame valide traitee ou octet rejete), 0 = incomplet */
static uint32_t try_decode(packet_dec_t *p)
{
    uint8_t *b = p->buf;
    uint32_t n = p->n;
    if (n == 0) return 0;
    uint32_t hl, len;
    if (b[0] == 2) {
        if (n < 2) return 0;
        hl = 2;
        len = b[1];
        if (len == 0) return 1;
    } else if (b[0] == 3) {
        if (n < 3) return 0;
        hl = 3;
        len = (uint32_t)b[1] << 8 | b[2];
        if (len < 255 || len > PACKET_MAX_PL) return 1;
    } else {
        return 1;
    }
    uint32_t total = hl + len + 3;
    if (n < total) return 0;
    if (b[total - 1] != 3) return 1;
    uint16_t c = (uint16_t)((uint16_t)b[hl + len] << 8 | b[hl + len + 1]);
    if (crc16(b + hl, len) != c) return 1;
    commands_process(p->iface, b + hl, len);
    return total;
}

void packet_dec_byte(packet_dec_t *p, uint8_t c, uint32_t now_ms)
{
    /* Abandon d'une trame partielle apres 100 ms de silence */
    if (p->n && now_ms - p->last_ms > 100u) p->n = 0;
    p->last_ms = now_ms;
    if (p->n >= sizeof(p->buf)) p->n = 0;
    p->buf[p->n++] = c;
    for (;;) {
        uint32_t k = try_decode(p);
        if (k == 0) break;
        memmove(p->buf, p->buf + k, p->n - k);
        p->n -= k;
    }
}

void packet_send(const comm_iface_t *iface, const uint8_t *pl, uint32_t n)
{
    if (!iface || n == 0 || n > PACKET_MAX_PL) return;
    if (iface->is_can) {
        can_vesc_send_buffer(iface->can_target, pl, n, iface->can_send_mode);
        return;
    }
    if (!iface->write) return;
    uint8_t h[3];
    uint32_t hl;
    if (n <= 255) { h[0] = 2; h[1] = (uint8_t)n; hl = 2; }
    else { h[0] = 3; h[1] = (uint8_t)(n >> 8); h[2] = (uint8_t)n; hl = 3; }
    uint16_t c = crc16(pl, n);
    uint8_t t[3] = { (uint8_t)(c >> 8), (uint8_t)c, 3 };
    iface->write(h, hl);
    iface->write(pl, n);
    iface->write(t, 3);
}
