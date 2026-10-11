/*
 * abs_enc_proto.h — Controles d'integrite des trames des codeurs absolus (testes sur PC).
 */
#ifndef ABS_ENC_PROTO_H
#define ABS_ENC_PROTO_H

#include <stdbool.h>
#include <stdint.h>

/* parite paire sur 16 bits (AS5047, MT6816) : true si correcte */
static inline bool abs_parity_even(uint16_t x)
{
    x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;
    return (~x) & 1u;
}

/* CRC8 polynome 0x1D (AS5x47U : graine 0xC4 puis complement ; TLE5012 : graine 0xFF puis complement) */
static inline uint8_t abs_crc8_1d(const uint8_t *d, int n, uint8_t c)
{
    for (int i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++) c = (uint8_t)((c & 0x80u) ? ((unsigned)c << 1) ^ 0x1Du : (unsigned)c << 1);
    }
    return c;
}

/* AMT22 : bits 15 et 14 = parites impaires des bits impairs et pairs de la position */
static inline bool abs_amt22_ok(uint16_t p)
{
    uint16_t ck = 0x3;
    for (int i = 0; i < 14; i += 2) ck ^= (p >> i) & 0x3u;
    return ck == (p >> 14);
}

/* TLE5012 : mot de securite (bits 14..12 a 1 = pas d'erreur) et CRC de commande + donnee */
static inline bool abs_tle5012_ok(uint16_t cmd, uint16_t v, uint16_t sw)
{
    uint8_t t[4] = { (uint8_t)(cmd >> 8), (uint8_t)cmd, (uint8_t)(v >> 8), (uint8_t)v };
    uint8_t c = (uint8_t)~abs_crc8_1d(t, 4, 0xFF);
    return (sw & 0x4000u) && (sw & 0x2000u) && (sw & 0x1000u) && c == (uint8_t)sw;
}

#endif
