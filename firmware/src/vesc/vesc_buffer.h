/*
 * vesc_buffer.h — Encodages binaires du protocole VESC (big-endian).
 *  - f16(v, s)  = int16(v * s)       - f32(v, s) = int32(v * s)
 *  - f32_auto   = representation IEEE-754 du float, sous-normaux ramenes a 0
 *    (equivalent exact du codage frexp/ldexp de la reference).
 */
#ifndef VESC_BUFFER_H
#define VESC_BUFFER_H

#include <stdint.h>
#include <string.h>

static inline void vb_put_u8(uint8_t *b, uint8_t v, int32_t *i) { b[(*i)++] = v; }
static inline void vb_put_u16(uint8_t *b, uint16_t v, int32_t *i) { b[(*i)++] = (uint8_t)(v >> 8); b[(*i)++] = (uint8_t)v; }
static inline void vb_put_i16(uint8_t *b, int16_t v, int32_t *i) { vb_put_u16(b, (uint16_t)v, i); }
static inline void vb_put_u32(uint8_t *b, uint32_t v, int32_t *i)
{
    b[(*i)++] = (uint8_t)(v >> 24); b[(*i)++] = (uint8_t)(v >> 16);
    b[(*i)++] = (uint8_t)(v >> 8);  b[(*i)++] = (uint8_t)v;
}
static inline void vb_put_i32(uint8_t *b, int32_t v, int32_t *i) { vb_put_u32(b, (uint32_t)v, i); }

static inline int32_t vb_sat_i32(float x)
{
    if (x >= 2147483520.0f) return INT32_MAX;
    if (x <= -2147483520.0f) return INT32_MIN;
    return (int32_t)x;
}
static inline int16_t vb_sat_i16(float x)
{
    if (x >= 32767.0f) return 32767;
    if (x <= -32768.0f) return -32768;
    return (int16_t)x;
}
static inline void vb_put_f16(uint8_t *b, float v, float s, int32_t *i) { vb_put_i16(b, vb_sat_i16(v * s), i); }
static inline void vb_put_f32(uint8_t *b, float v, float s, int32_t *i) { vb_put_i32(b, vb_sat_i32(v * s), i); }
static inline void vb_put_f32_auto(uint8_t *b, float v, int32_t *i)
{
    uint32_t u;
    memcpy(&u, &v, 4);
    if (((u >> 23) & 0xFF) == 0) u = 0;   /* zero et sous-normaux -> 0 (comme la reference) */
    vb_put_u32(b, u, i);
}

static inline uint8_t vb_get_u8(const uint8_t *b, int32_t *i) { return b[(*i)++]; }
static inline uint16_t vb_get_u16(const uint8_t *b, int32_t *i)
{
    uint16_t v = (uint16_t)((uint16_t)b[*i] << 8 | b[*i + 1]);
    *i += 2;
    return v;
}
static inline int16_t vb_get_i16(const uint8_t *b, int32_t *i) { return (int16_t)vb_get_u16(b, i); }
static inline uint32_t vb_get_u32(const uint8_t *b, int32_t *i)
{
    uint32_t v = (uint32_t)b[*i] << 24 | (uint32_t)b[*i + 1] << 16 | (uint32_t)b[*i + 2] << 8 | b[*i + 3];
    *i += 4;
    return v;
}
static inline int32_t vb_get_i32(const uint8_t *b, int32_t *i) { return (int32_t)vb_get_u32(b, i); }
static inline float vb_get_f16(const uint8_t *b, float s, int32_t *i) { return (float)vb_get_i16(b, i) / s; }
static inline float vb_get_f32(const uint8_t *b, float s, int32_t *i) { return (float)vb_get_i32(b, i) / s; }
static inline float vb_get_f32_auto(const uint8_t *b, int32_t *i)
{
    uint32_t u = vb_get_u32(b, i);
    if (((u >> 23) & 0xFF) == 0) return 0.0f;
    float v;
    memcpy(&v, &u, 4);
    return v;
}

#endif /* VESC_BUFFER_H */
