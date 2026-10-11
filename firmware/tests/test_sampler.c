/*
 * test_sampler.c — Echantillonnage pour VESC Tool (COMM_SAMPLE_PRINT) :
 *  - mode immediat : nombre de paquets, format (index, 8 + 1 flottants, etat, angle, index 32 bits)
 *  - conversion des courants (offset, echelle) et des tensions (point milieu)
 *  - decimation, declenchement sur defaut avec historique, renvoi d'un echantillon
 *  - envoi au rythme de l'interface (place libre)
 */
#include <stdio.h>
#include <string.h>
#include "sampler.h"
#include "vesc_buffer.h"
#include "vesc_defaults.h"

foc_telem_t g_foc_telem;
void can_vesc_send_buffer(uint8_t t, const uint8_t *d, uint32_t n, uint8_t m) { (void)t; (void)d; (void)n; (void)m; }
void commands_process(const comm_iface_t *s, uint8_t *d, uint32_t n) { (void)s; (void)d; (void)n; }

static int fails;
static void check(const char *w, int ok) { printf("  [%s] %s\n", ok ? " OK " : "ECHEC", w); if (!ok) fails++; }

/* interface de test : decode les trames (2, len, payload, crc16, 3) */
static uint8_t s_raw[200000];
static uint32_t s_rawn, s_free = 1000000;
static uint32_t w(const uint8_t *d, uint32_t n) { memcpy(s_raw + s_rawn, d, n); s_rawn += n; return n; }
static uint32_t fr(void) { return s_free; }
static const comm_iface_t IFT = { w, 0, 0, false, fr };

static int packets(uint8_t pl[][64], int max)
{
    int np = 0;
    uint32_t k = 0;
    while (k + 2 < s_rawn && np < max) {
        uint32_t len = s_raw[k + 1];
        memcpy(pl[np], s_raw + k + 2, len < 64 ? len : 64);
        np++;
        k += 2 + len + 3;
    }
    return np;
}

static void req(uint8_t mode, uint16_t len, uint8_t dec, uint8_t raw)
{
    uint8_t d[5];
    int32_t i = 0;
    d[i++] = mode;
    vb_put_u16(d, len, &i);
    d[i++] = dec;
    d[i++] = raw;
    sampler_request(&IFT, d, 5);
}

static void isr_n(int n, uint16_t *cnt)
{
    for (int k = 0; k < n; k++) {
        foc_adc_t in;
        memset(&in, 0, sizeof(in));
        in.cs[0] = (uint16_t)(2048 + 100); in.cs[1] = 2048; in.cs[2] = (uint16_t)(2048 - 100);
        in.vph[0] = 1000; in.vph[1] = 2000; in.vph[2] = 3000;
        in.hall = 5;
        g_foc_telem.theta = (float)(*cnt) * 0.001f;
        (*cnt)++;
        sampler_isr(&in);
    }
}

int main(void)
{
    static uint8_t pl[1200][64];
    uint16_t cnt = 0;
    for (int x = 0; x < 3; x++) g_foc_telem.off_cs[x] = 2048.0f;
    sampler_set_scale(0.05f, 0.03f, 0.02f, 15000.0f);

    printf("== Echantillonnage immediat (10 echantillons)\n");
    req(1, 10, 1, 0);
    isr_n(20, &cnt);
    for (int k = 0; k < 20; k++) sampler_poll();
    int np = packets(pl, 1200);
    check("10 paquets envoyes", np == 10);
    {
        int32_t i = 1;
        int16_t idx = vb_get_i16(pl[3], &i);
        float i0 = vb_get_f32_auto(pl[3], &i), i1 = vb_get_f32_auto(pl[3], &i), i2 = vb_get_f32_auto(pl[3], &i);
        float v1 = vb_get_f32_auto(pl[3], &i), v2 = vb_get_f32_auto(pl[3], &i), v3 = vb_get_f32_auto(pl[3], &i);
        float vz = vb_get_f32_auto(pl[3], &i);
        (void)vb_get_f32_auto(pl[3], &i);
        float fsw = vb_get_f32_auto(pl[3], &i);
        uint8_t st = pl[3][i++];
        i++;
        int32_t idx2 = vb_get_i32(pl[3], &i);
        check("en-tete COMM_SAMPLE_PRINT, index 3", pl[3][0] == COMM_SAMPLE_PRINT && idx == 3 && idx2 == 3);
        check("courants : (brut - offset) x echelle", i0 > 4.99f && i0 < 5.01f && i1 > -0.01f && i1 < 0.01f && i2 < -4.99f);
        check("tensions rapportees au point milieu, point milieu = moyenne",
              v1 < -19.9f && v1 > -20.1f && v2 > -0.1f && v2 < 0.1f && v3 > 19.9f && vz > 39.9f && vz < 40.1f);
        check("frequence d'echantillonnage 15 kHz, etat Hall << 3", fsw > 14999.0f && fsw < 15001.0f && st == (5 << 3));
        check("taille du paquet (45 octets)", i == 45);
    }

    printf("== Decimation 4\n");
    s_rawn = 0;
    req(1, 5, 4, 0);
    isr_n(40, &cnt);
    for (int k = 0; k < 20; k++) sampler_poll();
    np = packets(pl, 1200);
    {
        int32_t i = 1 + 2 + 8 * 4;
        (void)i;
        int32_t j = 1 + 2;
        for (int q = 0; q < 8; q++) (void)vb_get_f32_auto(pl[0], &j);
        float fsw = vb_get_f32_auto(pl[0], &j);
        check("5 paquets, frequence 15 kHz / 4", np == 5 && fsw > 3749.0f && fsw < 3751.0f);
    }

    printf("== Declenchement sur defaut, 100 echantillons dont l'historique\n");
    s_rawn = 0;
    g_foc_telem.fault = 0;
    req(4, 100, 1, 0);
    isr_n(300, &cnt);
    check("rien avant le defaut", s_rawn == 0);
    g_foc_telem.fault = 4;
    isr_n(SAMPLER_MAX, &cnt);
    for (int k = 0; k < 400; k++) sampler_poll();
    np = packets(pl, 1200);
    check("tout le tampon envoye (1000 echantillons)", np == SAMPLER_MAX);
    g_foc_telem.fault = 0;

    printf("== Renvoi d'un echantillon et rythme de l'interface\n");
    s_rawn = 0;
    req(8, 42, 1, 0);
    sampler_poll();
    np = packets(pl, 1200);
    {
        int32_t i = 1;
        check("un paquet, index 42", np == 1 && vb_get_i16(pl[0], &i) == 42);
    }
    s_rawn = 0;
    s_free = 10;
    req(1, 10, 1, 0);
    isr_n(20, &cnt);
    sampler_poll();
    check("interface pleine : rien n'est envoye", s_rawn == 0);
    s_free = 1000000;
    for (int k = 0; k < 5; k++) sampler_poll();
    check("place libre : envoi repris", packets(pl, 1200) == 10);

    printf("\n%s (%d echec(s))\n", fails ? "ECHEC" : "TOUS LES TESTS PASSENT", fails);
    return fails ? 1 : 0;
}
