/*
 * test_abs_enc.c — Controles d'integrite des codeurs absolus :
 *  - CRC8 de l'AS5x47U sur les commandes de lecture : valeurs precalculees du firmware VESC
 *  - parite paire (AS5047, MT6816) : trame correcte acceptee, un bit change refuse
 *  - somme de controle AMT22 contre la definition (bits impairs / pairs, parite impaire)
 *  - TLE5012 : CRC calcule accepte, mot de securite en erreur ou CRC faux refuses
 */
#include <stdio.h>
#include <stdlib.h>
#include "abs_enc_proto.h"

static int fails;
static void check(const char *w, int ok) { printf("  [%s] %s\n", ok ? " OK " : "ECHEC", w); if (!ok) fails++; }

static uint8_t as5x47u_cmd_crc(uint16_t cmd)
{
    uint8_t b[2] = { (uint8_t)(cmd >> 8), (uint8_t)cmd };
    return (uint8_t)(abs_crc8_1d(b, 2, 0xC4) ^ 0xFFu);
}

int main(void)
{
    printf("== AS5x47U : CRC des commandes (constantes du firmware VESC)\n");
    check("lecture position 0x7FFF -> 0xBD", as5x47u_cmd_crc(0x3FFF | 0x4000) == 0xBD);
    check("lecture ERRFL 0x4001 -> 0x06", as5x47u_cmd_crc(0x0001 | 0x4000) == 0x06);
    check("lecture DIAG 0x7FF5 -> 0x6F", as5x47u_cmd_crc(0x3FF5 | 0x4000) == 0x6F);
    check("lecture MAGN 0x7FFD -> 0x87", as5x47u_cmd_crc(0x3FFD | 0x4000) == 0x87);
    check("lecture AGC 0x7FF9 -> 0xF3", as5x47u_cmd_crc(0x3FF9 | 0x4000) == 0xF3);

    printf("== Parite paire\n");
    int ok = 1;
    srand(1);
    for (int k = 0; k < 2000; k++) {
        uint16_t d = (uint16_t)(rand() & 0x7FFF), n = 0;
        for (int b = 0; b < 15; b++) n += (d >> b) & 1u;
        uint16_t w = (uint16_t)(d | ((n & 1u) << 15));       /* bit 15 = parite paire */
        if (!abs_parity_even(w) || abs_parity_even((uint16_t)(w ^ (1u << (k % 16))))) ok = 0;
    }
    check("2000 trames : correctes acceptees, un bit faux refuse", ok);

    printf("== AMT22 : somme de controle\n");
    ok = 1;
    for (uint32_t pos = 0; pos < 16384; pos += 7) {
        unsigned odd = 0, even = 0;
        for (int b = 0; b < 14; b++) { if (b & 1) odd ^= (pos >> b) & 1u; else even ^= (pos >> b) & 1u; }
        uint16_t k1 = (uint16_t)(!odd), k0 = (uint16_t)(!even);
        uint16_t w = (uint16_t)(pos | (k1 << 15) | (k0 << 14));
        if (!abs_amt22_ok(w) || abs_amt22_ok((uint16_t)(w ^ 0x8000u))) ok = 0;
    }
    check("toutes les positions : somme definie par le constructeur acceptee, bit de controle faux refuse", ok);

    printf("== TLE5012 : mot de securite et CRC\n");
    uint16_t cmd = (uint16_t)((1u << 15) | (0x02u << 4) | 1u), v = 0x1234;
    uint8_t t[4] = { (uint8_t)(cmd >> 8), (uint8_t)cmd, (uint8_t)(v >> 8), (uint8_t)v };
    uint8_t c = (uint8_t)~abs_crc8_1d(t, 4, 0xFF);
    uint16_t sw = (uint16_t)(0x7000u | c);
    check("trame correcte acceptee", abs_tle5012_ok(cmd, v, sw));
    check("erreur systeme (bit 14 a 0) refusee", !abs_tle5012_ok(cmd, v, (uint16_t)(sw & ~0x4000u)));
    check("angle invalide (bit 12 a 0) refuse", !abs_tle5012_ok(cmd, v, (uint16_t)(sw & ~0x1000u)));
    check("donnee alteree refusee (CRC)", !abs_tle5012_ok(cmd, (uint16_t)(v ^ 1u), sw));

    printf("\n%s (%d echec(s))\n", fails ? "ECHEC" : "TOUS LES TESTS PASSENT", fails);
    return fails ? 1 : 0;
}
