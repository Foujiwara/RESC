/*
 * i2c_bb.h — I2C logiciel (maitre, ~400 kHz), broches quelconques en drain ouvert.
 */
#ifndef I2C_BB_H
#define I2C_BB_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t sda, scl;
    bool err;
} i2c_bb_t;

void i2c_bb_init(i2c_bb_t *b, uint8_t sda, uint8_t scl);
/* ecrit tx (registre puis donnees), puis lit rx_n octets si rx ; false si pas d'acquittement */
bool i2c_bb_txrx(i2c_bb_t *b, uint8_t addr, const uint8_t *tx, int tx_n, uint8_t *rx, int rx_n);

#endif
