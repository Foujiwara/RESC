/*
 * i2c_bb.c — I2C logiciel. Les deux lignes sont en drain ouvert avec tirage interne (les
 * cartes ont en general leurs propres resistances) ; l'esclave peut etirer l'horloge.
 */
#include "i2c_bb.h"
#include "stm32.h"
#include "gpio.h"
#include "hw_config.h"

static void dly(void) { for (volatile int i = 0; i < 25; i++) __NOP(); }     /* ~1.25 us : ~400 kHz */

static void sda(i2c_bb_t *b, bool v) { gpio_write(b->sda, v); }
static void scl(i2c_bb_t *b, bool v)
{
    gpio_write(b->scl, v);
    if (v) {
        /* etirement d'horloge : attente que la ligne remonte (borne) */
        for (int k = 0; k < 2000 && !gpio_read(b->scl); k++) dly();
    }
}

void i2c_bb_init(i2c_bb_t *b, uint8_t sda_pin, uint8_t scl_pin)
{
    b->sda = sda_pin;
    b->scl = scl_pin;
    b->err = false;
    gpio_write(sda_pin, true);
    gpio_write(scl_pin, true);
    gpio_cfg(sda_pin, GPIO_MODE_OUT, true, 2, GPIO_PUPD_UP, 0);
    gpio_cfg(scl_pin, GPIO_MODE_OUT, true, 2, GPIO_PUPD_UP, 0);
    /* liberation du bus : 9 impulsions d'horloge si un esclave tient SDA */
    for (int k = 0; k < 9 && !gpio_read(sda_pin); k++) { scl(b, false); dly(); scl(b, true); dly(); }
}

static void start(i2c_bb_t *b) { sda(b, true); scl(b, true); dly(); sda(b, false); dly(); scl(b, false); dly(); }
static void stop(i2c_bb_t *b) { sda(b, false); dly(); scl(b, true); dly(); sda(b, true); dly(); }

static bool wbyte(i2c_bb_t *b, uint8_t v)
{
    for (int i = 0; i < 8; i++) {
        sda(b, (v & 0x80u) != 0);
        v <<= 1;
        dly();
        scl(b, true);
        dly();
        scl(b, false);
    }
    sda(b, true);
    dly();
    scl(b, true);
    dly();
    bool ack = !gpio_read(b->sda);
    scl(b, false);
    return ack;
}

static uint8_t rbyte(i2c_bb_t *b, bool ack)
{
    uint8_t v = 0;
    sda(b, true);
    for (int i = 0; i < 8; i++) {
        dly();
        scl(b, true);
        dly();
        v = (uint8_t)(v << 1) | (gpio_read(b->sda) ? 1u : 0u);
        scl(b, false);
    }
    sda(b, !ack);
    dly();
    scl(b, true);
    dly();
    scl(b, false);
    sda(b, true);
    return v;
}

bool i2c_bb_txrx(i2c_bb_t *b, uint8_t addr, const uint8_t *tx, int tx_n, uint8_t *rx, int rx_n)
{
    bool ok = true;
    if (tx_n > 0) {
        start(b);
        ok = wbyte(b, (uint8_t)(addr << 1));
        for (int i = 0; ok && i < tx_n; i++) ok = wbyte(b, tx[i]);
    }
    if (ok && rx && rx_n > 0) {
        start(b);                                   /* depart repete */
        ok = wbyte(b, (uint8_t)(addr << 1 | 1u));
        for (int i = 0; ok && i < rx_n; i++) rx[i] = rbyte(b, i + 1 < rx_n);
    }
    stop(b);
    b->err = !ok;
    return ok;
}
