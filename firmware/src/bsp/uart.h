/* uart.h — UART generique a tampons circulaires (RX/TX par interruption) */
#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct uart_s uart_t;

uart_t *uart_open(int inst, uint8_t tx_pin, uint8_t rx_pin, uint32_t baud);
void uart_set_baud(uart_t *u, uint32_t baud);
int  uart_getc(uart_t *u);                       /* -1 si vide */
uint32_t uart_write(uart_t *u, const uint8_t *d, uint32_t n);
uint32_t uart_tx_free(uart_t *u);

#endif
