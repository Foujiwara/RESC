/*
 * packet.h — Trame paquet du protocole (UART / USB / BLE) :
 *   [2][len8] ou [3][len16] , payload , [crc16 hi][crc16 lo] , [3]
 * CRC-16/XMODEM (poly 0x1021, init 0) sur le payload.
 */
#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <stdbool.h>

#define PACKET_MAX_PL 1024u

typedef struct comm_iface_s comm_iface_t;
struct comm_iface_s {
    uint32_t (*write)(const uint8_t *d, uint32_t n);   /* NULL pour une interface CAN */
    uint8_t can_target;                                /* reponse via CAN vers ce controleur */
    uint8_t can_send_mode;
    bool is_can;
    uint32_t (*tx_free)(void);                         /* place libre en emission (NULL : inconnue) */
};

typedef struct {
    uint8_t buf[PACKET_MAX_PL + 8];
    uint32_t n;
    uint32_t last_ms;
    const comm_iface_t *iface;
} packet_dec_t;

uint16_t crc16(const uint8_t *d, uint32_t n);
void packet_dec_init(packet_dec_t *p, const comm_iface_t *iface);
/* Ajoute un octet ; appelle commands_process() pour chaque trame valide. */
void packet_dec_byte(packet_dec_t *p, uint8_t c, uint32_t now_ms);
/* Envoie un payload sur une interface (encadrement ou transport CAN). */
void packet_send(const comm_iface_t *iface, const uint8_t *pl, uint32_t n);

#endif
