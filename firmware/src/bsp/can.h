/* can.h — bxCAN1, trames etendues, files logicielles RX/TX */
#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t id;      /* identifiant 29 bits */
    uint8_t len;
    uint8_t data[8];
} can_frame_t;

bool can_init(uint32_t bitrate);
void can_set_bitrate(uint32_t bitrate);
bool can_send(uint32_t eid, const uint8_t *data, uint8_t len);
bool can_recv(can_frame_t *f);
uint32_t can_errors(void);

#endif
