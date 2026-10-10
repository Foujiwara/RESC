#ifndef COMMANDS_H
#define COMMANDS_H
#include <stdint.h>
#include "packet.h"

void commands_init(void);
void commands_process(const comm_iface_t *src, uint8_t *data, uint32_t len);
void commands_poll(uint32_t now_ms);                 /* reponses differees (detection) */
/* Texte vers la console de VESC Tool (COMM_PRINT) sur l'interface du terminal */
void commands_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
/* Demandes executees par la boucle principale */
extern volatile bool g_req_reboot, g_req_bootloader;

#endif
