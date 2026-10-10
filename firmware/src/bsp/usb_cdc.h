/* usb_cdc.h — Port serie virtuel USB (OTG FS, CDC ACM), VID/PID 0483:5740 */
#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdbool.h>
#include <stdint.h>

void usb_cdc_init(void);
int  usb_cdc_getc(void);                         /* -1 si vide */
uint32_t usb_cdc_write(const uint8_t *d, uint32_t n);
bool usb_cdc_configured(void);
void usb_cdc_poll(void);                         /* relance l'emission (boucle principale) */

#endif
