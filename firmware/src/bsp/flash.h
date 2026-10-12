/*
 * flash.h — Flash interne. Disposition compatible avec le bootloader de
 * reference : secteur 0 vecteurs, 1-2 configuration, 3-7 application,
 * 8-10 tampon « nouvelle application », 11 bootloader (efface seulement sur demande de l'outil,
 * COMM_ERASE_BOOTLOADER, pour en ecrire un nouveau a la suite du tampon).
 */
#ifndef FLASH_H
#define FLASH_H

#include <stdbool.h>
#include <stdint.h>

#define FLASH_NEWAPP_BASE   0x08080000u
#define FLASH_NEWAPP_SIZE   (3u * 128u * 1024u)
#define FLASH_BOOTLOADER    0x080E0000u

bool flash_erase_sector(uint32_t sector);
bool flash_program(uint32_t addr, const uint8_t *data, uint32_t len);
bool flash_erase_new_app(uint32_t size);
bool flash_write_new_app(uint32_t offset, const uint8_t *data, uint32_t len);
bool flash_bootloader_present(void);

/* Copie de la configuration pendant une mise a jour : le bootloader efface les secteurs de
 * configuration. Derniers 8 Kio du tampon (secteur 10), au-dela de l'image recue. */
#define FLASH_UPD_BACKUP    (FLASH_NEWAPP_BASE + FLASH_NEWAPP_SIZE - 0x2000u)
bool flash_upd_backup_free(void);                 /* image recue hors de la zone, zone vierge */
bool flash_upd_backup_write(uint32_t off, const uint8_t *d, uint32_t n);
bool flash_upd_backup_erase(void);
const uint8_t *flash_upd_backup_ptr(void);       /* contenu de la zone (0 si indisponible) */
bool flash_erase_bootloader(void);
void flash_jump_to_bootloader(void);

#endif
