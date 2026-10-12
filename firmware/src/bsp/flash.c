/*
 * flash.c — Effacement / programmation de la flash et sauts au bootloader.
 * Pendant un effacement, toute lecture de la flash bloque le CPU : l'appelant
 * doit avoir coupe le pont (le chemin FOC est en RAM, mais les autres
 * interruptions et la boucle principale sont figees). Le watchdog est
 * ralenti pendant l'operation.
 */
#include "flash.h"
#include "stm32.h"
#include "wdg.h"
#include "resc_conf.h"

static void unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_SOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR;
}

static void lock(void) { FLASH->CR |= FLASH_CR_LOCK; }

static bool wait_done(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { }
    return !(FLASH->SR & (FLASH_SR_SOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR));
}

bool flash_erase_sector(uint32_t sector)
{
    if (sector == 0 || sector >= 11) return false;      /* vecteurs et bootloader proteges */
    wdg_slow(true);
    unlock();
    FLASH->CR = (FLASH->CR & ~(FLASH_CR_PSIZE | FLASH_CR_SNB)) | FLASH_CR_PSIZE_1 | (sector << FLASH_CR_SNB_Pos) | FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    bool ok = wait_done();
    FLASH->CR &= ~FLASH_CR_SER;
    lock();
    wdg_slow(false);
    return ok;
}

bool flash_program(uint32_t addr, const uint8_t *d, uint32_t n)
{
    if (addr < 0x08004000u || addr + n > FLASH_BOOTLOADER) return false;
    unlock();
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE);                /* acces octet */
    bool ok = true;
    for (uint32_t i = 0; i < n && ok; i++) {
        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint8_t *)(addr + i) = d[i];
        ok = wait_done();
        FLASH->CR &= ~FLASH_CR_PG;
    }
    lock();
    return ok;
}

/* -------------------------------------------- configuration (secteurs 1, 2) */
#define CONF_SECTOR(slot)  (1u + (slot))
#define CONF_ADDR(slot)    (0x08004000u + (slot) * 0x4000u)

bool plat_conf_write(uint32_t slot, const uint8_t *data, uint32_t len)
{
    if (slot > 1 || len > 0x4000u) return false;
    if (!flash_erase_sector(CONF_SECTOR(slot))) return false;
    return flash_program(CONF_ADDR(slot), data, len);
}

const uint8_t *plat_conf_read(uint32_t slot, uint32_t *len)
{
    if (slot > 1) return 0;
    *len = 0x4000u;
    return (const uint8_t *)CONF_ADDR(slot);
}

/* -------------------------------------------- nouvelle application (8-10) */
static uint32_t s_newapp_end;      /* fin de l'image recue (decalage dans le tampon) */

bool flash_erase_new_app(uint32_t size)
{
    s_newapp_end = 0;
    for (uint32_t s = 0; s < 3; s++) {
        if (size > s * 128u * 1024u) {
            if (!flash_erase_sector(8 + s)) return false;
        }
    }
    return true;
}

/* Mise a jour du bootloader (VESC Tool) : apres effacement explicite du secteur 11, les ecritures
 * au-dela du tampon d'application (decalage >= 384 Kio) vont dans le secteur du bootloader. */
static bool s_bl_unlocked;

bool flash_erase_bootloader(void)
{
    s_bl_unlocked = flash_erase_sector(11);
    return s_bl_unlocked;
}

bool flash_write_new_app(uint32_t off, const uint8_t *d, uint32_t n)
{
    uint32_t lim = s_bl_unlocked ? FLASH_NEWAPP_SIZE + 128u * 1024u : FLASH_NEWAPP_SIZE;
    if (off + n > lim) return false;
    if (off + n > s_newapp_end) s_newapp_end = off + n;
    return flash_program(FLASH_NEWAPP_BASE + off, d, n);
}

bool flash_upd_backup_free(void)
{
    if (s_newapp_end > FLASH_UPD_BACKUP - FLASH_NEWAPP_BASE) return false;
    const uint32_t *p = (const uint32_t *)FLASH_UPD_BACKUP;
    for (uint32_t k = 0; k < 0x2000u / 4u; k++) if (p[k] != 0xFFFFFFFFu) return false;
    return true;
}

bool flash_upd_backup_write(uint32_t off, const uint8_t *d, uint32_t n)
{
    if (off + n > 0x2000u) return false;
    return flash_program(FLASH_UPD_BACKUP + off, d, n);
}

bool flash_upd_backup_erase(void) { return flash_erase_sector(10); }
const uint8_t *flash_upd_backup_ptr(void) { return (const uint8_t *)FLASH_UPD_BACKUP; }

bool flash_bootloader_present(void)
{
    uint32_t sp = *(volatile uint32_t *)FLASH_BOOTLOADER;
    uint32_t pc = *(volatile uint32_t *)(FLASH_BOOTLOADER + 4);
    return (sp & 0xFFF00000u) == 0x20000000u && pc >= FLASH_BOOTLOADER && pc < 0x08100000u;
}

void flash_jump_to_bootloader(void)
{
    if (!flash_bootloader_present()) return;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    wdg_slow(true);
    __disable_irq();
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }
    SysTick->CTRL = 0;
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;
    uint32_t sp = *(volatile uint32_t *)FLASH_BOOTLOADER;
    uint32_t pc = *(volatile uint32_t *)(FLASH_BOOTLOADER + 4);
    SCB->VTOR = FLASH_BOOTLOADER;
    __set_MSP(sp);
    __enable_irq();
    ((void (*)(void))pc)();
}
