/*
 * platform.h — Placement memoire et petites abstractions communes cible / PC.
 *
 * Sur cible : le chemin critique FOC s'execute depuis la SRAM (section
 * .ramfunc, aucune attente flash, insensible aux effacements flash) et son
 * etat vit en CCM (acces 0 wait-state, bus D separe).
 * Sur PC (simulation, tests) : macros vides.
 */
#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdbool.h>

#if defined(RESC_HOST)
#define RAMFUNC
#define CCMBSS
#define NOINIT
#else
#define RAMFUNC  __attribute__((section(".ramfunc"), noinline, used))
#define CCMBSS   __attribute__((section(".ccmbss")))
#define NOINIT   __attribute__((section(".noinit")))
#define CCMKEEP  __attribute__((section(".ccmkeep")))
#endif

#define RESC_FW_NAME        "RESC"
#ifndef RESC_GIT_HASH
#define RESC_GIT_HASH       "local"      /* remplace a la compilation par le commit (build.py) */
#endif
#define RESC_FW_MAJOR       1
#define RESC_FW_MINOR       8      /* incremente a chaque firmware livre */
#define RESC_VERSION_STR    "RESC 1.8.0"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* Priorite NVIC de la couche de controle (SysTick, 1 kHz). Les applications qui la commandent
 * tournent dans ce contexte ; les paquets (outil, CAN) dans la boucle principale. */
#define PLAT_CTRL_PRIO      4

/* Section critique vis-a-vis de la couche de controle : masque SysTick et les interruptions moins
 * prioritaires (BASEPRI), jamais l'ISR FOC ni les protections. Imbricable (rend l'ancien masque). */
#if defined(RESC_HOST)
static inline uint32_t plat_ctrl_lock(void) { return 0; }
static inline void plat_ctrl_unlock(uint32_t s) { (void)s; }
#else
static inline uint32_t plat_ctrl_lock(void)
{
    uint32_t old, v = (uint32_t)PLAT_CTRL_PRIO << 4;
    __asm volatile("mrs %0, basepri" : "=r"(old) :: "memory");
    if (old == 0 || old > v) __asm volatile("msr basepri, %0" :: "r"(v) : "memory");
    return old;
}
static inline void plat_ctrl_unlock(uint32_t s) { __asm volatile("msr basepri, %0" :: "r"(s) : "memory"); }
#endif

#endif /* PLATFORM_H */
