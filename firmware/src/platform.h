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
#endif

#define RESC_FW_NAME        "RESC"
#define RESC_FW_MAJOR       7      /* version annoncee a VESC Tool (format 7.00) */
#define RESC_FW_MINOR       0
#define RESC_VERSION_STR    "RESC 0.1.0"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

#endif /* PLATFORM_H */
