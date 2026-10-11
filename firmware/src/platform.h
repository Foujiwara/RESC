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
#ifndef RESC_GIT_HASH
#define RESC_GIT_HASH       "local"      /* remplace a la compilation par le commit (build.py) */
#endif
#ifdef RESC_CONF_PROFILE_RESC
/* profil RESC : vraie version RESC (RESC Tool lit la description de la configuration) */
#define RESC_FW_MAJOR       1
#define RESC_FW_MINOR       0
#define RESC_VERSION_STR    "RESC 1.0.0 (profil RESC)"
#else
#define RESC_FW_MAJOR       7      /* version annoncee a VESC Tool (format 7.00) */
#define RESC_FW_MINOR       0
#define RESC_VERSION_STR    "RESC 1.0.0 (profil VESC 7.00)"
#endif

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

#endif /* PLATFORM_H */
