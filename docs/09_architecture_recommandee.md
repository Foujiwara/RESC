# 9. Architecture recommandée

## 9.1 Synthèse des décisions

| Domaine | Décision | Justification (doc) | Remplaçable par |
|---|---|---|---|
| Ordonnancement | ISR FOC hors noyau + run-to-completion sur NVIC | Déterminisme, 0 surcoût, aucune section critique ne masque la FOC (3) | FreeRTOS en bas niveau |
| Mémoire | Code critique en SRAM1, état et piles en CCM, DMA en SRAM1/2 | Flash à banque unique bloquante, CCM sans DMA (1) | — |
| PWM | Centrée, SVPWM min-max, S1/U1 + prédiction d'angle de 1,5 T_s | Seul instant de mesure low-side commun ; erreur de 36° à 2 kHz sans compensation (4) | S1/U1 asymétrique, DPWM, S2/U2 (shunts de phase) |
| Courants | 2 meilleures phases, déclenchement adaptatif, drapeaux de validité, repli par prédiction | Fenêtre de 6,7 % T_s à m = 1 (4) | `cs_inline` |
| f_PWM | 25 kHz par défaut, validée au démarrage (T_win, f_e, ΔI) | Calculs (4) | Paramètre |
| Régulateur | PI complex vector + découplage + FEM FF, back-calculation | Robustesse, invariance en ω (5) | Deadbeat (option) |
| Observateur | Flux actif non linéaire + PLL type 2 normalisée ; I/f au démarrage | SPM + IPM, sans déphasage, faible L (6) | SMO-ST, EKF, HFI, capteurs |
| Protection | V_DS/DESAT + comparateurs → BKIN (Hi-Z) + EN dynamique + logiciel par tick | Indépendance du CPU, angle mort low-side (7) | — (obligatoire) |
| Comms | VESC CAN/UART derrière `motor_api` ; custom config XML | Interopérabilité sans dépendre du modèle MCCONF (2) | Émulation MCCONF, autres protocoles |

## 9.2 Arborescence du projet

```
RESC/
├── README.md
├── docs/                       études et décisions (ADR)
│   └── adr/                    une décision = un fichier (contexte, options, choix, conséquences)
├── hw/                         ── MATÉRIEL : seul endroit qui connaît les broches et registres (doc 11)
│   ├── hw_config.h             point d'entrée : sélectionne RESC_BOARD_HEADER, contrat documenté
│   ├── hw_types.h / hw_check.h constantes ; vérifications à la compilation
│   ├── boards/
│   │   ├── common/             brochage commun des cartes VESC (défauts)
│   │   ├── ubox/               carte de référence (REVIEWED) : BOARD_* / HWLIM_* / DEF_*
│   │   └── vesc/<fabricant>/   150 cartes générées depuis VESC 7.00 (IMPORTED)
│   ├── bsp/                    init horloges, GPIO, NVIC, MPU, VTOR en RAM
│   ├── drivers/                tim_pwm, adc_sync, dma, bxcan, uart_dma, flash, iwdg, gate_driver, encoder_*
│   └── startup/                vecteurs, linker script (.ramfunc, .ccm, .noinit)
├── core/                       ── CŒUR MOTEUR : aucune dépendance matérielle directe
│   ├── math/                   trig rapide, Clarke/Park, filtres, saturation, interpolation
│   ├── modulation/             svpwm_minmax, dpwm, overmod ; interface modulator_t
│   ├── sensing/                current_sense_t (lowside3, inline), offsets, validité, prédiction
│   ├── control/
│   │   ├── current/            pi_cv (complex vector), deadbeat ; interface current_ctrl_t
│   │   ├── reference/          mtpa, field_weakening, current_limits, derating
│   │   ├── speed_pos/          boucles vitesse/position
│   │   └── inverter_comp/      compensation dead-time / chutes
│   ├── estimation/
│   │   ├── observers/          flux_nl, active_flux, emf_ext, smo_st, ekf, hfi ; observer_vtbl_t
│   │   ├── tracking/           pll2, pll3_mech
│   │   ├── sensors/            encoder, hall, (même interface que les observateurs)
│   │   └── params/             R thermique, MRAS
│   ├── foc_engine.c            orchestration d'un tick FOC (.ramfunc)
│   └── motor_sm.c              machine d'état : INIT/CALIB/READY/STARTING/RUN/STOPPING/FAULT_LATCHED
├── safety/                     ── PROTECTIONS (prioritaires)
│   ├── fault_manager           sources, verrouillage, réarmement, mapping des codes
│   ├── fast_checks             contrôles par tick (.ramfunc)
│   ├── sync_monitor            indicateurs I1..I8, fusion, niveaux de réaction
│   ├── safe_state              politique Hi-Z / ASC
│   └── watchdog                IWDG + enable dynamique + moniteur d'échéance
├── app/
│   ├── motor_api               API stable en unités SI (consignes, mesures, état)
│   ├── command_arbiter         source active, timeout, rampes, bornage
│   └── config/                 schéma des paramètres, validation, stockage A/B CRC, migration de version
├── comm/                       ── COMMUNICATIONS (ne touche jamais au cœur directement)
│   ├── transport/              uart, can (files SPSC)
│   └── vesc/                   packet, crc16, can_proto, commands, values, custom_config_xml
├── diag/                       journal rapide (anneaux), statistiques RT, scope, historique des défauts
├── sim/                        ── SIMULATION (compilée pour PC)
│   ├── plant/                  modèle PMSM d/q avec saturation, onduleur avec dead-time, ADC + bruit + fenêtres
│   ├── harness/                boucle fermée : même code core/ + safety/ que la cible
│   └── replay/                 rejeu de journaux capturés sur la cible
├── tests/
│   ├── unit/                   math, modulation, PI, observateurs, protocole (vecteurs de référence)
│   ├── hil/                    scripts de banc (CAN/UART)
│   └── regression/             scénarios sim + critères chiffrés
└── tools/
    ├── analysis/               scripts de dimensionnement (svpwm_window.py…)
    ├── check_ramfunc.py        vérifie sur le .map que le chemin critique n'appelle rien en flash
    └── vesc_client/            client de test du protocole
```

**Règle de dépendance (vérifiée en CI) :** `core/` et `safety/` n'incluent jamais `hw/drivers/`. Ils reçoivent des structures, ce qui permet de compiler pour le PC (sim/tests). `comm/` ne connaît que `app/motor_api`.

## 9.3 Les trois niveaux de configuration

> Mise en œuvre : les trois niveaux coexistent dans **un fichier par carte**, comme chez VESC, et se distinguent par leur préfixe : `BOARD_*` (matériel), `HWLIM_*` (limites absolues), `DEF_*` (défauts). Voir [hw/hw_config.h](../hw/hw_config.h) et la doc 11. Le pseudo-code ci-dessous reste valable sur le fond.

```c
/* hardware_limits.h : ABSOLU, compilé, jamais modifiable */
#define HWL_I_PHASE_ABS_A        ...   /* dérivé du MOSFET/shunt, cf. doc 7 */
#define HWL_VBUS_ABS_MAX_V       ...
#define HWL_T_FET_ABS_MAX_C      ...
#define HWL_F_PWM_MIN_HZ / MAX_HZ ...
#define HWL_DEADTIME_NS          ...   /* matériel, appliqué au DTG puis verrouillé */
#define HWL_T_WIN_MIN_NS         ...   /* fenêtre ADC mesurée */

/* default_config.h : valeurs initiales des paramètres utilisateur */
/* config (runtime, persistant) : struct versionnée + CRC32, validée à chaque écriture */
```

- Validation : chaque paramètre a `{min, max, défaut, unité, dépendances}` dans un **schéma unique**. Ce schéma génère la validation C, le XML custom config VESC et la documentation.
- Validation croisée : f_PWM vs T_win, ERPM_max vs f_PWM, I_max_user ≤ HWL, V_max_user ≤ HWL, vitesse max vs contrainte UCG.
- Une configuration invalide est **refusée entièrement** (pas de bornage silencieux), avec un code d'erreur.
- Stockage : secteurs flash 1 et 2 (16 Ko) en A/B avec numéro de séquence. Écriture différée jusqu'à l'arrêt moteur.

## 9.4 Tick FOC (ordre d'exécution)

```
ISR JEOC (prio 1, .ramfunc):
  1. lecture ADC (i_abc, v_abc, vbus) + drapeaux de validité         ── sensing
  2. fast_checks (OC logiciel, Vbus, cohérence) → break logiciel si besoin ── safety
  3. Clarke ; sélection mesure/prédiction                            ── sensing
  4. observer_active.update ; observers_shadow.update (budget)       ── estimation
  5. Park (θ à l'instant d'échantillonnage)
  6. lecture de la boîte à lettres des références (id*, iq* ou mode)
  7. régulateur de courant + découplage + FF + anti-windup            ── control
  8. limitation tension ; Park⁻¹ avec θ + ω·1,5·T_s ; compensation onduleur
  9. modulateur → CCR1..3 ; calcul de CCR4 (déclenchement ADC suivant) ── modulation
 10. sync_monitor (indicateurs rapides) ; journal rapide ; télémétrie double tampon
 11. basculement de l'enable dynamique ; rechargement du moniteur d'échéance ; stats DWT
 12. tous les N ticks : pend SWI contrôle
```

## 9.5 Diagnostics (indépendants des communications)

- **Anneau rapide** (CCM) : 1 enregistrement par tick, 32 à 48 octets (i_ab, i_dq*, i_dq, v_dq, θ, ω, résidus, drapeaux, cycles). Profondeur de 256 ticks → instantané pré-défaut **figé** par l'ISR break.
- **Capture déclenchée** (« scope ») : canaux sélectionnables, décimation, pré/post-déclenchement, lue ensuite par COMM_PLOT_* ou par terminal.
- **Historique des défauts** : 32 entrées (code, source matérielle, timestamp, instantané réduit), en `.noinit` puis en flash à l'arrêt.
- **Comparaison des observateurs** : sorties des observateurs d'ombre + référence capteur dans l'anneau. Statistiques d'erreur en ligne (moyenne, RMS, max par plage de vitesse).
- Tous les écrivains sont mono-producteur ; les lecteurs (comm) ne bloquent jamais le producteur. Si le buffer est plein, l'écriture **écrase** (pour la trace) ou **compte les pertes** (pour les événements).
