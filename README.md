# RESC — Firmware FOC haute performance pour STM32F405RGT6

Firmware de commande moteur PMSM/BLDC **entièrement nouveau**. Il n'est pas dérivé de VESC.
Pour l'interopérabilité avec VESC Tool, les écrans et les contrôleurs existants, RESC reprend de VESC (`release_7_00`) les **protocoles CAN/UART**, le **format et les valeurs par défaut de la configuration**, et des **données de cartes** (brochages, valeurs). Le détail est dans [NOTICE.md](NOTICE.md). Le cœur de commande (FOC, observateurs, contrôle, protections, détections) est un développement indépendant.

> **État : firmware v0.1, premier essai sur carte réussi** (connexion USB et VESC Tool, données temps réel, paramètres, rotation du moteur après détection du flux). Détection R/L corrigée, à revalider sur la carte.
> Carte de test : **Ubox SINGLE 85_200**. Procédure : [docs/12_essai_banc.md](docs/12_essai_banc.md).

## Firmware

| Élément | Où | État |
|---|---|---|
| Cœur FOC RESC (PI d/q découplé, anti-windup, SVPWM, compensation du retard, reconstruction 3 shunts) | `firmware/src/core/foc.c` | simulation OK |
| Observateur de flux actif non linéaire + PLL | `firmware/src/core/observer.c` | simulation OK |
| Hall (interpolation bornée au secteur) et codeur ABI, bascule vers l'observateur | `firmware/src/core/angle_sensors.c` | simulation OK |
| Démarrage I/f avec relais sans à-coup, reprise au vol | `foc.c` | simulation OK |
| Détections manuelles RESC : offsets, R/L, flux, Hall, codeur (pas d'auto-configuration) | `foc.c`, `app/detect_app.c` | simulation OK |
| Applications de contrôle : courant, frein, duty, vitesse, timeout, déclassements | `firmware/src/core/ctrl.c` | simulation OK |
| Protections : surintensité logicielle + watchdog ADC, surtension, sous-tension, températures, défaut driver, break | `foc.c`, `bsp/motor_hw.c` | partiellement simulé |
| STM32F405 : TIM1, ADC triple, UART, CAN, USB CDC, flash, IWDG | `firmware/src/bsp/` | **à valider sur carte** |
| Protocole (USB/UART/CAN), configuration VESC Tool 7.00, terminal | `firmware/src/comm/`, `firmware/src/vesc/` | tests PC OK |

```bash
python firmware/build.py
```

```bash
python firmware/build.py --sim
```

Images prêtes à flasher : `firmware/release/resc_ubox_single_85_200.hex` / `.bin`. Console : `python tools/resc_cli.py`.

## Convention de fiabilité (utilisée dans tous les documents)

| Tag | Signification |
|---|---|
| **[C]** | **Confirmé** : vérifié dans le datasheet, le manuel de référence RM0090 ou le source VESC 7.00 |
| **[E]** | **Estimé** : calcul, ordre de grandeur ou retour d'expérience. Il faudra le mesurer |
| **[V]** | **À valider** : dépend du matériel réel ou d'un essai. Ne pas l'utiliser comme hypothèse de sécurité |

## Documents

| # | Document | Contenu |
|---|---|---|
| 1 | [Audit STM32F405RGT6](docs/01_audit_mcu.md) | Caractéristiques, périphériques, contraintes, multiplexage des broches |
| 2 | [Protocoles VESC 7.00](docs/02_protocole_vesc.md) | CAN, UART, messages, encodages, compatibilité VESC Tool |
| 3 | [Architecture temps réel](docs/03_architecture_temps_reel.md) | Bare-metal, RTOS, hybride ; priorités ; instrumentation |
| 4 | [PWM, ADC, courants](docs/04_pwm_adc_courants.md) | Modulation, échantillonnage, 3 shunts, fréquences |
| 5 | [Commande FOC](docs/05_commande_foc.md) | Régulateurs, découplage, saturation, MTPA, FW, dead-time |
| 6 | [Observateurs sensorless](docs/06_observateurs.md) | Comparaison de 10 méthodes, interface, mode comparatif |
| 7 | [Protection et perte de synchro](docs/07_protection.md) | Surintensité absolue matérielle, état sûr, détection de décrochage |
| 8 | [Informations matérielles manquantes](docs/08_infos_manquantes.md) | Ce qu'il faut fournir avant de coder |
| 9 | [Architecture recommandée](docs/09_architecture_recommandee.md) | Synthèse, arborescence, interfaces, configuration |
| 10 | [Feuille de route](docs/10_feuille_de_route.md) | Étapes, critères de passage, validation |
| 11 | [Cartes supportées](docs/11_cartes_vesc.md) | Configuration matérielle compatible VESC, recensement, carte Ubox de référence |
| 12 | [Premier essai au banc](docs/12_essai_banc.md) | Sauvegarde, flashage, vérifications, détections, premiers tours |

## Configuration matérielle

Une carte est choisie à la compilation, comme le `HW_HEADER` de VESC :

```
-DRESC_BOARD_HEADER='"boards/ubox/board_ubox_single_85_200.h"'
```

- [hw/hw_config.h](hw/hw_config.h) : point d'entrée et contrat (`BOARD_*` = matériel, `HWLIM_*` = limites absolues, `DEF_*` = défauts).
- [hw/boards/common/board_common_f405.h](hw/boards/common/board_common_f405.h) : brochage commun, repris des 172 configurations VESC 7.00.
- [hw/boards/ubox/](hw/boards/ubox/) : carte de référence, relue à la main.
- `hw/boards/imported/` : 150 cartes générées automatiquement (statut IMPORTED, pont interdit tant qu'elles ne sont pas relues).
- Vérification : `python -I tools/hw_import/check_boards.py . --quiet`

Outils d'analyse : [tools/analysis/svpwm_window.py](tools/analysis/svpwm_window.py) (fenêtres de mesure 3 shunts, retards, ondulation) et [tools/analysis/vesc_hw_survey.py](tools/analysis/vesc_hw_survey.py) (recensement des cartes VESC).

## Résumé des choix recommandés

| Domaine | Choix recommandé | Alternative prévue |
|---|---|---|
| Ordonnancement | Hybride : ISR FOC hors noyau + ordonnanceur run-to-completion sur NVIC (sans RTOS tiers) | FreeRTOS pour les tâches non critiques |
| PWM | Centrée, SVPWM min-max, 1 échantillonnage + 1 mise à jour, compensation du retard par prédiction d'angle | Double échantillonnage / double mise à jour pour les moteurs à faible L |
| Mesure du courant | 3 shunts, sélection dynamique des 2 meilleures phases, déclenchement ADC adaptatif, repli par prédiction du modèle | Shunts de phase (variante matérielle) |
| f_PWM par défaut | Propre à chaque carte : 15 kHz sur l'Ubox (= `f_zv/2` de VESC), plage 8–40 kHz, validée selon la fenêtre ADC | 20–25 kHz après mesure de T_win |
| Courant | PI en repère d/q avec découplage + FEM en anticipation, anti-windup par back-calculation, limitation vectorielle | Deadbeat prédictif (option, moteurs à faible L) |
| Sensorless | Observateur de flux actif non linéaire + PLL de type 2 à anticipation | SMO adaptatif, EKF (référence/ombre), HFI basse vitesse (IPM) |
| Protection absolue | Selon la carte (`BOARD_HW_OCP`) : BKIN si câblé, sinon nFAULT → break logiciel, sinon watchdog ADC + break logiciel. **Sur l'Ubox : logicielle uniquement** (doc 07 §7.8) | Modification matérielle : comparateur → PB12/BKIN |
| Chien de garde | IWDG + **enable dynamique** du driver (le signal meurt si l'ISR FOC s'arrête) | Moniteur d'échéance par timer → BKIN |

## Licence

GNU GPL v3 (voir [LICENSE](LICENSE)). Code tiers et elements repris du projet VESC pour la compatibilite : voir [NOTICE.md](NOTICE.md).

> Firmware experimental de commande de puissance, encore en phase d'essai sur banc :
> a utiliser avec une alimentation limitee en courant, sous votre responsabilite.
