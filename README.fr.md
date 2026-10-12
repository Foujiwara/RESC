# RESC — Firmware FOC haute performance pour STM32F405RGT6

[English](README.md) · **Français**

Firmware de commande moteur **entièrement nouveau**, pour moteurs à aimants permanents (PMSM / BLDC / IPM) et à induction (ACIM). Il n'est pas dérivé de VESC.
Pour l'interopérabilité avec VESC Tool, les écrans et les contrôleurs existants, RESC reprend de VESC (`release_7_00`) les **protocoles CAN/UART**, le **format et les valeurs par défaut de la configuration**, les **données de cartes** (brochages, valeurs) et, en options sélectionnables, les **équations des observateurs VESC** (réimplémentés sous leurs noms d'origine). Le détail est dans [NOTICE.md](NOTICE.md). Le cœur de commande (FOC, observateurs RESC, contrôle, protections, détections) est un développement indépendant.

> **État : firmware 1.0, validé en simulation et par tests PC, pas encore essayé sur la carte.** La v0.1 a été essayée sur carte (USB, VESC Tool, données temps réel, rotation après détection du flux).
> Carte de test : **Ubox SINGLE 85_200**. Procédure : [docs/12_essai_banc.md](docs/12_essai_banc.md). Fonctions et réglages : [docs/14_fonctions_v1.md](docs/14_fonctions_v1.md).

## Firmware

| Élément | Où | État |
|---|---|---|
| Cœur FOC RESC (PI d/q, anticipation configurable, anti-windup, SVPWM, compensation du retard, reconstruction 3 shunts) | `firmware/src/core/foc.c` | simulation OK |
| 14 observateurs : les 7 de VESC 7.00 (ORTEGA_ORIGINAL, MXLEMMING, MXV et leurs variantes LAMBDA_COMP) et 7 observateurs RESC (flux actif, rappel radial, FEM de Luenberger, hybride, λ adaptatif) | `firmware/src/core/observer.c` | simulation OK |
| Capteurs : Hall (mélange vers l'observateur, vote majoritaire), codeur ABI, HFI, VSS | `core/foc.c`, `core/angle_sensors.c` | simulation OK |
| Démarrage I/f amorti, options sensorless VESC, compensation saturation et température, MTPA, défluxage, filtres de phase | `foc.c` | simulation OK |
| Moteur à induction : flux rotorique orienté, codeur ou sans capteur, identification Lm/Lr/Rr | `foc.c`, `app/detect_app.c` | simulation OK |
| Détections manuelles : offsets courant et tension, R/L (3 paliers), flux, Hall, codeur, induction | `foc.c`, `app/detect_app.c` | simulation OK |
| Contrôle : courant, frein, duty (mode tension, frein de descente), vitesse, position, boucle ouverte, limitations | `firmware/src/core/ctrl.c` | simulation OK |
| Applications : PPM, ADC (+ boutons, marche arrière intelligente et commande hybride courant / rapport cyclique de RESC), VESC Remote, PAS, multi-contrôleurs CAN avec antipatinage, arrêt d'urgence | `firmware/src/app/apps.c`, `bsp/inputs.c` | tests PC OK |
| Configuration : profils VESC700 / RESC générés depuis les XML, stockage par nom avec migration, description envoyée à RESC Tool (commande 250) | `firmware/conf/`, `tools/resc_conf/`, `src/vesc/`, `src/comm/conf_desc.c` | tests PC OK |
| Fonctions de VESC 7.00 : échantillonnage V0/V7, surmodulation, frein tenu jusqu'à l'arrêt avec court-circuit par les low-sides, frein à main, échantillonnage pour l'outil, statistiques, odomètre, codeurs absolus (tous les modes du port capteur de VESC sauf Custom : AS5047, MT6816, AS5x47U, TLE5012, AMT22, MA782, BiSS-C, TS5700N8501, AD2S1205, sin/cos, PWM, PWM + ABI), IMU (BMI160), BMS VESC par CAN, assistant (détection automatique, CAN compris), mise à jour du firmware sur le bus CAN | `core/`, `bsp/abs_enc.c`, `app/imu.c`, `app/bms.c`, `app/detect_all.c`, `comm/sampler.c` | simulation et tests PC OK |
| Protections : surintensité logicielle + watchdog ADC, surtension, sous-tension, températures, défaut driver, break, survitesse | `foc.c`, `ctrl.c`, `bsp/motor_hw.c` | partiellement simulé |
| STM32F405 : TIM1, ADC triple, TIM4 (servo), UART, CAN, USB CDC, flash, IWDG | `firmware/src/bsp/` | **à valider sur carte** |

## Profils de configuration

| | Profil VESC700 (défaut) | Profil RESC |
|---|---|---|
| Outil | VESC Tool officiel, Android, et RESC Tool | **RESC Tool uniquement** (version qui lit la description) |
| Paramètres | Les 202 + 121 de VESC 7.00, octets identiques | 179 + 111 : sans BLDC, DC, DRV8301, UAVCAN, NRF24, limite lente du courant absolu ; seuls les choix pris en charge par RESC et par la carte restent dans les listes |
| Dans RESC Tool | Paramètres inutilisés cachés | **Seuls les paramètres utilisés par les réglages en cours sont affichés** (voir ci-dessous) |

**Motor Type** (Motor > General > General) : **FOC Permanent Magnets (PMSM / BLDC / IPM)** ou **FOC Induction (ACIM)**.

En profil RESC, le firmware construit les onglets envoyés à RESC Tool d'après la configuration en cours : type de moteur, source d'angle (sans capteur, codeur, Hall, HFI, VSS), options (sonde de température, défluxage, filtre de phase...) et types de commande des applications. Un paramètre n'apparaît que s'il est utilisé ; les onglets et groupes vides disparaissent. Les valeurs cachées restent transmises et conservées. Après un changement de type de moteur ou de capteur, écrire la configuration puis se reconnecter pour mettre la liste à jour. Les règles d'affichage sont dans [tools/resc_conf/desc_vis.py](tools/resc_conf/desc_vis.py).

**Affichage dynamique (commande 250, bit 7) :** un RESC Tool qui envoie `0x80 | conf` reçoit tous les onglets, chacun avec sa condition `visibleIf`, plus une section `<Pages>`. Il affiche et cache alors les paramètres pendant l'édition, sans écrire ni se reconnecter. Sans le bit, la réponse ne change pas. Exemples de flux : [tools/resc_conf/samples/](tools/resc_conf/samples/).

## Compiler et flasher

```bash
python firmware/build.py
```

```bash
python firmware/build.py --conf resc
```

```bash
python firmware/build.py --sim
```

```bash
python firmware/build.py --tests
```

Images prêtes à flasher dans `firmware/release/` :
- `resc_ubox_single_85_200.bin` / `.hex` : profil VESC700 (VESC Tool et RESC Tool).
- `resc_ubox_single_85_200_profil_resc.bin` : profil RESC (uniquement avec un RESC Tool qui lit la description ; à ne pas distribuer avant cette version de l'outil).

Console : `python tools/resc_cli.py`. Le terminal (`help`) liste toutes les commandes, dont les détections (`detect_rl`, `detect_acim`...) et `get` / `set` / `list` pour tous les paramètres.

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
| 13 | [Audit du code et indépendance du FOC](docs/13_audit_code_et_independance_foc.md) | Ce qui vient de VESC, ce qui est propre à RESC |
| 14 | [Fonctions RESC 1.0](docs/14_fonctions_v1.md) | Observateurs, capteurs, sensorless, filtres de phase, applications, induction, profils |
| — | [RESC 0.1 vs VESC 7.00 (PDF, anglais)](docs/RESC_vs_VESC_FOC_observer.pdf) | Comparatif du cœur FOC et de l'observateur, **version 0.1** (dépassé : pas de HFI, un seul observateur à l'époque) |

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

## Choix de conception

| Domaine | Choix | Alternative prévue |
|---|---|---|
| Ordonnancement | Hybride : ISR FOC hors noyau + ordonnanceur run-to-completion sur NVIC (sans RTOS tiers) | FreeRTOS pour les tâches non critiques |
| PWM | Centrée, SVPWM min-max, 1 échantillonnage + 1 mise à jour, compensation du retard par prédiction d'angle | Double échantillonnage / double mise à jour pour les moteurs à faible L |
| Mesure du courant | 3 shunts, sélection dynamique des 2 meilleures phases, déclenchement ADC adaptatif, repli par prédiction du modèle | Shunts de phase (variante matérielle) |
| f_PWM par défaut | Propre à chaque carte : 15 kHz sur l'Ubox (= `f_zv/2` de VESC), plage 8–40 kHz, validée selon la fenêtre ADC | 20–25 kHz après mesure de T_win |
| Courant | PI en repère d/q avec découplage + FEM en anticipation, anti-windup par back-calculation, limitation vectorielle | Deadbeat prédictif (option, moteurs à faible L) |
| Sensorless | Au choix : observateurs VESC ou observateurs RESC flux actif / FEM + PLL de type 2 | HFI / VSS à basse vitesse (IPM) |
| Protection absolue | Selon la carte (`BOARD_HW_OCP`) : BKIN si câblé, sinon nFAULT → break logiciel, sinon watchdog ADC + break logiciel. **Sur l'Ubox : logicielle uniquement** (doc 07 §7.8) | Modification matérielle : comparateur → PB12/BKIN |
| Chien de garde | IWDG + **enable dynamique** du driver (le signal meurt si l'ISR FOC s'arrête) | Moniteur d'échéance par timer → BKIN |

## Licence

GNU GPL v3 (voir [LICENSE](LICENSE)). Code tiers et éléments repris du projet VESC pour la compatibilité : voir [NOTICE.md](NOTICE.md).

> Firmware expérimental de commande de puissance, encore en phase d'essai sur banc :
> à utiliser avec une alimentation limitée en courant, sous votre responsabilité.
