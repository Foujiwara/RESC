# 11. Cartes supportées : configuration matérielle compatible VESC

Le firmware doit tourner **sur les mêmes cartes que VESC**. La configuration matérielle suit donc le même principe que les `hwconf` de VESC : un fichier par carte, choisi à la compilation. Le format est le nôtre, il ne dépend pas de ChibiOS. Le brochage commun et les valeurs de chaque carte sont **repris des fichiers de VESC 7.00**.

## 11.1 Organisation

```
hw/
├── hw_config.h                    point d'entrée unique ; contrat documenté
├── hw_types.h                     broches (HW_PA0…), types de driver, topologies, statuts
├── hw_check.h                     vérifications à la compilation (#error)
└── boards/
    ├── common/board_common_f405.h   brochage commun VESC (défauts #ifndef)
    ├── ubox/                      CARTE DE RÉFÉRENCE, relue à la main (REVIEWED)
    │   ├── board_ubox_100_core.h
    │   ├── board_ubox_single_85_200.h   ← cible des essais
    │   ├── board_ubox_single_80.h, board_ubox_single_100.h, board_ubox_v2_100.h
    └── vesc/<fabricant>/board_*.h 150 cartes générées (IMPORTED)
```

Sélection de la carte : `-DRESC_BOARD_HEADER='"boards/ubox/board_ubox_single_85_200.h"'`. C'est l'équivalent de `HW_HEADER` chez VESC. Les algorithmes ne lisent que les macros `BOARD_*`, `HWLIM_*` et `DEF_*`. Changer de carte ne modifie donc ni la FOC, ni les observateurs, ni les protections.

| Préfixe | Nature | Défaut commun autorisé ? |
|---|---|---|
| `BOARD_*` broches, affectations ADC | fait matériel | **oui** (brochage VESC commun) |
| `BOARD_*` analogique (shunt, gain, VREF, diviseurs, NTC), dead-time, protection | fait matériel | **non**, doit être défini par la carte |
| `HWLIM_*` | limite **absolue** (issue de `HW_LIM_*`) | non |
| `DEF_*` | défaut d'un paramètre utilisateur (issu de `MCCONF_*`) | non |

## 11.2 Recensement des cartes VESC 7.00 [C]

Outil : [tools/analysis/vesc_hw_survey.py](../tools/analysis/vesc_hw_survey.py). Il contient un mini-préprocesseur qui résout `#define`/`#if` et lit `hw_setup_adc_channels()` dans les `.c`. Données : [docs/data/vesc_hw_survey.csv](data/vesc_hw_survey.csv).

- 314 en-têtes `hw_*.h` ; **172 configurations** une fois retirés les `_core` et `_no_limits`.
- **154 compatibles avec la cible** (LQFP64, mono-moteur, TIM1 standard). Les 18 autres sont exclues : boîtier LQFP100 (TIM1 sur PE8–PE13, PD/PE) et/ou bi-moteur (TIM8). Il s'agit des Duet, Stormcore, Unity, MBOT, a50s_dual, EDU et Luna M600.

### Brochage commun retenu (défauts de `board_common_f405.h`)

| Fonction | Brochage | Cartes (sur 172) |
|---|---|---|
| PWM haut / bas | PA8, PA9, PA10 / PB13, PB14, PB15 (TIM1) | 156 |
| Courants A/B/C | IN10/IN11/IN12 = PC0/PC1/PC2 sur ADC1/ADC2/ADC3 | ≈ 134 |
| Tensions de phase | IN0/IN1/IN2 = PA0/PA1/PA2 sur ADC1/2/3 | ≈ 141 |
| Vbus | IN13 = PC3 sur ADC3 | 141 |
| T_FET / T_moteur | IN3 = PA3 / IN14 = PC4 | 121 / 119 |
| ADC_EXT1 / EXT2 | PA5 / PA6 | 121 / 135 |
| UART | USART3 PB10/PB11 (I2C2 sur les mêmes broches) | 146 |
| CAN | CAN1 PB8/PB9 (défaut de `hw.h`) | 157 |
| Halls / codeur | PC6/PC7/PC8, TIM3 | 148 |
| Servo/PPM | PB6 (TIM4_CH1) | 139 |
| SPI capteur | SPI1 PA5/PA6/PA7, NSS PA4 (95) ou PB11 (66) | 132 |

### Ce qui varie et que chaque carte doit fournir

| Élément | Constat |
|---|---|
| Gate driver | 143 « discret/sans SPI », 17 DRV8301, 11 DRV8323S, 1 DRV8313 |
| Shunts | 148 à 3 shunts, dont 54 **shunts de phase en ligne** ; 24 à 2 shunts |
| Polarité inversée de l'ampli | 53 cartes |
| Dead-time | de 0 à 1500 ns (660 ns : 21 cartes ; non défini → 360 ns par défaut VESC : 35) |
| Shunt / gain | 0,5 mΩ ×20 le plus fréquent ; de 0,1 mΩ à 1 mΩ, gains de 10 à 50 |
| **BKIN câblé** | **7 / 172** seulement (toujours PB12) |
| Fréquence | `foc_f_zv` = 30 kHz sur 109 cartes → **f_PWM = 15 kHz** (`mcpwm_foc.c` : `ARR = F_clk / f_zv` en mode centré) [C] |

**Contrainte ADC confirmée [C, DS8626] :** seules les voies IN0–3 et IN10–13 existent sur ADC3. Les voies IN4–9, IN14 et IN15 ne sont disponibles que sur ADC1/ADC2. `hw_check.h` le vérifie pour les courants et les tensions mesurés simultanément.

## 11.3 Carte de référence : Ubox SINGLE 85_200

Sources : `hwconf/Ubox/100v/hw_ubox_single_85_200.h` + `hw_ubox_100_core.{h,c}`, relues ligne à ligne. Résultat : [board_ubox_100_core.h](../hw/boards/ubox/board_ubox_100_core.h). Le recensement automatique donne les mêmes valeurs, ce qui confirme les deux.

| Élément | Valeur | Conséquence |
|---|---|---|
| Driver | discret, **ni EN, ni nFAULT, ni BKIN** exposés au MCU | voir §11.4 |
| Courants | 3 shunts **low-side**, 0,5 mΩ/3 = 167 µΩ, gain 20, **polarité inversée** | 3,33 mV/A ; **±516 A** de pleine échelle ; 0,25 A/LSB [E] |
| VREF | 3,44 V (`V_REG`) | à confirmer par mesure de VDDA [V] |
| Vbus | 150 k / 4,7 k | 113 V de pleine échelle, 27,6 mV/LSB [E] |
| Tensions de phase | PA0–PA2, diviseur supposé identique à Vbus (VESC le traite ainsi) | [V] |
| T_FET | max de 3 NTC 10 k β 3380 (PA3, PB0, PB1), NTC côté VREF | — |
| Dead-time | 660 ns | — |
| Alimentation | **PA15 = maintien du régulateur 12 V**, PC13 = bouton (astuce open-drain, niveau bas ≈ 1 V) | **PA15 doit être activée en tout premier au boot**, sinon la carte s'éteint |
| COMM | USART3 PB10/PB11 ; UART4 PC10/PC11 (module NRF52) ; CAN1 PB8/PB9 | — |
| IMU | BMI160 en I2C logiciel PB4/PB12 | **PB12 (BKIN) est occupée** |
| Limites | ±300 A moteur, ±300 A entrée, coupure absolue 420 A, 11–85 V, 200 kERPM, duty 0,99, 110 °C | `HWLIM_*` |
| Défauts | 15 kHz, coupure 400 A, Vbus 12–82 V, entrée +250/−200 A | `DEF_*` |

Conflit relevé dans VESC : `NRF5x_SWDIO` est défini sur PA15, qui sert aussi au maintien d'alimentation. La programmation SWD du module NRF n'est pas reprise.

**Fenêtres de mesure (doc 04) à 15 kHz :** T_s = 66,7 µs. La fenêtre 2 shunts à m = 1 vaut 6,7 % × T_s = **4,47 µs**, ce qui est confortable avec un dead-time de 660 ns. Avec T_win = 3 µs (valeur prudente par défaut), la limite est de **22 kHz** pour garder toute la plage linéaire. T_win doit être mesuré [V].

## 11.4 Conséquence majeure : la protection absolue sur le matériel VESC

| `BOARD_HW_OCP` | Signification | Cartes (154) |
|---|---|---|
| 0 | aucune protection indépendante **connue du MCU** | 104 importées + 4 Ubox |
| 1 | protection interne du driver non remontée | 0 (classées en 2 si nFAULT) |
| 2 | défaut driver sur GPIO (nFAULT → EXTI) | 41 |
| 3 | défaut sur BKIN (coupure matérielle sans CPU) | 5 |

Sur la carte de test (Ubox), **l'exigence d'une coupure indépendante du CPU (cahier des charges §9) ne peut pas être remplie par le firmware seul.** Le firmware va le plus loin possible sans modification matérielle :
1. watchdog analogique de l'ADC sur le groupe injecté (seuils ±I_trip) → interruption prioritaire → break logiciel de TIM1 (`EGR.BG`), qui passe par le même chemin que le matériel ;
2. contrôle à **chaque** échantillon dans l'ISR FOC, plus le détecteur de perte de synchronisation (doc 07 §7.7) ;
3. enable dynamique impossible (pas d'EN) → IWDG en dernier recours.

**Limite physique :** il n'y a qu'un échantillon valide par période, quand le low-side conduit. On peut donc rester aveugle jusqu'à 66 µs à 15 kHz. Le cas « surintensité pendant la conduction high-side » n'est **pas couvert**. Voir doc 07 §7.8 pour les options matérielles.

## 11.5 Statuts et politique de sécurité

| Statut | Signification | Pont |
|---|---|---|
| `IMPORTED` | généré automatiquement | **interdit** (sauf build banc `RESC_ALLOW_UNREVIEWED_BOARD`) |
| `REVIEWED` | relu à la main contre les sources | autorisé, courant ≤ 25 % des `HWLIM_*` |
| `VALIDATED` | validé au banc (phases 3–4 de la doc 10) | pleine plage |

Le générateur ne voit pas ce que les `.c` de VESC font de spécifique : maintien d'alimentation, multiplexeurs analogiques, détection de défaut par fonction. Exemple vérifié : sur l'Ubox, il rate PA15/PC13 ; sur la STR365/STR500, la T_FET passe par un multiplexeur. C'est pourquoi **aucune carte générée n'est utilisable sans relecture**.

## 11.6 Outils et vérification

| Outil | Rôle |
|---|---|
| `tools/analysis/vesc_hw_survey.py <bldc/hwconf> <csv>` | recensement → CSV |
| `tools/hw_import/vesc_board_import.py <csv> hw/boards/imported` | génération des cartes IMPORTED |
| `tools/hw_import/check_boards.py . [--quiet]` | évalue `hw_check.h` pour chaque carte (mini-préprocesseur), contrôle les broches hors LQFP64 et les **doublons** (sauf partages voulus du connecteur COMM et des sondes FET) |

État actuel ([rapport](data/board_check_report.txt)) : **154 cartes, 0 erreur sur les cartes relues** (les 4 Ubox). 33 cartes importées sont signalées (doublons réels, multiplexeurs, broches hors boîtier). Ce sont les premières à relire si l'on veut les supporter.

Tests négatifs effectués : 8 mutations de la carte Ubox (dead-time 0, 2 courants sur le même ADC, OCP = 3 sans BKIN, doublon de broche, défaut hors limites, shunt manquant, broche LQFP100, UART incohérent). **Les 8 sont détectées.**

> ⚠ Aucun compilateur C n'est installé sur ce poste. Les en-têtes ont été vérifiés avec le mini-préprocesseur, **pas compilés**. La compilation réelle (arm-none-eabi-gcc) fait partie de la phase 1.

## 11.7 Ajouter ou relire une carte

1. Partir du fichier généré (`hw/boards/imported/…`) ou du modèle Ubox.
2. Relire le `.h` **et le `.c`** VESC : `hw_init_gpio`, `hw_setup_adc_channels`, fonctions de température et d'arrêt.
3. Compléter les points `[V]`, renseigner `BOARD_HW_OCP` honnêtement.
4. `check_boards.py` sans erreur → passer en `HW_BOARD_STATUS_REVIEWED`, en déplaçant le fichier dans un dossier de fabricant hors `vesc/` pour qu'une régénération ne l'écrase pas, et l'ajouter à `MANUAL` dans le générateur.
5. Après les essais de la doc 10 (phases 3–4) → `HW_BOARD_STATUS_VALIDATED`.
