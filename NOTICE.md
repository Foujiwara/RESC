# Licences et code tiers

RESC est distribue sous licence **GNU GPL v3** (voir `LICENSE`).

## Code tiers inclus

| Fichiers | Origine | Licence |
|---|---|---|
| `firmware/cmsis/stm32f4xx.h`, `stm32f405xx.h`, `system_stm32f4xx.h` | STMicroelectronics, cmsis_device_f4 v2.6.10 | Apache 2.0 |
| `firmware/cmsis/core_cm4.h`, `cmsis_*.h`, `mpu_armv7.h` | Arm, CMSIS_5 5.9.0 | Apache 2.0 |

## Fichiers derives du projet VESC (bldc, branche release_7_00, GPL v3, Benjamin Vedder)

Le coeur de commande RESC (FOC, observateurs, controle, protections, detections)
est un developpement independant. Pour l'interoperabilite avec VESC Tool, les ecrans
et les controleurs existants, les elements suivants reprennent des **formats et des
donnees** du projet VESC :

| Fichiers | Contenu repris |
|---|---|
| `firmware/src/vesc/vesc_conf.c`, `vesc_conf.h`, `vesc_defaults.h` | generes par `tools/vesc_compat/gen_conf.py` : ordre et encodage des champs de configuration, signatures, valeurs par defaut, enumerations |
| `firmware/src/comm/*` | identifiants et formats des messages CAN/UART (reimplementation) |
| `hw/boards/imported/**`, `docs/data/vesc_hw_survey.csv` | brochages et valeurs des cartes, extraits de `hwconf` par `tools/analysis/vesc_hw_survey.py` |
| `hw/boards/ubox/*`, `hw/boards/common/*` | brochages et valeurs relus dans `hwconf` (Ubox, cartes VESC courantes) |

Source de reference : https://github.com/vedderb/bldc/tree/release_7_00
