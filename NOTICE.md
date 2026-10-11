# Licences et code tiers

RESC est distribue sous licence **GNU GPL v3** (voir `LICENSE`).

## Code tiers inclus

| Fichiers | Origine | Licence |
|---|---|---|
| `firmware/cmsis/stm32f4xx.h`, `stm32f405xx.h`, `system_stm32f4xx.h` | STMicroelectronics, cmsis_device_f4 v2.6.10 | Apache 2.0 |
| `firmware/cmsis/core_cm4.h`, `cmsis_*.h`, `mpu_armv7.h` | Arm, CMSIS_5 5.9.0 | Apache 2.0 |

## Fichiers derives du projet VESC (bldc, branche release_7_00, GPL v3, Benjamin Vedder)

Le coeur de commande RESC (FOC, controle, protections, detections, observateurs
RESC_..., types 7 a 13) est un developpement independant. Pour l'interoperabilite avec VESC Tool, les ecrans
et les controleurs existants, les elements suivants reprennent des **formats et des
donnees** du projet VESC :

| Fichiers | Contenu repris |
|---|---|
| `firmware/src/vesc/vesc_conf.c`, `vesc_conf.h`, `vesc_defaults.h`, `conf_xml.c` | generes par `tools/resc_conf/gen_resc_conf.py` depuis les XML de VESC Tool 7.00 (`firmware/conf/vesc700/`) : ordre et encodage des champs de configuration, signatures, valeurs par defaut, enumerations |
| `firmware/src/core/observer.c` (observateurs 0 a 6) | equations, coefficients et bornes des observateurs de `motor/foc_math.c` (`foc_observer_update`), reimplementes : ORTEGA_ORIGINAL, ORTEGA_LAMBDA_COMP, MXV, MXV_LAMBDA_COMP, MXV_LAMBDA_COMP_LIN (Benjamin Vedder, GPL v3) et MXLEMMING, MXLEMMING_LAMBDA_COMP (voir ci-dessous) |
| `firmware/src/comm/*` | identifiants et formats des messages CAN/UART (reimplementation) |
| `hw/boards/imported/**`, `docs/data/vesc_hw_survey.csv` | brochages et valeurs des cartes, extraits de `hwconf` par `tools/analysis/vesc_hw_survey.py` |
| `hw/boards/ubox/*`, `hw/boards/common/*` | brochages et valeurs relus dans `hwconf` (Ubox, cartes VESC courantes) |

Source de reference : https://github.com/vedderb/bldc/tree/release_7_00

## Observateur MXLEMMING

L'observateur MXLEMMING (`firmware/src/core/observer.c`, types 1 et 3) a pour auteur
**David Molony** (projet MESC FOC, licence BSD 3 clauses avec la condition suivante, reproduite
dans le code comme elle le demande) : la note de licence doit accompagner toute implementation ou
modification, et David Molony doit etre cite comme auteur original.
