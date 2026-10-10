# 1. Audit du STM32F405RGT6

Sources : datasheet DS8626 (STM32F405xx/407xx) et manuel de référence RM0090. Les points marqués [C] doivent être relus dans la révision du document utilisée pour le PCB.

## 1.1 Caractéristiques principales

| Ressource | Valeur | Tag | Conséquence pour le firmware |
|---|---|---|---|
| Cœur | Cortex-M4F, 168 MHz, FPU **simple précision** uniquement | [C] | Tout le calcul FOC en `float`. Interdire `double` (`-fsingle-precision-constant`, `-Wdouble-promotion`) |
| Flash | 1 Mo, banque **unique**, secteurs 4×16 Ko + 1×64 Ko + 7×128 Ko | [C] | Effacer ou écrire la flash bloque la lecture de la flash (voir §1.4) |
| Accélérateur flash | ART (cache I 64 lignes, prefetch), 5 wait states à 168 MHz / 3,3 V | [C] | Gigue d'exécution si le code n'est pas dans le cache → code critique en RAM |
| SRAM | 112 Ko SRAM1 + 16 Ko SRAM2 + **64 Ko CCM** | [C] | CCM : bus D seul, **pas d'exécution de code, pas de DMA** sur F405 |
| Boîtier | LQFP64 : 51 GPIO, 16 voies ADC externes | [C] | Multiplexage serré (voir §1.3) |
| ADC | 3 × 12 bits, 2,4 Msps chacun, modes simultané/entrelacé, 4 rangs injectés par ADC | [C] | Mesure **simultanée** de Ia, Ib, Ic (ADC1/2/3 en mode injecté simultané) |
| Horloge ADC | ≤ 36 MHz ; APB2 = 84 MHz, diviseurs /2 /4 /6 /8 → **21 MHz** (/4) | [C] | 12 bits + 3 cycles d'échantillonnage = 0,71 µs ; + 15 cycles = 1,29 µs |
| Timers avancés | TIM1, TIM8 (168 MHz), sorties complémentaires, dead-time, BKIN, RCR | [C] | TIM1 = pont triphasé. TIM8 en réserve (2e moteur ou autre usage) |
| Résolution PWM | 168 MHz en mode centré → ARR = 168e6/(2·f_PWM) : 3360 pas à 25 kHz | [C] | ≈ 11,7 bits de duty, suffisant |
| Dead-time | DTG 8 bits, t_DTS = 5,95 ns (CKD=0), jusqu'à ≈ 6 µs | [C] | Réglé dans `hardware_config.h` et verrouillé (LOCK) |
| Break | **Une seule entrée BKIN par timer avancé**, polarité programmable, **pas de filtre numérique** (BKF n'existe pas sur F4), pas de BKIN2 | [C] | Filtrage RC externe obligatoire. Une seule entrée → **OU câblé** des sources de défaut |
| Break par horloge | La défaillance HSE détectée par le CSS déclenche aussi le break de TIM1/TIM8 | [C] relire RM0090 §17.3.13 | Couverture gratuite de la panne d'horloge |
| Comparateurs analogiques | **Aucun** (contrairement aux F3, G4, F30x) | [C] | Protection absolue = comparateurs **externes** |
| Ampli-op internes | **Aucun** | [C] | Amplis de shunt externes (ou intégrés au gate driver) |
| Watchdog analogique ADC | Oui, 1 par ADC. Il génère une **interruption**, pas un break | [C] | Utilisable en surveillance logicielle rapide seulement, pas en protection absolue |
| DMA | DMA1, DMA2 (8 flux chacun). ADC → DMA2 uniquement | [C] | Buffers DMA en SRAM1/SRAM2, jamais en CCM |
| CAN | 2 × bxCAN 2.0B, 1 Mbit/s, 3 boîtes TX, 2 FIFO RX de profondeur 3, 28 bancs de filtres partagés ; CAN2 dépend de l'horloge de CAN1 | [C] | FIFO RX peu profonde → ISR CAN rapide (voir §1.5) |
| UART | USART1/6 sur APB2 (jusqu'à 10,5 Mbit/s), USART2/3, UART4/5 | [C] | UART VESC par défaut à 115200 bauds avec DMA circulaire en RX |
| USB | OTG FS (PA11/PA12) | [C] | Hors périmètre initial, mais VESC Tool l'utilise beaucoup (voir doc 2) |
| Watchdogs | IWDG (LSI ≈ 32 kHz, indépendant), WWDG (APB1) | [C] | IWDG + enable dynamique externe (doc 7) |
| Compteur de cycles | DWT->CYCCNT | [C] | Mesure du temps d'ISR à 6 ns près |
| Tension | Détecteur PVD programmable, BOR | [C] | Défaut « MCU sous-tension » |

## 1.2 Budget CPU — ordres de grandeur

Coûts unitaires Cortex-M4F [C, ARM TRM] : mul/add/MAC flottant = 1 cycle (VMLA 3 cycles), division et racine flottantes = 14 cycles. L'empilement paresseux du FPU ajoute ≈ 12 à 30 cycles par interruption.

Estimation pour une ISR FOC complète, code optimisé (`-O2`, sin/cos par table interpolée ou polynôme) [E] :

| Bloc | Cycles [E] |
|---|---|
| Entrée/sortie ISR + contexte FPU | 30–50 |
| Lecture ADC, offsets, gains, validité | 30–60 |
| Clarke + Park + Park inverse | 30 |
| sin/cos (table 512 + interpolation ou polynôme d'ordre 5) | 30–50 |
| 2 PI + découplage + anticipation + anti-windup | 60–100 |
| Limitation vectorielle (sqrt, div) | 40–60 |
| SVPWM min-max + sélection des phases + déclenchement ADC | 50–80 |
| Observateur de flux non linéaire + PLL | 150–300 |
| Protections logicielles + journal circulaire | 80–150 |
| **Total** | **≈ 500–900 cycles ≈ 3–5,5 µs** |

En pratique, compter **× 1,5 à 2** pour le compilateur et les accès mémoire, soit **≈ 5–10 µs**. À 25 kHz (40 µs), la charge est de 15 à 25 % [E]. Il reste de la marge pour 1 ou 2 observateurs en mode ombre ou pour un EKF (doc 6). **À mesurer** au DWT dès l'étape 3 de la feuille de route [V].

## 1.3 Multiplexage des broches (LQFP64) — contraintes identifiées

Les alternatives ci-dessous sont [C] d'après le tableau des fonctions alternatives de DS8626. **Le brochage réel est imposé par le PCB existant**, qui n'est pas fourni (voir doc 8). Ce tableau sert à le vérifier.

| Fonction | Options de broches (LQFP64) | Conflits notables |
|---|---|---|
| TIM1 CH1/2/3 | PA8, PA9, PA10 | PA9/PA10 = USART1 et VBUS/ID USB |
| TIM1 CH1N/2N/3N | PB13/PB14/PB15 **ou** PA7/PB0/PB1 | PB13 = CAN2_TX ; PB0/PB1 = ADC IN8/IN9 ; PA7 = ADC IN7 |
| TIM1_BKIN | **PA6** ou **PB12** | PB12 = CAN2_RX ; PA6 = TIM8_BKIN, ADC IN6, SPI1_MISO |
| TIM8 CH1/2/3 | PC6, PC7, PC8 | = USART6 TX/RX |
| CAN1 | **PB8/PB9** ou PA11/PA12 | PA11/PA12 = USB |
| CAN2 | PB12/PB13 ou PB5/PB6 | Inutilisable avec TIM1 sur PB13 et BKIN sur PB12 |
| USART3 | PB10/PB11 | — |
| USART6 | PC6/PC7 | TIM8 |
| Codeur ABI | TIM3 (PA6/PA7, PB4/PB5, PC6/PC7) ou TIM4 (PB6/PB7) | — |
| Entrées ADC | PA0–PA7 (IN0–7), PB0–PB1 (IN8–9), PC0–PC5 (IN10–15) | 16 voies au total, partagées avec les fonctions ci-dessus |

**Pièges confirmés :**
1. TIM1_BKIN et TIM8_BKIN partagent PA6. Une broche ne peut porter qu'une fonction alternative à la fois.
2. Si TIM1 utilise PB13–PB15 et BKIN sur PB12, **CAN2 n'est pas disponible** (CAN1 sur PB8/PB9 reste possible).
3. Si TIM1 utilise PA7/PB0/PB1, on perd 3 voies ADC.
4. Les sorties PWM doivent avoir des **pull-down externes** (ou le niveau qui bloque le driver) : pendant le reset et le boot, les GPIO sont en entrée flottante [C].

> **Mise à jour :** le firmware vise les cartes VESC existantes. Le brochage effectif est repris des `hwconf` de VESC 7.00 : brochage commun et cartes dans la [doc 11](11_cartes_vesc.md) et dans `hw/boards/`. Le tableau ci-dessous reste une proposition pour un **nouveau** PCB.
>
> Contrainte supplémentaire [C, DS8626] : IN4–IN9, IN14 et IN15 n'existent que sur ADC1/ADC2. Seules IN0–3 et IN10–13 sont présentes sur les trois ADC.

Proposition de référence pour un nouveau PCB [E, à valider sous STM32CubeMX] :

| Fonction | Broche |
|---|---|
| PWM HS / LS | PA8, PA9, PA10 / PB13, PB14, PB15 |
| BKIN (OU câblé des défauts) | PB12 |
| Ia, Ib, Ic | PC0, PC1, PC2 (IN10–12), sur ADC1/2/3 en simultané |
| Va, Vb, Vc (tensions de phase) | PA0, PA1, PC3 |
| Vbus | PA4 |
| T_FET, T_moteur | PC4, PC5 |
| CAN1 | PB8 (RX), PB9 (TX) |
| UART VESC | USART3 PB10/PB11 |
| UART 2 (display/debug) | USART6 PC6/PC7 |
| Codeur ABI | TIM4 PB6/PB7 + index en EXTI |
| Codeur SPI (AS5047…) | SPI3 PC10/PC11/PC12 + CS PA15 |
| Halls | TIM3 PB4/PB5/PB0, ou GPIO EXTI |
| Moniteur d'échéance (TIM9_CH1) → BKIN | PA2 |
| Enable dynamique du driver | PC9 (sortie timer ou GPIO basculée par l'ISR) |
| Lecture de la source de défaut | GPIO dédiées (nFAULT, OC_A/B/C, OV) |
| SWD | PA13/PA14 |

## 1.4 Contrainte critique : flash à banque unique

Sur le F405 [C] :
- Pendant un effacement ou une programmation, **toute lecture de la flash par le CPU (instruction ou donnée) est bloquée** jusqu'à la fin de l'opération.
- L'effacement d'un secteur dure de **quelques centaines de ms (16 Ko) à 1–2 s (128 Ko)**, avec des maxima de plusieurs secondes [C, DS8626 « Flash memory programming »].

**Conséquence :** si une ISR FOC exécute du code ou lit une constante en flash pendant un effacement, elle est figée. La PWM continue alors avec les derniers rapports cycliques et le courant n'est plus régulé. C'est un **risque de destruction**.

**Mesures retenues (cumulatives) :**
1. Table des vecteurs relocalisée en SRAM (VTOR).
2. ISR FOC, ISR break, protections et **toutes leurs dépendances** (sin/cos, tables, constantes, `memcpy` éventuel) en SRAM1. Section `.ramfunc` vérifiée par un script sur le `.map` (aucun symbole appelé hors RAM).
3. État FOC et piles en CCM (accès à 0 wait state, pas de DMA).
4. Écriture de la configuration **autorisée uniquement moteur arrêté**, ponts désactivés, vitesse estimée nulle. La règle est imposée par la couche de stockage, pas par l'appelant.
5. Stockage de la configuration dans les secteurs de 16 Ko (1 et 2) en A/B : effacement court et retour arrière possible.

## 1.5 Communications : contraintes de temps

- CAN à 1 Mbit/s, trame étendue de 8 octets ≈ 130 bits ≈ 130 µs. Avec une FIFO de 3, l'ISR CAN doit être servie en moins de ≈ 390 µs [E]. L'ISR FOC (< 10 µs) ne gêne pas cette échéance.
- UART à 115200 bauds : 87 µs/octet. Le DMA circulaire évite une interruption par octet.
