# 3. Architecture temps réel

## 3.1 Exigences

| Exigence | Valeur cible | Tag |
|---|---|---|
| Période FOC | 25–40 µs (25–40 kHz), parfois 2× en double échantillonnage | [E] |
| Gigue de démarrage de l'ISR FOC | < 0,5 µs | [E] |
| Latence échantillonnage ADC → nouveaux CCR prêts | < 0,5·T_s (pour l'option double mise à jour) ou < T_s | [C] (découle de la modulation, doc 4) |
| Aucun blocage de la FOC par la flash, les communications ou les diagnostics | absolu | — |
| Boucles lentes (vitesse, position) | 1–10 kHz | [E] |

## 3.2 Approches comparées

| Critère | A. Bare-metal pur (ISR + boucle principale) | B. Ordonnanceur coopératif | C. RTOS (FreeRTOS/ChibiOS) pour tout | **D. Hybride : ISR hors noyau + run-to-completion sur NVIC** |
|---|---|---|---|---|
| Déterminisme FOC | Excellent | Excellent (FOC en ISR) | Bon si l'ISR FOC est au-dessus de `MAX_SYSCALL` | **Excellent** |
| Préemption entre tâches non critiques | Aucune | Aucune (une tâche lente retarde tout) | Complète | **Par niveaux de priorité NVIC** |
| Surcoût | ~0 | ~0 | Contexte ≈ 1–2 µs, ticks | **~0** (le NVIC est l'ordonnanceur) |
| Appels bloquants | Non | Non | Oui | Non (machines à états) |
| Analyse du pire cas | Simple | Simple | Plus complexe (inversions, files) | **Simple** (une pile, priorités fixes) |
| RAM | Minimale | Minimale | Une pile par tâche | Une seule pile (MSP) |
| Écosystème (USB, FS) | Manuel | Manuel | Riche | Manuel. Un RTOS reste ajoutable en niveau bas |
| Risque : section critique qui masque la FOC | Faible | Faible | Réel si mal configuré | **Nul par construction** : BASEPRI ne descend jamais sous le niveau FOC |

**Choix recommandé : D.** Le NVIC du Cortex-M4 (16 niveaux de préemption avec `PRIGROUP` = 4 bits) sert d'ordonnanceur préemptif à priorités fixes. Les niveaux « logiciels » sont des IRQ inutilisées déclenchées par `NVIC_SetPendingIRQ`. Toutes les tâches s'exécutent jusqu'au bout (run-to-completion) sur la pile principale. C'est le modèle « superloop + IRQ hiérarchisées » des variateurs industriels, avec une préemption réelle entre les communications et le travail de fond.

**Alternative prévue :** si un besoin futur l'impose (pile USB lourde, système de fichiers), FreeRTOS peut tourner **uniquement dans les niveaux les plus bas** (thread mode + PendSV). `configMAX_SYSCALL_INTERRUPT_PRIORITY` est alors placé sous le niveau des communications. Le cœur moteur ne dépend d'aucun noyau.

## 3.3 Carte des priorités (valeur NVIC basse = plus prioritaire)

| Prio | Source | Rôle | Durée max visée |
|---|---|---|---|
| — | Matériel | BKIN → MOE = 0 (asynchrone, sans CPU) ; CSS → break | ns |
| NMI | CSS / défaut d'horloge | Journaliser, état sûr | < 2 µs |
| 0 | TIM1_BRK | Journaliser le défaut, figer l'instantané, passer en FAULT. Ne réarme **jamais** | < 2 µs |
| 1 | ADC JEOC (fin des injectés) = **ISR FOC** | Mesures, protections rapides, FOC, observateur(s), PWM, journal | 5–10 µs [E] |
| 2 | TIM1_UP (optionnel) | Double mise à jour (doc 4) | < 2 µs |
| 3 | SWI « contrôle » (déclenchée par l'ISR FOC tous les N ticks) | Boucle vitesse/position, rampes, MTPA/FW lents, estimation thermique | < 20 µs |
| 5 | CAN RX/TX, DMA UART | Transfert vers/depuis des buffers circulaires uniquement | < 3 µs |
| 7 | SWI « comm » | Décodage des paquets, dispatch, réponses | < 200 µs |
| 10 | SWI « service » (1 kHz) | Supervision, chien de garde logique, statuts CAN, calculs lents | < 500 µs |
| thread | Boucle de fond | Écriture flash (moteur arrêté uniquement), idle, `WFI` | illimité |

**Règles :**
1. Aucune section critique au-dessus de la priorité 3 : on utilise `BASEPRI`, jamais `cpsid i` hors démarrage.
2. Échanges FOC ↔ reste par structures **sans verrou** :
   - consignes : boîte à lettres simple producteur / simple consommateur avec compteur de séquence (seqlock). La FOC lit la dernière version cohérente.
   - télémétrie : double tampon, échangé à chaque tick.
   - journal rapide : buffer circulaire SPSC (doc 9).
3. L'ISR FOC n'appelle **aucune** fonction hors section `.ramfunc` (vérification automatique sur le `.map`).

## 3.4 Instrumentation intégrée (toujours compilée, coût < 1 %)

| Mesure | Méthode |
|---|---|
| Durée de l'ISR FOC | `DWT->CYCCNT` à l'entrée et à la sortie → min/max/moyenne + histogramme 32 classes |
| Latence ADC → PWM | `TIM1->CNT` lu en fin d'ISR, comparé à la position de déclenchement et à l'échéance de mise à jour |
| Gigue | Écart entre l'entrée dans l'ISR et l'instant de déclenchement (CNT à l'entrée) → histogramme |
| Charge CPU | Cycles cumulés par niveau de priorité / cycles totaux, sur 100 ms |
| Dépassement d'échéance | (a) en fin d'ISR, `CNT` a-t-il franchi le point de mise à jour ? (b) le compteur de ticks FOC suit-il le compteur d'update du timer ? → défaut `DEADLINE_MISS` |
| Débordement ADC | Drapeaux OVR / JEOC manquant |
| Pile | Remplissage par motif + mesure du niveau max (watermark) |
| Oscilloscope | 2 GPIO de debug (ISR FOC active, prise d'échantillon), activables par configuration |

Toutes ces mesures sont exposées par la commande terminal `rt_stats` et par des champs custom config.
