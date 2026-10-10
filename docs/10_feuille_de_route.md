# 10. Feuille de route de développement et de validation

Principe : **chaque phase a un critère de sortie mesuré**. Une compilation réussie ne valide rien. Le courant de test augmente seulement après validation du palier précédent. Les stratégies expérimentales (deadbeat, HFI, surmodulation, ASC) ne sont jamais essayées d'abord à fort courant.

| Phase | Contenu | Puissance | Critères de sortie |
|---|---|---|---|
| **0. Audit** (ce dossier) | Études, choix, infos manquantes | — | Infos 🔴 de la doc 8 reçues ; ADR validées |
| **1. Socle** | Toolchain (GCC arm-none-eabi, CMake), linker (.ramfunc/.ccm/.noinit), BSP, CI (build cible + PC, tests unitaires, `check_ramfunc`, règles de dépendance, **`check_boards.py` + compilation de toutes les cartes REVIEWED**) ; `hw_validate()` (vérifications flottantes) ; **maintien d'alimentation PA15 de l'Ubox en tout premier** | Aucune | CI verte ; boot sur l'Ubox 85_200 sans extinction ; DWT opérationnel |
| **2. Communications** | UART/CAN VESC, `motor_api` simulé (moteur fictif), custom config | Aucune | Tests unitaires avec vecteurs de référence (CRC, encodages) ; interopérabilité avec un VESC réel, un display et VESC Tool (connexion, valeurs temps réel, FORWARD_CAN) ; aucune incidence sur la gigue d'un ISR factice à 25 kHz |
| **3. PWM + ADC sans puissance** | TIM1 centré, dead-time, déclenchement adaptatif, ADC simultané, sélection de phase, mesure de T_win | Driver alimenté, **pont sans Vbus** (ou Vbus bas courant limité) | Dead-time et polarités mesurés à l'oscilloscope ; temps d'ISR, latence et gigue mesurés ; T_win caractérisé |
| **4. Protections** | BKIN, sources de défaut, état Hi-Z, EN dynamique, moniteur d'échéance, IWDG, HardFault, break logiciel, journal pré-défaut | Injection de défauts (générateur sur les comparateurs, blocage volontaire de l'ISR, arrêt d'horloge) | Délai BKIN → grilles OFF mesuré pour **chaque** source ; la PWM s'arrête si l'ISR se bloque (< délai spécifié) ; aucun réarmement non autorisé ; 100 % des chemins de défaut testés |
| **5. Simulation** | Modèle PMSM + onduleur + ADC en C (sim/), boucle fermée avec le même code core | PC | Réponses indicielles du courant conformes (dépassement, temps) ; observateurs convergents ; scénarios de décrochage détectés |
| **6. FOC avec capteur** | Identification R, L (Ld/Lq), ψ, alignement codeur ; boucle de courant puis vitesse | Vbus réduit, alimentation de labo limitée en courant, puis I ≤ 10–20 % I_max | Bande passante du courant mesurée ; découplage vérifié à haute f_e ; compensation du dead-time validée |
| **7. Sensorless** | Observateur A3 + PLL, démarrage I/f, reprise au vol | ≤ 20–30 % I_max | Erreur d'angle vs codeur conforme (doc 6 §6.5) |
| **8. Comparaison des observateurs** | Ombre A2/A3/B2/SMO-ST/EKF, rejeu, réglage du sync_monitor (décrochages provoqués) | ≤ 30 % I_max | Rapport comparatif ; seuils du sync_monitor choisis avec taux de fausses alarmes et de détection mesurés |
| **9. Fréquences et latences** | f_PWM optimale (pertes vs ondulation vs fenêtre), S1/U1 asymétrique, budget CPU | Puissance croissante | Mesures de rendement et de température ; marge CPU ≥ 30 % |
| **10. MTPA / FW / haut duty** | MTPA, défluxage, repli de mesure à haut m, surmodulation (option), politique ASC | Montée progressive, vitesse puis courant | Stabilité à f_e max ; transitions sans à-coups ; essai de coupure à haut régime (UCG/ASC) à courant réduit |
| **11. Validation à puissance croissante** | Paliers 25 / 50 / 75 / 100 % ; endurance ; thermique ; EMI ; non-régression | Pleine | Pas de défaut sur le profil d'endurance ; protections retestées à pleine puissance (court-circuit contrôlé sur charge dédiée, si le banc le permet) |

## Tests et non-régression (en continu)

- **Unitaires** (PC) : math, modulation (fenêtres vs `svpwm_window.py`), PI (réponse), observateurs (convergence sur trajectoires synthétiques), protocole (vecteurs générés à partir de trames VESC capturées).
- **Simulation** : scénarios nommés (échelon de couple, inversion, perte de mesure, R faussé, décrochage, coupure de BMS en freinage) avec critères chiffrés. Exécutés à chaque commit.
- **HIL/banc** : scripts CAN/UART reproductibles ; journaux archivés et rejouables.
- **Revue de sécurité** à chaque modification de `safety/`, `hw/drivers/tim_pwm`, `hardware_limits.h` ou du linker script.

## Prochaines actions immédiates

1. Configuration matérielle : **faite** pour l'Ubox 85_200 (doc 11). Restent les points U1–U8 de la [doc 8](08_infos_manquantes.md).
2. Décider de l'option de protection matérielle de l'Ubox (doc 07 §7.8 : rien, comparateur sur PB12, ou + capteur de bus) **avant** les essais au-delà de 25 % du courant.
3. Lancer la phase 1 (socle, linker, CI) en parallèle : elle ne dépend pas du matériel.
