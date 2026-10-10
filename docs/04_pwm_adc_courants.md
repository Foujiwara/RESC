# 4. PWM, ADC et mesure des courants

Les calculs de ce document sont reproductibles avec [tools/analysis/svpwm_window.py](../tools/analysis/svpwm_window.py).

## 4.1 Porteuse : centrée ou alignée à gauche

| Critère | Centrée (up/down) | Alignée à gauche (edge) |
|---|---|---|
| Harmoniques | Ondulation 2× plus faible à f_PWM égale (les fronts des phases sont décalés) | Plus forte |
| Point d'échantillonnage « moyen » | **Au sommet ou au creux, le courant échantillonné ≈ courant moyen** (l'ondulation est symétrique) | Il faut corriger l'ondulation |
| Fenêtre low-side commune | Centrée au sommet, symétrique | Collée en fin de période |
| Fréquence d'update possible | 1× ou 2× par période | 1× |
| EMI | Fronts répartis | Fronts simultanés en début de période |

**Choix : centrée** [C, pratique universelle des variateurs industriels]. L'alignement à gauche ne présente aucun avantage pour 3 shunts.

## 4.2 Modulation

| Méthode | Tension max linéaire (crête phase) | Pertes de commutation | Remarque |
|---|---|---|---|
| SPWM sinus | Vdc/2 | référence | 15 % de tension perdue |
| **SVPWM symétrique (injection min-max)** | **Vdc/√3** | référence | Équivalent à la SVPWM « par secteurs », mais sans calcul de secteur ni trigonométrie. ≈ 15 cycles |
| DPWM (discontinue, 60° clampés) | Vdc/√3 | **-33 %** | Une phase reste collée à 0 ou 1 → une fenêtre low-side nulle ou pleine. Couplage fort avec la mesure 3 shunts |
| Surmodulation (Bolognani, Holtz) | jusqu'à 2Vdc/π (six-step) | — | Fenêtres ADC fermées → mesure partiellement impossible |

**Choix :** SVPWM min-max par défaut. La DPWM est un **module optionnel** pour les hautes vitesses (gain de rendement), à activer seulement si la stratégie de mesure le permet. Avec la DPWM « clampée à 0 » (DPWMMIN), la phase clampée a une fenêtre low-side pleine, ce qui **facilite** la mesure 3 shunts [E, à simuler].

## 4.3 Échantillonnage et mise à jour

On note T_s = 1/f_PWM. Le retard équivalent est la somme du calcul et du bloqueur d'ordre zéro.

| Schéma | Fréquence FOC | Retard équivalent [C, théorie] | Charge CPU | Remarque |
|---|---|---|---|---|
| **S1/U1** : 1 échantillon au sommet, 1 update/période | f_PWM | ≈ 1,5 T_s | 1× | Simple et robuste. Retard compensé par prédiction d'angle |
| S1/U1 asymétrique : échantillon au sommet, update au creux | f_PWM | ≈ 1,0 T_s | 1× | Le calcul doit finir en T_s/2. Impulsions asymétriques mais volt-secondes correctes |
| **S2/U2** : échantillon au sommet et au creux, update 2× | 2·f_PWM | ≈ 0,75 T_s | 2× | Au creux, les low-side sont OFF : en 3 shunts, **le courant n'est mesurable qu'au sommet** → l'échantillon du creux n'existe pas en low-side. ⚠ Non applicable sans shunts de phase |
| S1/U2 : 1 échantillon, 2 updates (la 2e prédite) | f_PWM | ≈ 1,0 T_s | 1,3× | Seconde moitié calculée à partir de la prédiction du modèle |

**Conséquence de la topologie low-side :** le seul instant de mesure commun est le vecteur nul V0 (000), centré au sommet du compteur. La double mesure par période n'est possible qu'avec des **shunts de phase** (variante future).

**Choix :** S1/U1 symétrique par défaut, avec **compensation du retard** :
θ_appliqué = θ_k + ω̂·(1,5·T_s) pour la Park inverse, et θ_mesure aligné sur l'instant d'échantillonnage pour la Park directe.

Sans compensation, l'erreur d'angle vaut ω_e·T_d (calcul du script) :

| f_PWM \ f_e | 0,5 kHz | 1 kHz | 2 kHz | 3 kHz |
|---|---|---|---|---|
| 20 kHz | 13,5° | 27° | 54° | 81° |
| 30 kHz | 9° | 18° | 36° | 54° |
| 40 kHz | 6,7° | 13,5° | 27° | 40,5° |

Au-delà d'environ 1 kHz électrique, l'erreur sans compensation dégrade fortement le couplage d/q et la stabilité. La compensation est donc **obligatoire**, pas optionnelle. L'option S1/U1 asymétrique est conservée (paramètre) pour les moteurs à haute f_e.

## 4.4 Fenêtres de mesure avec 3 shunts low-side

La fenêtre low-side de la phase x vaut (1 − d_x)·T_s, centrée au sommet. Seules **deux phases** sont nécessaires (Ic = −Ia − Ib). À chaque période, on choisit les deux phases dont le rapport cyclique est le plus faible.

Fenêtre minimale de la 2e meilleure phase, sur tout l'angle électrique [C, calcul exact] :

| m (1 = limite linéaire SVPWM) | Fenêtre 2 shunts (× T_s) | Fenêtre 3 shunts (× T_s) |
|---|---|---|
| 0,5 | 0,284 | 0,250 |
| 0,8 | 0,154 | 0,100 |
| 0,9 | 0,110 | 0,050 |
| 0,95 | 0,089 | 0,025 |
| **1,0** | **0,067** | **0** |

Le pire cas se situe à la frontière entre deux secteurs, où deux phases ont un rapport cyclique de 0,933.

La fenêtre nécessaire vaut **T_win ≥ t_dead + t_settle + t_sample** si l'instant de déclenchement est optimisé (§4.5). Sinon, avec un déclenchement fixe au sommet, il faut **2·(t_dead + t_settle) + t_sample**.

f_PWM maximale qui garde 2 shunts valides jusqu'à m = 1 :

| T_win | 1,0 µs | 1,5 µs | 2,0 µs | 2,5 µs | 3,0 µs |
|---|---|---|---|---|---|
| f_PWM max | 67 kHz | 44,7 kHz | 33,5 kHz | 26,8 kHz | 22,3 kHz |

T_win dépend du **matériel** (temps d'établissement de l'ampli de shunt après le front, ringing, dead-time). Il doit être **mesuré à l'oscilloscope** [V]. Exemple typique [E] : 0,3 µs de dead-time + 1,2 µs d'établissement + 0,7 µs d'échantillonnage ≈ 2,2 µs.

## 4.5 Stratégie de mesure retenue

1. **Déclenchement ADC adaptatif :** TIM1_CH4 (CCR4 préchargé) déclenche les injectés d'ADC1/2/3 en simultané. À chaque période, CCR4 est placé au plus tard possible dans la fenêtre la plus étroite des deux phases retenues : `t_trig = fin_fenêtre − t_sample − marge`. Toutes les fenêtres sont centrées au sommet et imbriquées, donc aucun front d'une autre phase ne tombe dans l'intervalle de mesure.
2. **Sélection de phase :** les deux phases de plus faible d_x de la période **dont on mesure** (la PWM appliquée pendant l'échantillonnage, pas la suivante).
3. **Validité :** chaque échantillon porte un drapeau `valid` calculé à partir de la fenêtre réelle, comparée à `T_win_min` (paramètre matériel). **Une mesure invalide n'entre jamais dans la régulation.**
4. **Repli par niveaux :**
   - a) 2 phases valides → mesure nominale.
   - b) moins de 2 phases valides pendant un tick (surmodulation, transitoire) → **courant prédit** par le modèle discret du moteur (R, Ld, Lq, ψ, V appliquées). Drapeau `predicted`. Le PI gèle son intégrateur.
   - c) plus de N ticks prédits consécutifs (par ex. 3) → la limitation de modulation est abaissée automatiquement (m_max -= Δ) jusqu'au retour des fenêtres. Événement journalisé.
   - d) prédiction prolongée impossible → réduction de couple puis arrêt contrôlé (doc 7).
5. **Limite de modulation :** m_max est calculé au démarrage à partir de f_PWM et T_win (tableau ci-dessus) et **n'est jamais dépassé hors du mode surmodulation explicite**.
6. **Rangs injectés suivants :** rang 2 = tensions de phase (Va, Vb, Vc) pour la reprise au vol et l'estimation de la tension réellement appliquée ; rang 3 = Vbus ; rang 4 = température (alternée) ou référence interne.

### Calibration des offsets

- Au démarrage, pont désactivé : moyenne de 1024 échantillons. On rejette si |offset − mid| > seuil (défauts 15–17).
- Pendant le fonctionnement : suivi lent de l'offset pendant les périodes où le courant est connu comme nul (pont OFF, moteur arrêté).

## 4.6 Choix de la fréquence PWM / FOC

Critères (les valeurs viennent du script ; ce sont des **bornes**) :

| Critère | Effet de f_PWM ↑ | Ordre de grandeur |
|---|---|---|
| Ondulation de courant ΔI_pp ≤ Vdc/(4·L·f) | ↓ | 72 V, 10 µH : 90 A à 20 kHz, 45 A à 40 kHz |
| Rapport f_PWM/f_e (stabilité, discrétisation) | ↑ | viser ≥ 10–15 ; ≥ 8 avec compensation du retard [E] |
| Pertes de commutation ∝ f·Vdc·I·(t_r + t_f) | ↑ | dépend des MOSFET et du driver [V] |
| Fenêtre ADC (§4.4) | ↓ | 2,5 µs → ≤ 26,8 kHz pour m = 1 |
| Charge CPU | ↑ | 25 kHz × 8 µs = 20 % [E] |

**Recommandation (révisée, doc 11) :** la valeur par défaut est **propre à chaque carte** (`DEF_F_PWM_HZ`). Pour les cartes VESC, elle vaut `foc_f_zv / 2`, soit **15 kHz** pour la plupart, y compris l'Ubox 85_200, car leur dimensionnement thermique a été validé à cette fréquence [C]. La plage est configurable de `HWLIM_F_PWM_MIN_HZ` à `HWLIM_F_PWM_MAX_HZ` (8–40 kHz). À 15 kHz, la fenêtre 2 shunts à m = 1 vaut 4,47 µs. Monter vers 20–22 kHz réduit l'ondulation et la fenêtre aveugle de protection (doc 07 §7.8), au prix de pertes de commutation à mesurer. 25 kHz reste la cible pour les moteurs à faible L si T_win ≤ 2,7 µs. Au démarrage, le firmware vérifie :
- `f_PWM ≤ f_max(T_win, m_max)` ;
- `f_PWM / f_e_max ≥ 10`, avec f_e_max déduit de la limite d'ERPM ;
- `ΔI_pp(Vbus_max, L_min) ≤ ΔI_pp_admissible`.

Si une condition échoue, la configuration est **refusée** avec un message explicite.

Pour les moteurs à très faible L (< 10 µH) : monter f_PWM n'est possible qu'avec des shunts de phase ou un T_win très court. C'est l'argument principal pour la variante « shunts de phase ».

## 4.7 Variante shunts de phase

**54 cartes VESC** ont déjà des shunts de phase en ligne (`HW_HAS_PHASE_SHUNTS`, doc 11). L'implémentation `cs_inline` n'est donc pas « future » : elle est nécessaire dès qu'on vise ces cartes. La carte de test Ubox 85_200 est en 3 shunts **low-side** : `cs_lowside3` reste la priorité.


- Mesure continue, indépendante du rapport cyclique : S2/U2 possible, m jusqu'à la surmodulation, détection de courts-circuits phase–masse.
- Ampli à fort rejet de mode commun obligatoire (type INA240 [E]).
- Abstraction prévue : `current_sense_t` avec une implémentation `cs_lowside3` et une implémentation `cs_inline`. La FOC ne voit que `{ia, ib, ic, valid_mask, t_sample}`.
