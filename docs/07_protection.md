# 7. Protection surintensité absolue et détection de perte de synchronisation

> Cette fonction a **priorité sur toutes les autres**. Rien dans ce document ne fixe de seuil ou de délai numérique universel. Les valeurs dépendent des composants réels (doc 8). Les nombres donnés sont des **exemples de méthode**.

## 7.1 Constats techniques préalables

1. **Le STM32F405 n'a pas de comparateur analogique** [C]. Le watchdog analogique de l'ADC ne déclenche qu'une interruption : il dépend du CPU, de l'horloge ADC et de la séquence de conversion. **Ce n'est pas une protection absolue.**
2. **Les shunts low-side sont aveugles quand le high-side conduit** [C, topologie] :
   - un court-circuit phase–masse ou phase–châssis, avec le high-side ON, ne traverse **aucun** shunt ;
   - en PWM centrée, chaque phase repasse en low-side une fois par période (sauf à d = 100 %). La latence de détection par shunt peut donc atteindre **≈ d·T_s ≈ 35 µs** à 25 kHz ;
   - une perte de synchronisation à haut régime (vecteurs actifs longs) laisse le courant croître pendant les intervalles high-side.
3. **Ordre de grandeur de di/dt** [E, méthode] : `di/dt = V/L_boucle`.
   - Moteur, défaut d'angle : 72 V / 10 µH = 7 A/µs → +250 A en 35 µs.
   - Court-circuit franc aux bornes, avec seulement l'inductance des câbles (≈ 0,3–1 µH) : 72 V / 0,5 µH = **144 A/µs**. 1 µs de détection = +144 A.
   - **Conséquence :** une protection basée sur les shunts n'est pas suffisante contre le court-circuit franc. Il faut une détection **au niveau des MOSFET** (V_DS / désaturation), avec un temps de réaction de l'ordre de la µs.
4. **Le break de TIM1 est asynchrone et matériel** [C] : BKIN actif → MOE = 0 sans intervention du CPU. Les sorties passent à leur état de repos (OISx/OISxN) selon OSSI. MOE reste à 0 tant que le logiciel ne le remet pas à 1 (AOE = 0) : c'est le **verrouillage du défaut**. Pas de filtre numérique sur F4 → filtre RC externe dimensionné contre le bruit de commutation.

## 7.2 Architecture de protection en couches (recommandée)

```
        ┌──────────── Niveau 0 : gate driver (sans MCU) ────────────┐
MOSFET ─┤ V_DS monitoring / DESAT par transistor, blanking ~0,5–3 µs ├─► coupure locale du driver
        └────────────────────────────┬───────────────────────────────┘
                                     │ nFAULT (open-drain)
Shunts ─► comparateurs fenêtre (±I_trip) par phase ─┐
Bus    ─► comparateur courant DC (shunt ou Hall) ───┤
Vbus   ─► comparateur OV matériel ──────────────────┼─► OU câblé ─► RC ─► TIM1_BKIN (niveau 1, µC, sans CPU)
Moniteur d'échéance (TIM9 one-pulse) ───────────────┤               │
                                                    └─► bascule ─► EN/INH du driver (niveau 1bis, sans µC)
                                                          ▲ RAZ uniquement par GPIO µC + absence de défaut
ISR FOC : contrôle logiciel |i| > I_soft, Vbus, cohérence (niveau 2, 1 tick)
FOC : limitation vectorielle (niveau 3, fonctionnement normal)
```

| Niveau | Mécanisme | Couvre | Temps de réaction [E] | Indépendant de |
|---|---|---|---|---|
| 0 | V_DS/DESAT du driver | Court-circuit franc, shoot-through, défaut MOSFET, phase–masse | blanking + 0,1–1 µs (selon le driver) [V] | CPU, firmware, shunts |
| 1 | Comparateurs → BKIN → MOE = 0 | Surintensité de phase (pendant les fenêtres low-side), bus, OV | comparateur 50–500 ns + RC 100–500 ns + chaîne logique ≈ 0,3–1 µs [V] | CPU, firmware |
| 1bis | Même signal → bascule → EN du driver | Idem, et **défaillance du timer ou de la config** | idem | MCU entier |
| 1ter | Courant bus DC (shunt de bus ou capteur Hall) | **Comble l'angle mort high-side** des shunts low-side | idem | CPU |
| 2 | Logiciel, dans l'ISR FOC | |i|, Vbus, offsets, incohérences | 1 tick (40 µs) | — |
| 3 | Limitation FOC | Fonctionnement normal | boucle de courant | — |

**Recommandation sur le matériel :** si le PCB actuel n'a ni V_DS/DESAT ni comparateurs câblés sur BKIN, **la protection absolue n'existe pas** et le firmware ne peut pas la créer. C'est le point bloquant n°1 de la doc 8. Un driver triphasé avec surveillance V_DS intégrée (famille « DRV83xx » ou équivalent [E]) ou des drivers à désaturation couvrent le niveau 0.

### Dimensionnement des seuils (méthode, pas de valeur universelle)

```
I_pic_atteint = I_trip + (V_bus_max / L_min_boucle) · (t_comparateur + t_filtre + t_logique + t_coupure_driver + t_turnoff_MOSFET)
Exigence : I_pic_atteint < min( I_DM MOSFET (impulsion, Tj max) , I_saturation du shunt/ampli , I_démagnétisation moteur , tenue des pistes/connecteurs )
Et :      I_trip > I_max_utile + ondulation crête (Vdc/(4·L·f)) + marge de bruit
```
Si les deux contraintes sont incompatibles, il faut réduire les délais (comparateur plus rapide, RC plus faible, détection V_DS) ou revoir le matériel. **Il ne faut pas compenser en logiciel.**

Énergie de coupure : à la coupure, le courant inductif circule dans les diodes et fait remonter Vbus de `ΔV ≈ L·I²/(2·C_bus·V)` [E]. Le surtension d'avalanche locale dépend de l'inductance de la maille de commutation. Il faut **vérifier la tenue en avalanche** ou le dimensionnement des snubbers [V].

## 7.3 État sûr après défaut : il dépend du défaut et de la vitesse

| État | Principe | Avantage | Danger |
|---|---|---|---|
| **Roue libre / Hi-Z** (6 interrupteurs OFF) | MOE = 0, sorties inactives | Valide pour tout défaut de MOSFET (on n'active rien) | Si E_ll_crête > Vbus (haut régime, défluxage) : redressement par les diodes → **freinage incontrôlé et surtension bus** (UCG). Courant inductif restitué au bus |
| **Court-circuit actif (ASC)** (3 low-side ON) | Les phases sont court-circuitées : le courant tend vers I_sc = ψ/Ld | Pas de remontée du bus ; courant borné en régime établi | Transitoire jusqu'à ~2·I_sc ; **interdit** si un low-side ou une phase est en défaut ; couple de freinage ; I_sc doit être < I_max thermique |

**Politique retenue :**
1. **Matériel (BKIN) → toujours Hi-Z** (OISx = OISxN = inactif). C'est le seul état valide sans rien savoir du défaut.
2. **Après analyse logicielle** (ISR break, < 2 µs) : passage en ASC **uniquement si** (a) aucun défaut de pont n'est signalé (V_DS/DESAT, OC phase), (b) |ω̂| > ω_UCG (seuil où la FEM dépasse Vbus), (c) I_sc_estimé = ψ/Ld < I_ASC_max, (d) le BKIN est retombé. L'ASC passe par le mode « force inactive » des canaux OC (le dead-time matériel est conservé) puis MOE = 1. **Jamais par GPIO.**
3. Défaut V_DS/DESAT ou OC → Hi-Z définitif jusqu'à intervention.
4. Surtension bus avec batterie absente → ASC si les conditions le permettent, sinon Hi-Z + hacheur de freinage si présent [V].
5. `LOCK` niveau 1 fige OIS et le dead-time [C]. On choisit donc LOCK niveau 1 : la politique 1 est fixe et le basculement vers l'ASC n'utilise pas OIS.

**Conséquence de conception :** la vitesse max autorisée (paramètre utilisateur) est bornée par la contrainte UCG ou par la capacité ASC (`hardware_limits.h`).

## 7.4 Comportement après défaut

1. Coupure (matérielle) → ISR BRK (prio 0) : lit les GPIO des sources (nFAULT, OC_A/B/C, OV, échéance), fige le **journal pré-défaut** (anneau des 64 à 256 derniers ticks FOC : i_abc, i_dq, v_dq, θ, ω, Vbus, drapeaux), horodate, passe la machine d'état en `FAULT_LATCHED`.
2. Le défaut est **mémorisé en RAM** (section non initialisée, qui survit au reset logiciel) puis copié en flash **à l'arrêt** seulement.
3. **Réarmement** (`FAULT_LATCHED → READY`) uniquement sur demande explicite (commande ou cycle d'alimentation, selon le type) et si : sources de défaut inactives, Vbus dans la plage, températures OK, |ω̂| < ω_rearm (ou reprise au vol validée), offsets ADC revérifiés, consigne nulle depuis au moins t_min. Après N défauts en T secondes → verrouillage jusqu'au cycle d'alimentation.
4. **Jamais de redémarrage automatique** sous consigne non nulle.

## 7.5 Chien de garde : la PWM ne doit pas survivre au CPU

Problème [C] : si le CPU se bloque (boucle infinie dans une ISR, HardFault mal géré, flash figée), **TIM1 continue de générer la dernière PWM**.

| Mesure | Délai de coupure [E] | Remarque |
|---|---|---|
| IWDG | ≥ ~1 ms en pratique (LSI ≈ 32 kHz, imprécis ±50 %) | Reset → GPIO en entrée → les pull-down externes coupent. Trop lent seul contre un décrochage à fort courant |
| **Enable dynamique du driver** : l'ISR FOC bascule une broche à chaque tick ; un circuit (pompe de charge ou monostable redéclenchable) n'active le driver que si le signal alterne | ≈ 2–4 périodes FOC (~100 µs) | Pratique industrielle. Indépendant du MCU. **Recommandé** [V : matériel] |
| Moniteur d'échéance on-chip : TIM9 one-pulse rechargé par l'ISR FOC → sortie vers le OU de BKIN | ≈ 2 périodes | Ne nécessite qu'une broche et une diode/porte. Indépendant de l'exécution CPU, pas de l'horloge |
| HardFault / NMI handler | immédiat | Force MOE = 0 et EN driver = 0 en tout premier (code en RAM) |

## 7.6 Surveillance logicielle rapide (niveau 2, à chaque tick)

- |i_x| > I_soft_trip sur un échantillon **valide** → `TIMx_EGR.BG = 1` (break logiciel, même chemin que le matériel) + défaut.
- Somme ia + ib + ic ≠ 0 (quand les 3 sont valides) → incohérence de mesure (défaut 18).
- Vbus > V_soft_max ou < V_min ; dV/dt anormal.
- Échantillon ADC figé (même valeur N fois avec PWM active), JEOC manquant, OVR.
- Dépassement d'échéance FOC.

## 7.7 Détection de perte de synchronisation (en amont de la surintensité)

Indicateurs calculés à chaque tick (ou dans la SWI de contrôle), normalisés en 0..1 :

| Indicateur | Calcul | Fiable quand | Fausses alarmes possibles |
|---|---|---|---|
| I1. Résidu de prédiction du courant | ‖i_mes(k) − i_pred(k \| k−1)‖ / I_ref, avec le modèle d/q discret | Toujours (avec mesure valide) | Erreurs de paramètres L, dead-time à basse vitesse |
| I2. Résidu de l'observateur | Écart |ψ̂| − ψ_PM (A2/A3), surface de glissement (SMO), innovation (EKF) | Vitesse > seuil | Saturation magnétique, température de l'aimant |
| I3. Cohérence FEM / tension | Vq − R·iq − ω̂·Ld·id comparé à ω̂·ψ | Vitesse moyenne/haute | Erreurs de ψ (température) |
| I4. Vitesse estimée anormale | |dω̂/dt| > a_max physique (J, couple max) ; sauts de θ̂ | Toujours | Charge très légère (J faible) |
| I5. Saturation tension prolongée | Durée en saturation sans défluxage efficace | Haut régime | Bus faible, accélérations |
| I6. Oscillation Id/Iq | Énergie de l'erreur de courant dans une bande (filtre passe-bande) | Toujours | Résonances mécaniques |
| I7. Divergence entre estimateurs | |θ̂_actif − θ̂_ombre| (diversité : flux vs SMO, ou capteur) | Si l'ombre est active | Si les deux ont la même hypothèse fausse (même R) |
| I8. Id incohérent | Id mesuré très loin de Id* alors que Vd n'est pas saturé | Toujours | — |

**Fusion :** score S = Σ wᵢ·Iᵢ, avec les poids **dépendant du régime** (par exemple I2/I3 pondérés à 0 sous 5 % de la vitesse nominale). Une décision n'est prise que si **au moins deux indicateurs indépendants** dépassent leur seuil, ou un indicateur au-delà d'un seuil critique, afin de ne pas tirer de conclusion d'un indicateur isolé (cahier des charges §10).

| Niveau | Condition | Réaction |
|---|---|---|
| 0 | S < s_warn | — |
| 1 Avertissement | S > s_warn pendant t1 | Drapeau, journal, statut |
| 2 Réduction | S > s_red | I_max réduit (rampe), défluxage limité |
| 3 Mode dégradé validé | S > s_deg et un mode de repli validé existe (capteur présent, I/f à basse vitesse) | Bascule vers le mode validé ; sinon niveau 4 |
| 4 Arrêt contrôlé | S > s_stop | iq* → 0 (rampe), puis pont OFF (ASC/Hi-Z selon §7.3) |
| 5 Coupure | Surintensité logicielle ou matérielle | Break immédiat |

Les seuils et poids seront **appris au banc** (doc 10, phase 8) à partir de décrochages provoqués à faible courant : erreur d'angle injectée, saut de R/L faussé, coupure de mesure.

## 7.8 Réalité des cartes VESC et de la carte de test (Ubox 85_200)

Le recensement (doc 11) montre que **108 des 154 cartes compatibles** n'exposent aucune protection indépendante du MCU. C'est le cas de la carte de test : pas d'EN, pas de nFAULT, pas de BKIN, et PB12 est prise par l'IMU. VESC fonctionne aussi sur ces cartes avec une protection **purement logicielle**. Le firmware fait mieux que VESC sans modifier le matériel, mais **ne peut pas** atteindre l'exigence §9 (coupure sans CPU).

### Ce que le firmware met en place selon `BOARD_HW_OCP`

| `BOARD_HW_OCP` | Mécanisme firmware | Latence [E] | Indépendant du CPU |
|---|---|---|---|
| 3 (BKIN) | break matériel, état Hi-Z, puis analyse | < 1 µs + filtre RC | **oui** |
| 2 (nFAULT GPIO) | EXTI à priorité 0 → `TIM1->EGR = BG` (break logiciel) | ≈ 0,1–0,3 µs après le front + délai du driver | non (CPU vivant requis) ; le driver coupe souvent lui-même [V] |
| 0 / 1 | watchdog analogique ADC sur le groupe injecté → IRQ priorité 0 → `BG` ; contrôle à chaque échantillon dans l'ISR FOC ; détecteur de synchro (§7.7) | ≤ 1 période PWM (66 µs à 15 kHz) + ≈ 2 µs | non |

### Ordre de grandeur du risque sur l'Ubox (exemple, pas une spécification)

Avec 72 V, L = 10 µH et une perte de synchronisation à haut duty : di/dt ≈ 7 A/µs. 66 µs sans échantillon valide représentent jusqu'à **≈ 470 A** de croissance, au-delà des 420 A de coupure et proche de la pleine échelle de mesure (516 A). Conséquences pour cette carte :

1. **Essais de montée en puissance prudents :** courant limité à 25 % des `HWLIM_*` tant que la carte n'est pas `VALIDATED` (politique de `hw_config.h`), alimentation de labo limitée en courant pour les premières phases.
2. **Le détecteur de perte de synchronisation (§7.7) est critique ici :** c'est la seule barrière qui agit *avant* la surintensité.
3. Fréquence PWM plus élevée (20–22 kHz) = fenêtre aveugle plus courte. C'est un compromis avec les pertes de commutation, à évaluer en phase 9.

### Options matérielles recommandées (décision à prendre par l'utilisateur)

| Option | Effort | Gain |
|---|---|---|
| A. Comparateur fenêtre sur les 3 sorties d'ampli de shunt → **PB12 en TIM1_BKIN** (abandon de l'IMU) | petite carte fille + câblage | coupure < 1 µs sans CPU, mais **pendant les fenêtres low-side seulement** |
| B. Option A + capteur de courant de bus DC (Hall) dans le OU câblé | moyen | couvre aussi la conduction high-side |
| C. Vérifier si les drivers discrets de l'Ubox ont une désaturation interne (référence des composants) | aucun | savoir si un niveau 0 existe déjà [V] |
