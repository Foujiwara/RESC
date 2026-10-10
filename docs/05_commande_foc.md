# 5. Commande vectorielle (FOC)

## 5.1 Chaîne de commande (vue d'ensemble)

```
consigne (couple | courant | vitesse | position | duty)
   → command_arbiter (limites, rampes, timeout)
   → [boucle position] → [boucle vitesse]  (SWI contrôle, 1–10 kHz)
   → générateur de références id*/iq* : MTPA ⊕ défluxage ⊕ limite vectorielle ⊕ limites bus/thermiques
   → régulateur de courant d/q (ISR FOC) + découplage + anticipation FEM + anti-windup
   → limitation du vecteur tension (cercle ou hexagone) → Park⁻¹ avec angle prédit
   → compensation onduleur (dead-time, chutes) → SVPWM → CCR
```

Toutes les grandeurs sont en **unités SI (float)** dans le cœur. Il n'y a pas de per-unit implicite. Le per-unit est réservé aux paramètres de réglage adimensionnels.

## 5.2 Régulateurs de courant — comparaison

| Méthode | Dynamique | Robustesse aux erreurs R/L | Comportement en saturation | Coût | Réglage | Tag |
|---|---|---|---|---|---|---|
| PI d/q seul | Bonne à basse f_e, couplage croisé visible au-delà de ~ f_PWM/20 | Très bonne | Bon avec anti-windup | ~40 cycles | 2 paramètres par axe | [C] |
| **PI d/q + découplage ωL + anticipation FEM ωψ** | Très bonne jusqu'à haute f_e | Bonne (le découplage dépend de L ; une erreur de 30 % reste tolérable) | Bon | ~60 cycles | Placement de pôles direct : Kp = L·ω_c, Ki = R·ω_c | [C] |
| PI à « vecteur complexe » (Briz del Blanco 2000) | Pôle du modèle compensé exactement, invariant en ω_e | Bonne | Bon | ~80 cycles | Idem | [C] littérature |
| Deadbeat prédictif | Atteint la consigne en 1–2 périodes | **Faible** : très sensible à L et au retard ; amplifie le bruit | Délicat | ~100 cycles | Peu de paramètres mais pas de marge | [C] |
| MPC à ensemble fini (FCS-MPC) | Très rapide | Moyenne | Naturelle | **Trop coûteux** à 25 kHz sur M4 pour un horizon > 1 [E] | Fonction de coût | [E] |
| MPC continu (CCS) / explicite | Très bonne | Moyenne | Contraintes natives | Élevé (tables) | Complexe | [E] |
| Retour d'état + intégrateur (LQR) | Optimale selon le modèle | Bonne | Anti-windup à construire | ~80 cycles | Matrices de pondération | [C] |

**Choix recommandé : PI en repère d/q discrétisé de façon exacte, avec découplage et anticipation FEM**, plus le terme de compensation de rotation dans le régulateur (forme « complex vector » de Briz). Cette forme corrige la dégradation connue du PI découplé à haute f_e/f_PWM.

- Réglage par défaut automatique : ω_c = 2π·f_PWM/20 (limité par le retard total ≈ 1,5 T_s), Kp = L·ω_c, Ki = R·ω_c [E, à affiner par mesure de réponse indicielle].
- **Alternative implémentée derrière la même interface :** deadbeat avec observateur de Luenberger du courant, pour les moteurs à faible L. Mode expérimental, interdit au-dessus d'un courant de test.

## 5.3 Saturation, anti-windup et priorité d/q

1. Tension disponible : `V_max = m_max · Vbus / √3` (cercle inscrit à l'hexagone). En surmodulation explicite, on utilise l'hexagone.
2. **Priorité d/q** : on satisfait d'abord Vd (nécessaire au défluxage et à la stabilité), puis Vq dans la tension restante : `Vq_max = √(V_max² − Vd²)`. Option : priorité proportionnelle (garder la direction du vecteur).
3. **Anti-windup par back-calculation** : intégrateur += Ki·e + Kaw·(V_sat − V_cmd). Kaw = 1/Kp par défaut. C'est meilleur que le simple « clamping », qui laisse une erreur résiduelle au retour de la saturation.
4. Indicateur de saturation tension (durée, profondeur) transmis au défluxage et au détecteur de perte de synchro (doc 7).

## 5.4 Limitation vectorielle du courant

`id² + iq² ≤ I_max²`, avec I_max = min(utilisateur, matériel, déclassement thermique, limite d'observateur).
Pendant le défluxage, id est prioritaire, et iq est limité à `√(I_max² − id²)`.
Limites séparées **moteur / freinage / batterie** (courant d'entrée P/Vbus, estimé à partir de Vd·id + Vq·iq).

## 5.5 Régénération et bus

- Limite de courant batterie en charge : `I_bat_regen_max`.
- Surveillance de Vbus : rampe de réduction de iq de freinage entre `V_regen_start` et `V_max_user`, puis défaut OV matériel au-delà (doc 7).
- Cas critique : batterie déconnectée (BMS ouvert) pendant un freinage → la tension monte en quelques ms. La limitation logicielle doit agir à chaque tick FOC, et la protection OV matérielle reste l'ultime barrière.

## 5.6 Modes de commande

| Mode | Boucle | Notes |
|---|---|---|
| Couple / courant | iq* (et id* par MTPA) | Mode principal VESC (SET_CURRENT) |
| Freinage | iq* opposé à ω, borné ; jamais d'inversion de sens | SET_CURRENT_BRAKE |
| Vitesse | PI + anticipation d'accélération, filtre de consigne | SET_RPM |
| Position | P/PD en cascade sur la vitesse | SET_POS, nécessite un capteur |
| Duty | Consigne de |V|/V_max, **toujours sous limites de courant** (la boucle de courant reste active) | SET_DUTY : contrairement à une commande en tension pure, la sécurité en courant est conservée |

## 5.7 MTPA, défluxage, haut régime (modules indépendants)

| Module | Méthode retenue | Alternatives | Notes |
|---|---|---|---|
| MTPA | Formule analytique `id = (ψ − √(ψ² + 8(Lq−Ld)²iq²)) / (4(Lq−Ld))`, puis table 2D (I, ψ) si Ld/Lq saturent | Recherche en ligne (injection de signal), table identifiée au banc | SPM (Ld ≈ Lq) : id = 0. Module désactivé automatiquement |
| Défluxage | **Boucle de tension** : le PI agit sur `V_max − |V_cmd|` et produit id < 0. Robuste aux erreurs de paramètres | Feed-forward par modèle (rapide mais sensible à ψ, L) ; hybride FF + boucle | Combinaison FF + correction recommandée en version 2 |
| Ld/Lq(i) | Tables 1D/2D identifiées hors ligne, interpolation bilinéaire | Modèle analytique de saturation | Utilisé par MTPA, découplage et observateur |
| Surmodulation | Désactivée par défaut. Mode 1 (Bolognani) si les fenêtres ADC le permettent | Six-step pour la vitesse max (mesure impossible : courant prédit uniquement) | Très couplé à la mesure 3 shunts → **phase 10** |
| Limite de tension bus | V_max filtré sur Vbus mesuré à chaque tick (pas de filtre lent → sinon dépassement transitoire) | — | — |
| Transitions | Hystérésis et rampes sur id* ; continuité de l'intégrateur | — | — |

**Point de sécurité défluxage :** à haut régime avec id < 0, une perte de commande (coupure du pont) rend la FEM non compensée. Si E_ll_crête > Vbus, les diodes redressent et un **courant de freinage incontrôlé** apparaît (UCG, Uncontrolled Generator Mode). La vitesse maximale autorisée doit être bornée par ce critère, ou l'état sûr doit être un court-circuit actif (doc 7).

## 5.8 Dead-time et non-linéarités de l'onduleur

Distinction (exigée par le cahier des charges) :

| Élément | Nature | Où il est réglé |
|---|---|---|
| Dead-time matériel (DTG) | Garantit l'absence de conduction croisée. **Jamais réduit par le logiciel** | `hardware_config.h`, verrouillé par `LOCK` niveau ≥ 1 |
| Délais du driver (t_d_on, t_d_off) et du MOSFET (t_r, t_f, plateau Miller) | Physiques, dépendants de I, Vbus, T | Paramètres de compensation (mesurés) |
| Erreur de tension résultante | ΔV ≈ sign(i)·Vbus·(t_dead + t_on − t_off)/T_s + chute V_DS/diode | Compensée en logiciel |

L'effet Miller allonge le plateau de commutation, surtout à fort courant et haute tension : il réduit la tension effective et dépend du sens du courant.

**Compensation — comparaison :**

| Méthode | Principe | Précision | Risque |
|---|---|---|---|
| Signe du courant mesuré | ΔV·sign(i_x) | Bonne hors passage par zéro | Bruit au voisinage de i = 0 → oscillations |
| **Signe du courant de référence (ou estimé), avec zone linéaire** | ΔV·sat(i_x*/I_band) | Bonne, lisse | Robuste. **Choix par défaut** |
| Mesure des tensions de phase (rang 2 ADC) | Tension réellement appliquée | Excellente pour l'observateur | Bande passante du diviseur, retard |
| Observateur de perturbation (DOB) | Estime ΔV globalement | Bonne | Interaction avec la boucle de courant |
| Table identifiée ΔV(i, Vbus) | Calibrée au banc | Très bonne | Nécessite une procédure |

**Deux usages distincts :**
1. **Commande** : on ajoute ΔV à la consigne de tension (il faut éviter la double correction avec l'intégrateur PI).
2. **Observateur** : on lui fournit la tension *réellement appliquée* = consigne − erreur estimée, ou la mesure des tensions de phase si elle est disponible et validée. C'est **critique à basse vitesse**, où l'erreur de dead-time est du même ordre que la FEM.

Une fois la compensation active, elle est validée par la symétrie de Vd à id = 0 dans les deux sens de rotation.
