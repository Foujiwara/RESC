# 6. Observateurs sensorless

## 6.1 Principe d'architecture : observateurs interchangeables

On ne fige aucun observateur. Le cœur FOC consomme une **estimation d'angle et de vitesse avec une confiance**, quelle que soit sa source (observateur sensorless, codeur, halls, HFI, fusion).

```c
/* Entrée commune, construite une fois par tick FOC */
typedef struct {
    float i_ab[2];        /* courants αβ mesurés ou prédits [A] */
    float v_ab[2];        /* tension αβ réellement appliquée (compensée onduleur) [V] */
    float vbus, dt;       /* [V], [s] */
    uint32_t meas_flags;  /* VALID / PREDICTED / SATURATED ... */
    const motor_params_t *mp;  /* R, Ld, Lq, ψ (tables éventuelles), mesurées/identifiées */
} obs_input_t;

typedef struct {
    float theta;          /* angle électrique [rad] à l'instant d'échantillonnage */
    float omega;          /* vitesse électrique [rad/s] */
    float confidence;     /* 0..1, propre à l'observateur */
    float residual;       /* résidu normalisé (erreur de prédiction / de contrainte) */
    float flux_mag;       /* |ψ| estimé (si applicable) */
    uint32_t flags;       /* CONVERGED, LOW_SPEED, DIVERGING ... */
} obs_output_t;

typedef struct {
    const char *name;
    uint32_t    caps;     /* SALIENT_OK, STANDSTILL_OK, NEEDS_INJECTION, ... */
    size_t      state_size;
    void (*init)(void *st, const motor_params_t *mp);
    void (*reset)(void *st, float theta0, float omega0);   /* pour reprise / transition */
    void (*update)(void *st, const obs_input_t *in, obs_output_t *out); /* .ramfunc */
    void (*inject)(void *st, float v_inj_dq[2]);           /* NULL si pas d'injection */
    uint32_t    wcet_cycles; /* pire cas mesuré, utilisé par l'ordonnanceur d'ombre */
} observer_vtbl_t;
```

- **Actif** : un seul observateur pilote la commande. Il est choisi par configuration, et le choix n'est appliqué **qu'à l'arrêt**.
- **Ombre** : 0 à N observateurs supplémentaires, alimentés par **la même entrée**. Leurs sorties vont uniquement dans le journal et le détecteur de divergence (doc 7). Ils n'ont **aucune voie d'écriture** vers la commande (séparation par type : pas de pointeur vers l'état de commande). Budget CPU contrôlé : on additionne les `wcet_cycles` et on désactive l'ombre si le budget est dépassé.
- **Transition en marche** : **interdite** dans la version 1. Elle sera prévue plus tard via `reset(theta, omega)` et une fusion pondérée, **après** démonstration de la continuité au banc (cahier des charges §8).

## 6.2 Comparaison des méthodes

Notation : ★ (faible) à ★★★★★ (excellent). Les coûts en cycles sont des estimations M4F pour une implémentation optimisée en float [E]. Les performances relatives viennent de la littérature (références §6.5) et doivent être **confirmées par nos essais** avec codeur de référence [V].

| Méthode | Précision angulaire (vitesse moyenne/haute) | Basse vitesse | Haute f_e | Robustesse R | Robustesse L / saillance | Bruit | Coût [E] | Réglage | Resynchronisation |
|---|---|---|---|---|---|---|---|---|---|
| **A1. Flux, intégrateur pur + passe-haut** | ★★ (déphasage du filtre) | ★ (dérive) | ★★★★ | ★★ | ★★★ | ★★★ | ~60 c | Simple | ★★ |
| **A2. Flux non linéaire à contrainte de module (Ortega/Lee 2010)** | ★★★★ | ★★★ (jusqu'à ~2–5 % de la vitesse nominale [E]) | ★★★★★ | ★★★ (R biaise l'angle à basse vitesse) | ★★★ SPM ; dégradé si Ld ≠ Lq | ★★★★ | ~150 c | 1 gain (γ) | ★★★★ (convergence globale démontrée pour SPM) |
| **A3. Flux actif (Boldea 2008) + correction non linéaire** | ★★★★ | ★★★ | ★★★★★ | ★★★ | **★★★★ IPM et SPM** (le flux actif est aligné sur d quelle que soit la saillance) | ★★★★ | ~180 c | 1–2 gains | ★★★★ |
| A4. Flux hybride tension/courant (Jansen–Lorenz, Gopinath) | ★★★★ | ★★★★ (le modèle courant prend le relais) | ★★★★ | ★★★★ à basse vitesse | ★★★ (dépend de L via le modèle courant) | ★★★★ | ~200 c | Fréquence de croisement | ★★★ (le modèle courant dépend de θ̂) |
| **B1. FEM αβ (observateur de courant + filtre)** | ★★★ | ★★ (FEM ∝ ω) | ★★★ (déphasage du filtre ∝ ω) | ★★★ | ★★ (FEM étendue nécessaire si saillant) | ★★★ | ~100 c | Filtres à adapter à ω | ★★★ |
| B2. FEM étendue d/q (Chen 2003, Morimoto) | ★★★★ | ★★ | ★★★★ | ★★★ | **★★★★** (gère Ld ≠ Lq) | ★★★ | ~150 c | Gains de l'observateur | ★★ (le repère d/q estimé doit rester accroché) |
| **C. SMO (sigmoïde, gain adaptatif, LPF compensé)** | ★★★ | ★★ | ★★★ (déphasage du LPF compensé par table) | **★★★★** | ★★★ | ★★ (chattering) | ~120 c | Gain de glissement + LPF | ★★★ |
| C2. SMO super-twisting (2e ordre) | ★★★★ | ★★ | ★★★★ | ★★★★ | ★★★ | ★★★ (sans LPF) | ~180 c | 2 gains | ★★★ |
| **D. EKF 4 états (iα, iβ, ω, θ)** | ★★★★ | ★★★ | ★★★ (linéarisation + coût) | ★★★ (R estimable en 5e état) | ★★★ | **★★★★★** (filtrage optimal) | **1 500–4 000 c** (≈ 10–25 µs) | Q/R peu intuitifs | ★★ (sensible à l'initialisation) |
| E. Luenberger (courant/FEM) | ★★★ | ★★ | ★★★ | ★★★ | ★★★ | ★★★ | ~100 c | Placement de pôles | ★★★ |
| H1. MRAS (vitesse ou R) | ★★★ | ★★ | ★★★ | ★★★★ si adaptation de R | ★★★ | ★★★ | ~150 c | Adaptation PI | ★★ |
| **H2. HFI signal carré (à f_PWM/2)** | ★★★ (bruit acoustique) | **★★★★★ (y compris à l'arrêt)** | ✗ (désactivé au-delà de ~10 % de la vitesse) | ★★★★★ | Nécessite une saillance Lq/Ld > ~1,1–1,2 [E] ; SPM : souvent insuffisant | ★★ | ~100 c + perte de tension | Amplitude d'injection | ★★★ (ambiguïté N/S → détection de polarité) |
| H3. HFI sinus / pulsée d | ★★★ | ★★★★ | ✗ | ★★★★★ | Idem | ★★★ | ~150 c (démodulation) | Filtres | ★★★ |
| H4. Observateur pour faible L (flux avec L·i plutôt que L·di/dt) | Remarque : A2/A3 utilisent L·i et non di/dt. **Avantage structurel** sur les méthodes FEM pour les moteurs à faible L, où di/dt est bruité par l'ondulation | | | | | | | | |

### Estimation de vitesse et d'angle : boucle de poursuite

| Méthode | Retard en régime établi | Rejet du bruit | Remarque |
|---|---|---|---|
| atan2 direct | nul (en angle) | faible ; ω par dérivée = bruitée | ~50 c |
| **PLL de type 2 (PI) sur le vecteur normalisé** (erreur = sin(θ−θ̂)) | 0 à vitesse constante, erreur en accélération | Bonne | ~40 c. **Choix par défaut** |
| PLL de type 3 / observateur mécanique avec anticipation Te/J | 0 en accélération constante | Très bonne | Nécessite J et ψ ; très utile en forte dynamique (traction) |
| PLL en quadrature normalisée (amplitude) | Gain indépendant de |ψ|·ω | Bonne | Normaliser par |ψ̂| stabilise la bande passante sur toute la plage |

## 6.3 Analyse des points clés

**Dérive de l'observateur de flux (A).** L'intégrale de `v − R·i` accumule les offsets ADC et les erreurs de dead-time. A1 coupe la dérive par un passe-haut, mais introduit un déphasage important à basse vitesse. A2/A3 imposent `|ψ_est| = ψ_PM` : la correction est radiale et n'introduit **pas de déphasage** en régime établi. C'est la raison du choix.

**Effet de R à basse vitesse.** L'erreur d'angle vaut approximativement `Δθ ≈ ΔR·i_q / (ω·ψ)` [E, petit signal]. Elle diverge quand ω → 0. Exemple : ΔR = 20 % de R = 10 mΩ, iq = 100 A, ψ = 10 mWb → 2 mΩ·100 A/(ω·0,01) = 20/ω rad. À ω = 200 rad/s (≈ 32 Hz électrique), on obtient 0,1 rad ≈ 6°. **Conclusion :** une estimation thermique de R (modèle thermique ou MRAS sur R) est indispensable pour du fort couple à basse vitesse.

**Saillance (IPM).** A2 suppose Ld = Lq. Sur un IPM, l'angle estimé est biaisé en charge. A3 (flux actif ψ_a = ψ_s − Lq·i) et B2 (FEM étendue) se ramènent à un modèle non saillant. **On choisit A3 comme formulation générale**, qui dégénère en A2 si Ld = Lq.

**EKF sur M4F.** Pour 4 états et 2 mesures : la propagation `P = F·P·Fᵀ + Q` (≈ 130 MAC en exploitant la structure creuse), le gain (inverse 2×2) et la mise à jour représentent environ 400 opérations. En C générique, avec trigonométrie et accès mémoire, cela fait 1 500 à 4 000 cycles [E]. **C'est faisable à 25 kHz** (25 µs → ~40 % de charge), mais coûteux. Rôle retenu : **observateur de référence en mode ombre** et cas d'usage à fort bruit, pas l'observateur par défaut.

**SMO.** Robuste aux paramètres, mais le chattering impose un passe-bas dont le déphasage dépend de ω (à compenser par table). Le gain doit dépasser |FEM|, donc il est adaptatif avec ω. Bon candidat secondaire, très économique. La version super-twisting supprime le passe-bas.

**HFI.** C'est la seule voie fiable à l'arrêt et à très basse vitesse **si le moteur est saillant**. Sur les moteurs SPM haute puissance (hub, outrunner), la saillance est souvent faible : il faut la **mesurer** (identification Ld/Lq à l'arrêt) avant de décider [V]. Une détection de polarité (impulsions d'axe d, saturation) est obligatoire.

## 6.4 Choix recommandé

| Rôle | Méthode | Justification |
|---|---|---|
| **Observateur principal (v1)** | **A3 : flux actif non linéaire à contrainte de module + PLL de type 2 normalisée** | Valide en SPM et IPM, sans déphasage, convergence globale (SPM), coût ≈ 200 c avec PLL, 1 gain principal, adapté aux faibles L (pas de di/dt) |
| Démarrage | **I/f** (courant imposé, fréquence en rampe) avec transition par fusion vers l'observateur sur critère de confiance + reprise au vol à partir des tensions de phase | Indépendant de la saillance ; le couple de démarrage est contrôlé en courant |
| Basse vitesse (option) | HFI carré, si la saillance mesurée suffit | Commutation avec hystérésis sur la vitesse, **après validation de la continuité** |
| Secondaire / ombre | SMO super-twisting, EKF | Diversité de principe pour la détection de divergence (doc 7) |
| Adaptation | MRAS sur R (ou estimation thermique) | Réduit l'erreur basse vitesse |
| Avec capteur | Codeur ABI/SPI, halls (avec interpolation) → même interface `observer_vtbl_t` | Le capteur sert de **référence** pour la comparaison et de mode nominal en phase 6 |

**Différence avec VESC :** VESC utilise par défaut un observateur de flux non linéaire (proche de A2) avec PLL. Notre choix A3 est une **généralisation** (flux actif), retenue pour sa validité sur IPM et non par mimétisme. L'architecture permet de comparer objectivement A2, A3, B2, C2 et D sur le même jeu de données.

## 6.5 Protocole d'essais comparatifs [V]

1. Banc : moteur entraîné par un moteur de charge, codeur de référence de 14 bits au moins, aligné électriquement.
2. Pour chaque observateur, en **ombre**, pendant que la commande tourne **sur le codeur** : erreur d'angle (moyenne, écart-type, max), erreur de vitesse, temps de convergence après `reset` à un angle faux de ±90°/180°.
3. Profils : vitesse de 1 % à 100 % ; iq de 0 à I_max ; échelons de couple ; inversions ; R ± 30 % ; L ± 30 % (paramètres volontairement faussés) ; bruit ADC injecté.
4. Rejeu hors ligne : les trames brutes (`i_ab`, `v_ab`, `vbus`, θ_ref) sont journalisées puis rejouées dans la **même** implémentation C compilée pour PC (doc 9 `sim/`) → comparaison bit à bit possible.
5. Critères de sélection : |Δθ| < 5° électriques au-delà de 5 % de la vitesse nominale à I_max, aucune divergence sur 1 h de profil, coût < 25 % du budget FOC [E, seuils à confirmer].

Références principales : Lee, Hong, Nam, Ortega, Praly, Astolfi, *IEEE TPEL* 2010 (observateur de flux non linéaire) ; Boldea, Paicu, Andreescu, *IEEE TPEL* 2008 (flux actif) ; Chen, Tomita, Doki, Okuma, *IEEE TIE* 2003 (FEM étendue) ; Briz del Blanco, Degner, Lorenz, *IEEE TIA* 2000 (régulateur complex vector) ; Jansen & Lorenz, *IEEE TIA* 1995 (saillance/HFI) ; Bolognani et al., *IEEE TIA* 1999 (EKF PMSM).
