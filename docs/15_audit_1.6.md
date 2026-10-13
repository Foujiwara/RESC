# 15. Audit du firmware RESC 1.6 — défauts, irrégularités, plan de correction

Audit du code à la version 1.6 (commit `bfeda5d`), ciblé sur la chaîne temps réel (`foc.c`, `ctrl.c`,
`observer.c`, `motor_hw.c`, `main.c`, `resc_conf.c`) et sur la carte de l'utilisateur (Ubox SINGLE 85_200,
moteur 8 mΩ / 14 µH / 8,9 mWb, 72 V). Tests PC (`build.py --tests`) et simulation (`build.py --sim`) passent
tous à cette version ; l'audit porte donc sur ce que ces tests **ne couvrent pas**.

Le prompt à donner à un modèle plus simple pour corriger et optimiser est en **§15.6**.

---

## 15.1 Verdict général

1. **La cause de la casse n'est pas établie.** La 1.5 a introduit la reprise sur tension mesurée, la 1.6 l'a retirée
   « cause probable » : aucune des deux hypothèses n'est appuyée par une donnée du contrôleur (journal de défaut,
   courbe pré-défaut). Le firmware n'enregistre pas de tampon circulaire pré-défaut en permanence (seul
   l'échantillonneur VESC Tool, s'il a été armé à la main, le fait : `sampler.c:71-86`). Tant que ce manque n'est
   pas comblé, chaque version « corrige » une hypothèse.
2. **La carte n'a aucune protection matérielle indépendante** (`BOARD_HW_OCP 0`, pas de BKIN :
   `board_ubox_100_core.h:57-58`). Toute la protection est logicielle (comparaison échantillon par échantillon +
   watchdog analogique de l'ADC, les deux avec une latence d'une période PWM ≈ 67 µs). Le document 07 §7.1 le dit
   déjà : avec 72 V sur 14 µH, une erreur d'angle de 30° à 40 000 ERPM donne ≈ 1,4 A/µs, soit **+90 A par
   période** au-delà du seuil avant que la coupure n'agisse. La coupure est réglée à 400 A (`DEF_I_ABS_TRIP_A`)
   pour une pleine échelle des shunts de 516 A : il reste 100 A de marge, soit **une seule période de retard**.
   C'est le mécanisme de casse le plus plausible, et il est indépendant de la 1.5.
3. **Le simulateur valide un point de fonctionnement qui n'est pas celui de l'utilisateur** : le scénario « 27c
   reprise à 40 000 ERPM » tourne en réalité à **29 600 ERPM** (`sim_user.inc:24-27`, la boucle n'atteint pas la
   vitesse) et son critère porte sur `iq` (< 4 A d'écart) alors que la **crête de phase** affichée est de 7 à
   9 A pour 2 A demandés. Le modèle est un modèle moyen par période (pas d'ondulation, pas d'établissement
   d'ampli, Vbus idéal : `sim_motor.h:3-5`) : il ne peut pas reproduire une surintensité de reprise réelle.
4. Plusieurs **courses critiques (races)** entre les trois niveaux d'exécution (ISR FOC prio 1, SysTick prio 4,
   boucle principale) ne sont pas protégées. Aucune n'est prouvée responsable de la casse, mais chacune peut
   produire un comportement « qui ne marche pas correctement » de façon non reproductible.
5. Le cœur FOC est **une fonction de 870 lignes** (`foc_isr`, `foc.c:1245-2118`) avec une structure d'état de
   ~150 champs et des `goto`. Il est correct dans l'ensemble, mais invérifiable à la lecture et impossible à tester
   unitairement : c'est la raison pour laquelle les régressions se succèdent (1.1 → 1.3 Hall, 1.4 offsets, 1.5/1.6
   reprise).

---

## 15.2 Défauts de sécurité (à corriger en premier)

| # | Défaut | Où | Effet |
|---|---|---|---|
| S1 | **Seuil de coupure trop proche de la pleine échelle des shunts** : `i_abs_trip = min(l_abs_current_max, 420, 0.9 × 516)` = 400 A par défaut, pour une carte annoncée 200 A continus. Latence de détection = 1 période. | `resc_conf.c:294-297`, `board_ubox_100_core.h:114,140` | À 7 A/µs (doc 07), le courant dépasse la saturation des shunts avant la coupure : le pont n'est protégé que par la chance. |
| S2 | **Pas de détection anticipée de perte de synchronisation.** Le doc 07 §7.7 prévoit un résidu de prédiction de courant (I1) et une divergence entre estimateurs (I7). Rien n'est implémenté : seul `flux_ratio` hors [0.5, 2] pendant 20 ms (`foc.c:1990-2003`) coupe, soit **300 périodes** après le décrochage. | `foc.c:1990-2003` | Une reprise au vol avec un angle faux n'est détectée qu'une fois le courant déjà au seuil absolu. |
| S3 | **Reprise au vol sans validation de l'observateur.** En IDLE, `obs_fast` ne demande que `flux_ratio ∈ [0.7, 1.4]` et `ω > 0.8 st_omega`. L'observateur en roue libre tourne sur les tensions de phase mesurées avec des offsets bornés à ±1 V : à basse FEM il peut être plausible et faux. Rien ne compare l'angle observé à l'angle du vecteur de tension **mesuré** (disponible gratuitement pont coupé). | `foc.c:1505-1511`, `foc.c:1539-1556` | Fermeture de la boucle sur un angle faux ⇒ pointe de courant (le problème des versions 1.3 à 1.6). |
| S4 | **Après un défaut d'offset, le moteur peut tourner sans calibration.** `HIGH_OFFSET_CURR_x` ⇒ FAULT, `calibrated=false`. Un `clear` ramène en IDLE (`foc.c:1478-1483`) ; le chemin de démarrage (`foc.c:1497-1503`) ne teste pas `S.calibrated`. | `foc.c:1299-1302, 1478-1483, 1497-1503` | Démarrage avec des offsets de courant faux ⇒ mesure fausse de plusieurs dizaines d'ampères. |
| S5 | **Une coupure venue d'un autre contexte peut être annulée pendant une période.** `trip_at()` (`foc.c:235-257`) est appelé depuis SysTick (`ctrl.c:518-524`), EXTI et le break TIM1 (`motor_hw.c:235-242`, prio 0 > prio FOC). Si l'ISR FOC est en cours (ou préemptée), elle finit son tour avec `S.br = true` (`foc.c:2005`) et `pwm_write()` **remet MOE à 1** (`motor_hw.c:213-215`). Le gestionnaire de break efface même `BIF` avant d'appeler le défaut, ce qui lève la garde `!(SR & BIF)`. | `motor_hw.c:205-242`, `foc.c:235-257` | Sur une carte à BKIN : le pont est réactivé une période après un défaut **matériel**. Sur l'Ubox : une période de plus après un défaut thermique ou de survitesse. |
| S6 | **Réarmement automatique de la surintensité** : 3 fois par fenêtre glissante de 10 s, puis la fenêtre se remet à zéro (`ctrl.c:772-793`) ⇒ nombre illimité de tentatives à 10 s d'intervalle. Le doc 07 §7.4 demande un verrouillage après N défauts. | `ctrl.c:772-793` | Un défaut récurrent (angle faux à haute vitesse) est rejoué indéfiniment. |
| S7 | **Sur- et sous-tension sur un échantillon brut unique** (`S.vbus`, pas `S.vbus_f`). | `foc.c:1372-1373` | Coupures parasites sous fort courant (bruit de bus) ⇒ roue libre à haute vitesse ⇒ nouvelle reprise. |
| S8 | **Dépassement de l'ISR seulement compté**, jamais traité (`H.overruns++`, `motor_hw.c:442`). Le doc 07 §7.6 demande un défaut sur échéance manquée. | `motor_hw.c:439-445` | Une ISR trop longue (paramètre absurde, ajout de code) dégrade silencieusement la boucle. |
| S9 | **Critère de validité de la fenêtre low-side non vérifié.** `HWLIM_T_WIN_MIN_NS = 3000` est la valeur générique (`board_common_f405.h:286`), jamais mesurée sur l'Ubox (doc 08 U2). Le test `(1 − d) ≥ t_win·f` (`foc.c:1348`) compare la **fenêtre entière** à `t_win`, alors que l'échantillon est pris au **centre** de la fenêtre : le temps d'établissement réellement disponible est la moitié. | `foc.c:1346-1355`, `motor_hw.c:433`, `resc_conf.c:277-284` | Échantillons pris pendant la sonnerie des amplis près de 95 % de modulation (exactement le régime des 40 000 ERPM). |

## 15.3 Courses critiques et bugs de logique

| # | Défaut | Où | Correction |
|---|---|---|---|
| R1 | **Commande déchirée** : `ctrl_update()` écrit 12 champs de `g_foc_cmd` un par un à 1 kHz (`ctrl.c:818-831`) ; l'ISR FOC peut préempter au milieu et voir `iq_ref` neuf avec `run/brake/duty_mode/i_hi/i_lo/dir` anciens. | `ctrl.c:818-831`, `foc.h:163-178` | Publier la commande en un seul coup : double tampon + compteur de séquence (seqlock), l'ISR copie localement si la séquence est paire et identique avant/après. |
| R2 | **Paramètres déchirés** : `foc_set_params()` copie `p_next` **puis** met `p_pending` (`foc.c:340-344`). Si un `p_pending` précédent n'a pas encore été consommé, l'ISR applique une structure à moitié copiée. | `foc.c:340-344, 1247-1250` | Mettre `p_pending = false` avant la copie, barrière, copier, puis `true`. |
| R3 | `command()` dans `ctrl.c` écrit `mode` puis `value` (`ctrl.c:119-127`) depuis la boucle principale (paquets) et depuis SysTick (apps) sans exclusion. | `ctrl.c:119-127` | Même mécanisme que R1 (un seul writer publié par séquence), ou masquer SysTick pendant `command()`. |
| R4 | `released` compare `C.value` à `cc_min_current` (ampères) alors qu'en `CURRENT_REL` la valeur est relative (−1..1). | `ctrl.c:776` | Convertir selon le mode avant la comparaison. |
| R5 | Après `trip()` dans l'étape 4, l'ISR continue à exécuter l'observateur et le reste du tour (pas de `goto output`) ; inoffensif mais coûte des cycles au moment où l'on veut le pont coupé le plus vite possible. | `foc.c:1366-1379` | `goto output` immédiat après un trip. |
| R6 | `set_state()` écrit `S.st`, `st_time`, `telem.state` séparément et est appelé depuis plusieurs contextes (via `trip_at`). | `foc.c:176-181` | Tout défaut hors ISR devient une **requête** (`fault_req`, comme `clear_req`) consommée au début de `foc_isr()` ; seul `foc_hw_kill()` reste immédiat. |

## 15.4 Qualité, performance, tests

| # | Constat | Recommandation |
|---|---|---|
| Q1 | `foc_isr()` : 870 lignes, 9 `goto output`, machine d'états entrelacée avec la régulation. | Découper en fonctions `RAMFUNC static` par étape : `measure()`, `protect()`, `observe()`, `sm_idle()`, `sm_start()`, `sm_run()`, `regulate()`, `publish()`. Refactor mécanique, sans changement de comportement, validé par `--sim` identique à l'octet près sur les traces. |
| Q2 | Simulation : modèle moyen, pas d'ondulation, pas de temps d'établissement, Vbus idéal, divisers de phase sans erreur de gain ; scénario utilisateur à la mauvaise vitesse ; critères sur `iq` et non sur la crête de phase. | Forcer `M.omega_m` à 40 000 ERPM, ajouter un critère sur `g_trip_peak`, ajouter une erreur de gain ±2 % par diviseur de phase et un offset ±1 V, ajouter une impédance de bus (R + C) pour la régénération. |
| Q3 | Aucun test unitaire du cœur hors simulation ; aucun test de rejeu d'un journal réel. | Ajouter un mode « rejeu » : un fichier d'échantillons ADC bruts (sortie de l'échantillonneur VESC Tool) injecté dans `foc_isr()` sur PC. |
| Q4 | Diagnostic : `fault_log` garde code/ligne/tick (`foc.c:245-248`), mais **pas de forme d'onde**. | Tampon circulaire permanent en CCM : 256 derniers ticks de `i_ph[3]`, `id, iq, vd, vq, θ, ω, vbus, state, duty`, gelé par `trip_at()`, lisible par le terminal (`fault_dump`) et conservé en `.noinit` à travers un reset logiciel. |
| Q5 | Performance ISR : 3 à 4 `fm_sincos`, 2 `fm_atan2`, 5 à 6 `fm_sqrt`, une copie de 600 octets à chaque changement de paramètres. Charge non documentée (`terminal hw` l'affiche). | Mesurer sur cible (`hw` : `isr_cycles_max`) et noter la valeur dans le README. Pas d'optimisation à l'aveugle : la marge est probablement large à 15 kHz. |
| Q6 | `-Wshadow` : deux `cs` masqués dans `foc.c` (`foc.c:1259` vs locaux `cs` de sinus). | Renommer. Ajouter `-Wshadow` aux drapeaux. |
| Q7 | Valeurs par défaut de la carte : `DEF_I_ABS_TRIP_A 400`, `HWLIM_I_ABS_TRIP_A 420`, `HWLIM_I_MOTOR_A 300` pour une « 85_200 ». | Ramener `DEF_I_ABS_TRIP_A` à 300 et `HWLIM_I_ABS_TRIP_A` à 350 tant que U1 (doc 08 : référence des MOSFET, I_DM) n'est pas connu. |

## 15.5 Ce qui est correct et ne doit pas être « corrigé »

- Watchdog analogique dans la plage mesurable, réarmé par phase selon la fenêtre (`motor_hw.c:349-364, 429-437`).
- IWDG 50 ms conditionné à l'avance de l'ISR **et** de la boucle principale (`main.c:253-257`), HardFault qui coupe MOE (`startup.c:21-25`).
- Stockage de la configuration par nom avec CRC, copie avant mise à jour, restauration (`resc_conf.c:585-866`).
- Hall repris de `foc_correct_hall` (`angle_sensors.c:20-69`), observateurs 0–6 fidèles à VESC 7.00.
- Anti-windup de la boucle de courant par recalcul (`foc.c:387-406`), priorité à l'axe d.
- Bornage de toutes les grandeurs utilisateur dans `resc_conf_clamp_mc()` et `resc_conf_build()`.

---

## 15.6 Prompt pour un modèle plus simple

Copier tel quel. Il est auto-suffisant et ordonné par priorité ; chaque étape se valide seule.

```text
Tu travailles dans le dépôt RESC (firmware FOC pour cartes VESC, STM32F405, C11). Lis d'abord
docs/15_audit_1.6.md en entier : il décrit les défauts à corriger avec les fichiers et les lignes.
Ne lis pas tout le dépôt ; ouvre seulement les fichiers cités.

RÈGLES
- Une étape = un commit, message en français, qui commence par le numéro de l'étape (ex. "E1 : ...").
- Avant chaque commit : `cd firmware && python3 build.py --tests && python3 build.py --sim` doivent
  afficher "TOUS LES TESTS PASSENT" et "TOUS LES ESSAIS PASSENT". Si un essai de simulation casse,
  comprends pourquoi avant de toucher au test ; n'affaiblis jamais un critère existant.
- Ne change pas le protocole VESC, les structures de configuration (vesc_conf.h), les fichiers XML de
  firmware/conf, ni la présentation RESC Tool.
- Tout code exécuté dans l'ISR FOC est `RAMFUNC`, sans appel à la libm, sans double, sans division par
  une variable qui peut valoir zéro.
- Pas de refactor hors de l'étape en cours. Pas de renommage gratuit. Respecte le style du code voisin
  (commentaires en français sans accents, 120 colonnes).
- Ne touche pas aux binaires de firmware/release.

ÉTAPES, DANS L'ORDRE

E1. Coupure collante (audit S5).
    Dans firmware/src/bsp/motor_hw.c : ajoute `static volatile bool s_killed;`. `foc_hw_kill()`,
    le gestionnaire TIM1_BRK, `exti_fault()` et le watchdog analogique mettent `s_killed = true`
    en plus de MOE = 0. `pwm_write()` ne remet MOE à 1 que si `!s_killed`. Ajoute
    `void motor_hw_rearm(void)` qui remet `s_killed = false` ; appelle-la depuis foc.c uniquement au
    moment où l'état FAULT repasse en IDLE (foc.c, case FOC_ST_FAULT, quand clear_req est consommé).
    Déclare `foc_hw_rearm()` à côté de `foc_hw_kill()` dans foc.h et fournis une version vide dans
    sim/ (cherche où `foc_hw_kill` est défini pour la simulation). Le gestionnaire TIM1_BRK ne doit
    plus effacer BIF avant d'avoir mis s_killed.

E2. Défaut hors ISR = requête (audit R6, S5).
    Dans foc.c : ajoute `volatile uint32_t fault_req;` dans foc_t. `foc_raise_fault(code)` fait
    `foc_hw_kill()` puis `S.fault_req = code` (sans toucher à S.st ni à S.br). Au début de
    `foc_isr()`, après l'application des paramètres : si `S.fault_req`, appelle `trip_at(S.fault_req, 0)`
    et remets `fault_req = 0`. `trip_at()` n'est plus appelé qu'depuis l'ISR. Vérifie que la simulation
    (sim_main.c scénario "Mesure aberrante" et "timeout") passe toujours.

E3. Commande publiée atomiquement (audit R1, R3).
    Dans foc.h : garde `foc_cmd_t` tel quel mais ajoute `volatile uint32_t seq;` en tête. Dans ctrl.c,
    à la fin de ctrl_update(), remplis une `foc_cmd_t` locale puis publie : `g_foc_cmd.seq++` (impair =
    en cours d'écriture), `__DMB()` (macro vide sur PC : ajoute `#ifdef RESC_HOST #define __DMB()` dans
    platform.h), copie des champs, `__DMB()`, `g_foc_cmd.seq++`. Dans foc_isr(), au début, copie
    `g_foc_cmd` dans une `foc_cmd_t cmd` locale tant que `seq` est pair et identique avant/après la
    copie (au plus 3 essais, sinon garde la copie précédente conservée dans S). Remplace ensuite toutes
    les lectures `g_foc_cmd.` de foc.c par `S.cmd.`. Fais la même chose pour `command()` dans ctrl.c :
    écrire mode et value sous un compteur de séquence que ctrl_update() respecte.

E4. Paramètres : ordre de publication (audit R2).
    foc.c `foc_set_params()` : `S.p_pending = false; __DMB(); S.p_next = *p; __DMB(); S.p_pending = true;`.

E5. Calibration obligatoire (audit S4).
    foc.c, case FOC_ST_IDLE : toutes les branches qui mènent à enter_forced / enter_acim_start /
    enter_start / enter_hfi_start / enter_brake_short / set_state(FOC_ST_RUN) exigent `S.calibrated`.
    Quand un défaut HIGH_OFFSET_CURR_x est effacé (clear_req), repasse en FOC_ST_CALIB (remets cal_n et
    cal_acc à zéro) au lieu de IDLE.

E6. Tensions filtrées pour les coupures (audit S7).
    foc.c lignes de OVER_VOLTAGE / UNDER_VOLTAGE : utilise `S.vbus_f` et exige la condition sur 4
    périodes consécutives (compteur uint8_t dans foc_t). Garde S.vbus brut pour la télémétrie.

E7. Trip immédiat (audit R5).
    foc.c étape 4 : après chaque `trip(...)` de l'étape 4 et du bloc `imag`, `goto output`.
    Vérifie que `output:` traite bien le cas S.br == false (c'est déjà le cas).

E8. Échéance de l'ISR (audit S8).
    Ajoute `#define RESC_FAULT_ISR_OVERRUN 34` dans fault_codes.h (hors plage VESC, acceptable) et
    nomme-le dans terminal.c là où les codes sont nommés (cherche "UNDERSPEED" pour trouver la table).
    Dans app_tick_1khz (main.c), lis motor_hw_overruns() : si le compteur augmente de plus de 10 en 1 s,
    appelle `foc_raise_fault(RESC_FAULT_ISR_OVERRUN)`.

E9. Validation de la reprise au vol (audit S3) — la plus importante, à faire avec soin.
    Dans foc.c, en IDLE, calcule à chaque tour pont coupé l'angle du vecteur de tension mesuré
    `theta_v = atan2(vb, va)` (va, vb déjà calculés par applied_voltage) et l'angle de FEM attendu
    `theta_e = obs.theta + sign(obs.omega) * pi/2` (la FEM est en avance de 90 deg sur le flux).
    Filtre l'écart `fm_wrap_pi(theta_v - theta_e)` sur 32 périodes (moyenne de sin et cos séparément,
    puis atan2). Ajoute à `obs_fast` la condition : |écart filtré| < 15 deg ET |va, vb| > 2 V ET
    `S.meas_n >= 64`. Si la condition n'est pas remplie et que la commande demande `run`, ne démarre
    pas : laisse l'observateur se recaler et incrémente `g_foc_telem.obs_resync` (déjà existant) ;
    au-dessus de 0.5 s d'attente, recale l'observateur avec `obs_reset(&S.obs, &S.oc, theta_v - sign*pi/2,
    omega estimé par la dérivée de theta_v)`. Expose l'écart filtré dans g_foc_telem (`resume_err`,
    float) et affiche-le dans `terminal hw`.
    Validation : dans sim/sim_coast.inc le scénario "27b observateur faussé" doit toujours passer, et
    tu ajoutes un essai : observateur décalé de 90 deg par foc_dbg_obs_reset() à 20 000 ERPM, demande
    de 2 A ; la crête de phase (g_trip_peak) doit rester < 15 A et l'état doit passer en RUN en moins
    de 100 ms.

E10. Résidu de prédiction de courant (audit S2).
    foc.c, en FOC_ST_RUN, juste après la Park (étape 7) : prédis id/iq de ce tour à partir du tour
    précédent avec le modèle discret `i(k+1) = i(k) + dt/L * (v(k) - R i(k) - e(k))` sur les axes d, q
    (e_d = -w Lq iq, e_q = w Ld id + w lambda). Résidu r = |i_mes - i_pred| / max(i_abs_trip*0.1, 5 A).
    Filtre r sur 8 périodes. Si r_f > 1.0 pendant 8 périodes consécutives : trip(RESC_FAULT_UNDERSPEED)
    (perte de synchronisation). Ne l'active pas pendant l'injection HFI (S.hfi_on), ni en détection,
    ni pendant brk_short, ni les 20 premières périodes après une entrée en RUN. Expose r_f dans la
    télémétrie (`sync_resid`). Validation : tous les essais --sim passent ; l'essai E9 passe ; ajoute
    un essai qui force foc_dbg_obs_reset(theta + pi, omega) en pleine marche à 20 A : le défaut doit
    tomber en moins de 2 ms (30 périodes) et la crête de phase doit rester < 0.5 * i_abs_trip.

E11. Réarmement de la surintensité (audit S6).
    ctrl.c : après 3 ABS_OVER_CURRENT dans la même fenêtre de 10 s, verrouille (cause_gone = false)
    jusqu'à un `clear` explicite ou un redémarrage ; ne remets plus oc_count à zéro automatiquement
    tant que le verrou est posé. Journalise avec ctrl_log (ajoute un code CLOG_OC_LOCK dans ctrl.h).

E12. Seuils de la carte (audit S1, Q7).
    hw/boards/ubox/board_ubox_100_core.h, bloc 85_200 : DEF_I_ABS_TRIP_A 300, HWLIM_I_ABS_TRIP_A 350.
    Ajoute un commentaire : "a relever quand les MOSFET (I_DM) seront identifies, doc 08 U1".
    resc_conf.c : la marge courant moteur / coupure passe de 0.9 à 0.8.

E13. Fenêtre de mesure (audit S9).
    foc.c ligne du test de validité et motor_hw.c (win_frac) : le critère devient
    `(1 - d) >= 2 * t_win * f_pwm` (l'échantillon est au centre de la fenêtre ; il faut t_win avant
    lui). resc_conf.c : adapte m_win avec le même facteur 2. Vérifie que m_max reste >= 0.3 et que
    l'essai --sim "31. Surmodulation" passe encore ; si la vitesse à vide baisse, c'est attendu :
    ajuste le critère de cet essai avec le nouveau m_max calculé, en l'écrivant dans le commentaire.

E14. Tampon pré-défaut (audit Q4).
    Nouveau fichier firmware/src/core/blackbox.c/.h : tableau CCMBSS de 256 entrées
    {int16 i_ph[3] (A x 10), int16 id, iq, vd, vq (x10), int16 theta (rad x 1e4), int16 omega (rad/s / 4),
    uint16 vbus (V x 100), uint8 state, uint8 duty (x 200)} écrit à chaque foc_isr() (fonction
    RAMFUNC blackbox_push()), gelé par trip_at() (blackbox_freeze()), dégelé par le clear. Commande
    terminal `fault_dump` qui imprime les 256 lignes en CSV. La structure est en section .noinit pour
    survivre à un reset logiciel ; valide-la par un magic + crc simple au démarrage.

E15. Simulation fidèle au cas utilisateur (audit Q2).
    sim/sim_user.inc : force `M.omega_m` pour que sim_omega_e corresponde à 40 000 ERPM avant les
    reprises ; ajoute `check("crete de phase < 12 A", g_trip_peak < 12)` sur chaque reprise ; ajoute
    une erreur de gain de +2 %, -2 %, +1 % sur les trois diviseurs de phase (nouveau champ
    `vph_gain[3]` dans sim_motor_t, défaut 1.0, appliqué dans sim_motor.c là où adc->vph est calculé)
    et un offset de +0.8 V, -0.5 V, +0.3 V. Si les critères échouent après E9/E10, c'est un vrai
    problème : signale-le dans le message de commit et NE modifie PAS les critères.

E16. Découpage de foc_isr (audit Q1) — seulement si E1 à E15 sont terminées et vertes.
    Découpe foc_isr() en fonctions RAMFUNC static sans changer une seule ligne de logique :
    isr_measure(), isr_calibrate(), isr_protect(), isr_observe(), isr_sensors(), isr_state_idle(),
    isr_state_detect(), isr_state_forced(), isr_state_hfi_start(), isr_state_acim_mag(),
    isr_state_start(), isr_state_run(), isr_regulate(), isr_output(), isr_telemetry().
    Chaque fonction retourne bool (true = aller directement à la sortie, remplace les goto).
    Validation : la sortie complète de `build.py --sim` doit être identique à l'octet près avant et
    après (sauvegarde-la avant dans /tmp et compare avec diff).

E17. Nettoyage.
    Renomme les variables `cs` masquées dans foc.c (sinus) en `cth`. Ajoute "-Wshadow" aux drapeaux
    de compilation cible et PC dans firmware/build.py et corrige ce qui apparaît. Mets à jour
    README.md et README.fr.md (section sécurité : nouvelles protections, nouveau code 34, fault_dump),
    incrémente RESC_FW_MINOR dans platform.h et RESC_VERSION_STR.

À LA FIN
Donne un tableau : étape, fichiers touchés, essai ajouté, résultat --tests / --sim, et toute étape
que tu n'as pas pu terminer avec la raison exacte. Ne compile pas le binaire cible si
arm-none-eabi-gcc n'est pas installé : dis-le.
```
