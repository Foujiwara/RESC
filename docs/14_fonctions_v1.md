# RESC 1.0 : fonctions et réglages

Ce document décrit ce que fait chaque fonction ajoutée en version 1.0 et le paramètre qui la règle. Les noms sont ceux de VESC Tool / RESC Tool. Le terminal accepte les mêmes noms avec `get`, `set` et `list` (les `.` et `__` du XML y deviennent `_`, par exemple `app_ppm_conf_ctrl_type`).

État : tout est **validé en simulation** (104 vérifications) et par des **tests PC** (configuration, migration, commande 250, applications). Rien n'a encore été essayé sur la carte : les essais se font par paliers, comme dans [12_essai_banc.md](12_essai_banc.md).

## 1. Observateurs (`foc_observer_type`)

| Valeur | Nom VESC | Observateur RESC | Usage |
|---|---|---|---|
| 0 | ORTEGA_ORIGINAL | Flux actif, correction non linéaire (celui de RESC 0.1) | Recommandé, testé sur la carte |
| 1 | MXLEMMING | Flux actif, rappel radial linéaire de gain 2\|ω\| | Sans gain à régler |
| 2 | ORTEGA_LAMBDA_COMP | Flux actif non linéaire + λ adaptatif | λ mal connu, échauffement des aimants |
| 3 | MXLEMMING_LAMBDA_COMP | Rappel radial + λ adaptatif | Idem |
| 4 | MXV | Observateur de FEM (Luenberger sur le courant, sans dérivée) | Vitesses moyennes et hautes ; angle indépendant de λ |
| 5 | MXV_LAMBDA_COMP | FEM + λ adaptatif | |
| 6 | MXV_LAMBDA_COMP_LIN | Hybride : flux actif à basse vitesse, FEM à haute vitesse | |

**Migration depuis RESC 0.1.** RESC 0.1 ignorait le type d'observateur, le gain réduit à l'arrêt, le découplage, le filtre de phase et le mélange Hall : son comportement était fixe. Au premier démarrage de la 1.0, la migration de la configuration règle donc ces champs sur le comportement que tu as essayé :
- observateur 0 ;
- gain plein à l'arrêt ;
- anticipation croisée et FEM ;
- filtre de phase désactivé ;
- bascule Hall sans mélange.

Les nouvelles fonctions s'activent ensuite une par une. Une configuration neuve, ou réécrite par l'outil, prend les valeurs par défaut de VESC : observateur 3, filtre de phase actif sous 4000 ERPM.

Autres réglages :
- **λ adaptatif :** ré-estimé en marche à partir de |e|/|ω|, avec une constante de temps de 1 s, borné à ±30 % de la valeur configurée.
- **`foc_observer_gain_slow` :** fraction du gain gardée à l'arrêt.
- **`foc_observer_offset` :** avance d'angle en périodes. En profil VESC, la valeur par défaut −1 correspond à « aucune correction ».
- **`foc_speed_soure` :** source de la vitesse, soit l'angle de commande (via une PLL), soit l'observateur.

## 2. Capteurs (`foc_sensor_mode`)

| Valeur | Mode | Fonctionnement |
|---|---|---|
| 0 | Sensorless | Démarrage I/f, puis observateur |
| 1 et 9 | Codeur ABI | Index non utilisé |
| 2 | Hall | Mélange linéaire vers l'observateur entre `foc_sl_erpm_start` et `foc_sl_erpm` ; vote majoritaire sur `m_hall_extra_samples` lectures |
| 3 et 5 à 8 | HFI | Injection de créneaux ++−− sur l'axe d estimé et démodulation sur q (il faut Lq > Ld) ; polarité trouvée par impulsions de courant d ; observateur au-dessus de `foc_sl_erpm_hfi` |
| 4 | VSS | Position trouvée par injection à l'arrêt, puis démarrage I/f sans calage, puis observateur |

Réglages HFI :
- **`foc_hfi_voltage_start / run / max`** : tension d'injection.
- **`foc_hfi_gain`** : bande passante de suivi (2000 × gain rad/s).
- **`foc_hfi_amb_current`, `foc_hfi_amb_tres`** : courant (A) et durée (ms) des impulsions de polarité.

## 3. Options sensorless

| Paramètre | Effet dans RESC |
|---|---|
| `foc_openloop_rpm` | Vitesse de relais vers l'observateur |
| `foc_openloop_rpm_low` | Vitesse de relais à courant nul, en fraction de la précédente (0 = vitesse fixe) |
| `foc_sl_openloop_time_lock` / `_time_ramp` | Durée du calage, durée de la rampe |
| `foc_sl_openloop_time` | Temps minimal à la vitesse de relais, observateur accroché |
| `foc_sl_openloop_hyst` | Temps sous la vitesse de sortie avant de revenir en I/f |
| `foc_sl_openloop_boost_q` / `_max_q` | Courant de démarrage, et son maximum |
| `foc_start_curr_dec` / `_rpm` | Courant réduit à basse vitesse |
| `foc_sat_comp_mode` / `foc_sat_comp` | Compensation de la saturation (inductances, λ) |
| `foc_temp_comp` / `_base_temp` | R (+0,393 %/K) et λ (−0,11 %/K) selon la température du moteur |
| `foc_cc_decoupling` | Anticipation croisée et FEM. En profil VESC, l'anticipation de FEM reste toujours active |
| `foc_mtpa_mode` | MTPA sur la consigne ou sur le courant mesuré (moteurs saillants) |
| `foc_fw_current_max`, `_duty_start`, `_ramp_time`, `_q_current_factor` | Défluxage |

Le démarrage I/f est amorti : quand l'observateur voit le rotor plus rapide que la rampe, l'angle imposé le rattrape. Cela supprime le recul au démarrage.

## 4. Filtres de phase

**`foc_phase_filter_enable`** active le filtre RC des tensions de phase (PC9 sur l'Ubox) sous **`foc_phase_filter_max_erpm`**. L'observateur utilise alors les tensions mesurées, qui ne comportent pas l'erreur de temps mort.

La surveillance compare tension mesurée et tension commandée. Elle lève le défaut PHASE_FILTER sauf si **`foc_phase_filter_disable_fault`** est actif.

Valeurs par défaut VESC : filtre actif jusqu'à 4000 ERPM, défaut désactivé.

**À valider sur la carte :** la constante du filtre de l'Ubox n'est pas connue. Si l'angle se dégrade à basse vitesse, désactive le filtre.

## 5. Duty, vitesse, position

- **Duty :** tension imposée. Le courant est seulement borné. Quand la consigne baisse, un PI duty → courant de frein freine dans les limites (**`foc_duty_dowmramp_kp/ki`**).
- **Limite selon le duty :** **`l_duty_start`** réduit le courant de motorisation au-delà de ce duty.
- **PID vitesse :**
  - **`sp_pid_loop_rate`** fixe la cadence (1 kHz au plus).
  - **`s_pid_speed_source`** choisit la source : PLL filtrée, rapide ou brute.
- **Position :** commande VESC Tool, CAN `SET_POS`, PPM « Position 180°/360° ».
  - Régulateur PID position → courant : `p_pid_*`, réduction `p_pid_ang_div`, gains réduits sous `p_pid_gain_dec_angle`.
  - La position est l'angle électrique intégré. Elle n'est exacte qu'avec un codeur, ou Hall/sans capteur en mouvement.

## 6. Applications (`app_to_use`)

| App | Entrées | Fonctions |
|---|---|---|
| PPM | Servo PB6 (TIM4) | 12 types : courant, frein, duty, PID vitesse, marche arrière à hystérésis, marche arrière intelligente, position ; filtre médian, démarrage sécurisé, rampes, courbes |
| ADC | ADC1, ADC2 ; boutons sur TX (régulateur) et RX (marche arrière) quand l'app est « ADC » seule | 15 types ; ADC2 en frein ; régulateur de vitesse ; détection de fil coupé (`voltage_min/max`) |
| UART | Paquets VESC | La télécommande VESC Remote passe par `COMM_SET_CHUCK_DATA` (module NRF52 de l'Ubox sur UART4) |
| VESC Remote | Manette (`app_chuk_conf.*`) | Courant, sans marche arrière, bidirectionnel ; régulateur (bouton C) ; marche arrière (bouton Z), intelligente ou non ; frein de roue libre |
| PAS | Capteur de pédalage en quadrature sur TX/RX | Cadence ou couple constant ; ADC + PAS (la poignée prime) |

Fonctions communes :
- **Plusieurs contrôleurs :** `multi_esc` recopie la consigne aux contrôleurs vus sur le CAN.
- **Antipatinage :** `tc` / `tc_max_diff` réduisent le courant d'une roue qui tourne plus vite que la plus lente.
- **Interrupteur d'arrêt :** `kill_sw_mode`, sur PPM ou ADC2.
- **Coupure par l'outil :** `COMM_APP_DISABLE_OUTPUT`.
- **Valeurs décodées :** envoyées à l'outil (assistant de calibrage PPM/ADC/manette).

Courbes de gaz RESC (`throttle_exp_mode`), avec a = |entrée| et k = exposant :

| Mode | Formule |
|---|---|
| Exponentielle | (e^(k·a) − 1)/(e^k − 1) |
| Naturelle | a^(1+k) |
| Polynomiale | a + k(a³ − a) |

Un k positif adoucit le début de course.

**Bouton d'alimentation (`shutdown_mode`) :**

| Mode | Effet |
|---|---|
| ALWAYS_OFF | Coupe 0,5 s après relâchement. **Seulement avec un interrupteur à maintien** |
| ALWAYS_ON | Jamais coupé par le firmware |
| TOGGLE | Appui long de 2 s |
| OFF_AFTER_x | Appui long, ou inactivité pendant x |

Dans tous les cas, le moteur doit être arrêté. Par défaut : arrêt après 30 minutes d'inactivité.

**Sortie AUX (`m_out_aux_mode`), sondes de température moteur, défauts :**
- **Sondes de température moteur :** NTC 10k et 100k, PTC 1k, KTY83/122, KTY84/130, PT1000, sondes personnalisées.
- **Défauts supplémentaires (`l_additional_faults`) :** survitesse et survitesse absolue.
- **`m_fault_stop_time_ms` :** temps minimal avant le réarmement après un défaut.

## 7. Moteur à induction (profil RESC, ou terminal en profil VESC)

Réglage : `foc_motor_kind` = 1. Procédure :
1. `detect_rl` : mesure Rs et l'inductance transitoire σLs, qui devient « Motor L ».
2. `detect_acim` : échelon de courant continu, le moteur ne tourne pas, environ 5 s. On en tire Lm, Lr et Rr, à partir de la constante de temps rotorique.
3. Règle le courant magnétisant `foc_acim_im`, de l'ordre du courant à vide du moteur.
4. Optionnel : `foc_acim_base_erpm` pour le défluxage en 1/vitesse.
5. Choisis la source d'angle :
   - **Codeur :** commande vectorielle indirecte (angle rotor + glissement).
   - **Sans capteur :** magnétisation (3 τr), rampe de fréquence, puis observateur.

Il n'y a pas de reprise au vol sans capteur : le flux doit d'abord être rétabli.

Commandes du terminal pour l'induction :
- **`acim` :** paramètres et, en marche, flux rotorique (en mWb et en % du nominal), glissement et vitesse du rotor. Indique aussi l'étape suivante (`detect_rl`, `detect_acim`, réglage du courant magnétisant, `acim on`).
- **`acim on` / `acim off` :** passe en moteur à induction ou à aimants.
- **`detect_acim [A]`** : identification de Lm, Lr et Rr.
- **`conf` et `status` :** affichent les grandeurs propres à l'induction quand ce mode est actif.

## 8. Profils de configuration

| | Profil VESC700 (défaut) | Profil RESC |
|---|---|---|
| Compilation | `python firmware/build.py` | `python firmware/build.py --conf resc` |
| Outil | VESC Tool officiel, Android | **RESC Tool uniquement** (version qui lit la description) |
| Paramètres | Les 202 + 121 de VESC 7.00, octets identiques (inutilisés cachés dans RESC Tool) | 149 + 80 : sans BLDC, DC, sin/cos, DRV8301, BMS, UAVCAN, NRF24, IMU ; avec aides RESC, induction, bande passante, courant de mesure R |
| Version annoncée | 7.00 | 1.00 |

Points communs aux deux profils :
- **Description de la configuration :** la commande 250 envoie le XML du profil compilé.
- **Stockage en flash :** champ par champ, par nom. On peut passer d'un profil à l'autre, ou changer de version, sans perdre les réglages.
- **Ancien format de RESC 0.1 :** relu au premier démarrage, puis réécrit. `status` affiche « MIGRÉE ».

**Paramètres visibles.**
- **Profil RESC :** 149 paramètres moteur et 80 paramètres d'application, tous utilisés par le firmware et tous visibles.
- **Profil VESC700 :** la description envoyée à RESC Tool par la commande 250 est l'officielle, avec la même sérialisation et la même signature. Seule différence : les paramètres inutilisés sont retirés des onglets, donc invisibles (141/202 et 80/121 visibles), et les aides décrivent le comportement de RESC.
- **VESC Tool officiel :** il ne demande pas la description et affiche toujours tous les paramètres de VESC 7.00. On ne peut rien y cacher sans casser la compatibilité.
- **Onglets BLDC, DC et GPD :** ils restent présents, mais vides.

**Valeurs affichées.** La description envoyée à l'outil est générée pour la carte compilée :
- **valeurs par défaut :** les vraies valeurs du firmware, avec les réglages prudents de la carte ;
- **plages :** limitées au matériel (courants, tension, ERPM, fréquence PWM, duty, température) ;
- **plafond de courant :** une note l'indique tant que la carte n'est pas validée.

Le script est `tools/resc_conf/board_xml.py`. Il est lancé par `build.py`, à partir des valeurs relevées par `tests/dump_defaults.c`.

**Binaires dans `firmware/release/` :**
- `resc_ubox_single_85_200.bin` : profil VESC700. Fonctionne avec VESC Tool et avec RESC Tool.
- `resc_ubox_single_85_200_profil_resc.bin` : profil RESC. Uniquement avec un RESC Tool qui lit la description, c'est-à-dire compilé depuis le source qui contient la commande 250.

Le contrôle est automatique dans `python firmware/build.py --tests`, via `tools/resc_conf/audit_params.py`.

**Ne pas distribuer le profil RESC avant la version de RESC Tool qui lit la description.**

Génération : `python -I tools/resc_conf/make_resc_xml.py` (XML RESC), puis `python -I tools/resc_conf/gen_resc_conf.py` (code). Tests : `python firmware/build.py --tests`.
