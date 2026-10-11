# RESC 1.0 : fonctions et réglages

Ce document décrit ce que fait chaque fonction ajoutée en version 1.0 et le paramètre qui la règle. Les noms sont ceux de VESC Tool / RESC Tool. Le terminal accepte les mêmes noms avec `get`, `set` et `list` (les `.` et `__` du XML y deviennent `_`, par exemple `app_ppm_conf_ctrl_type`).

État : tout est **validé en simulation** (137 vérifications) et par des **tests PC** (configuration, migration, commande 250, applications). Rien n'a encore été essayé sur la carte : les essais se font par paliers, comme dans [12_essai_banc.md](12_essai_banc.md).

## 1. Observateurs (`foc_observer_type`)

| Valeur | Observateur | Origine |
|---|---|---|
| 0 | ORTEGA_ORIGINAL : observateur non linéaire sur le flux statorique (Lee, Ortega et al. 2010) | VESC 7.00, réimplémenté (mêmes équations, coefficients et bornes) |
| 1 | MXLEMMING : intégrateur du flux de l'aimant, composantes bornées à λ | VESC 7.00 (auteur David Molony, MESC) |
| 2 | ORTEGA_LAMBDA_COMP : Ortega + λ estimé (0,3 à 2,5 × λ) | VESC 7.00 |
| 3 | MXLEMMING_LAMBDA_COMP : MXLemming + λ estimé | VESC 7.00 |
| 4 | MXV : intégrateur, module ramené à λ s'il le dépasse | VESC 7.00 |
| 5 | MXV_LAMBDA_COMP : MXV + λ estimé (comme Ortega) | VESC 7.00 |
| 6 | MXV_LAMBDA_COMP_LIN : MXV + λ suivi par filtre passe-bas du module | VESC 7.00 |
| 7 | RESC_ACTIVE_FLUX : flux actif avec correction non linéaire, module λ + (Ld − Lq)·id (moteurs saillants) | RESC, celui essayé sur la carte ; défaut du profil RESC |
| 8 | RESC_RADIAL : même flux actif, module ramené linéairement vers λ avec un gain de 2·\|ω\| (rien à régler) | RESC |
| 9 | RESC_ACTIVE_FLUX_LAMBDA : 7 + λ réestimé en marche à partir de \|e\|/\|ω\| (1 s, ±30 %) | RESC |
| 10 | RESC_RADIAL_LAMBDA : 8 + λ réestimé | RESC |
| 11 | RESC_BEMF : observateur de FEM (Luenberger sur le courant, sans dérivée du courant, retard compensé), angle indépendant de λ | RESC |
| 12 | RESC_BEMF_LAMBDA : 11 + λ réestimé | RESC |
| 13 | RESC_HYBRID_LAMBDA : flux actif à basse vitesse, FEM à haute vitesse, λ réestimé | RESC |

En profil VESC 7.00, la liste de VESC Tool s'arrête à 6. Les observateurs RESC se choisissent alors au terminal, par exemple `set foc_observer_type 7`.

**Migration depuis RESC 0.1.** RESC 0.1 ignorait le type d'observateur, le gain réduit à l'arrêt, le découplage, le filtre de phase et le mélange Hall : son comportement était fixe. Au premier démarrage de la 1.0, la migration de la configuration règle donc ces champs sur le comportement que tu as essayé :
- observateur 7 (RESC_ACTIVE_FLUX) ;
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
| `foc_sl_openloop_boost_q` / `_max_q` | Courant de démarrage comme VESC : la consigne, au moins `boost_q` (plancher 0,5 A), au plus `max_q` (si > 0). Demander 3 A démarre à 3 A |
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
| ADC | ADC1, ADC2 ; boutons sur TX (régulateur) et RX (marche arrière) quand l'app est « ADC » seule | 15 types ; ADC2 en frein ; régulateur de vitesse ; détection de fil coupé (`voltage_min/max`) ; options RESC : marche arrière intelligente et commande hybride courant / rapport cyclique (ci-dessous) |
| UART | Paquets VESC | La télécommande VESC Remote passe par `COMM_SET_CHUCK_DATA` (module NRF52 de l'Ubox sur UART4) |
| VESC Remote | Manette (`app_chuk_conf.*`) | Courant, sans marche arrière, bidirectionnel ; régulateur (bouton C) ; marche arrière (bouton Z), intelligente ou non ; frein de roue libre |
| PAS | Capteur de pédalage en quadrature sur TX/RX | Cadence ou couple constant ; ADC + PAS (la poignée prime) |

Options RESC de l'app ADC (App > ADC > General, profil RESC) :
- **Marche arrière intelligente** (`resc_smart_rev`, types courant avec marche arrière : centre, bouton, bouton + frein ADC2, bouton + frein au centre). Demande de marche arrière en roulant en avant au-dessus de `resc_smart_rev_erpm` : frein proportionnel à la demande. En dessous : recul à rapport cyclique limité (`resc_smart_rev_max_duty`), atteint en `resc_smart_rev_ramp_time`.
- **Commande hybride courant / rapport cyclique** (`resc_hybrid`, types courant). Le courant reste proportionnel aux gaz, et le rapport cyclique est limité proportionnellement aux gaz : limite = gaz × `resc_hybrid_duty` × `l_max_duty`. Le courant décroît linéairement sur `resc_hybrid_band` (fraction de la limite) et s'annule à la limite, sans coupure brutale. À mi-gaz : mi-couple et, à vide, environ mi-vitesse (0,51 en simulation, sans dépassement). En courant seul, quelques pourcents de gaz suffisent à amener un moteur à vide à pleine vitesse. Le frein n'est pas concerné. La régulation tourne dans `ctrl.c` à 1 kHz, avec le pont maintenu à courant nul.

Fonctions communes :
- **Gaz relâchés :** une sortie nulle (ni gaz ni frein) n'est envoyée qu'une fois, puis l'app se tait. Les commandes de l'outil ou du CAN (courant, frein à main...) ne sont plus écrasées à chaque cycle.
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

Réglage : **Motor Type** (General, General ; `foc_motor_kind`) = **FOC Induction (ACIM)**. L'autre choix est **FOC Permanent Magnets (PMSM / BLDC / IPM)**. Procédure :
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
| Paramètres | Les 202 + 121 de VESC 7.00, octets identiques (inutilisés cachés dans RESC Tool) | 178 + 111 : sans BLDC, DC, DRV8301, UAVCAN, NRF24, limite lente du courant absolu ; avec aides RESC, induction, bande passante, courant de mesure R |
| Version annoncée | 7.00 | 1.00 |

Points communs aux deux profils :
- **Description de la configuration :** la commande 250 envoie le XML du profil compilé.
- **Affichage selon la configuration (profil RESC) :** les onglets envoyés par la commande 250 sont construits par le firmware à chaque connexion. Seuls les paramètres utilisés avec les réglages en cours y figurent :
  - type de moteur : en induction, plus de flux de l'aimant, de Ld - Lq, de MTPA, de défluxage ni de choix d'observateur ; en aimants permanents, plus de paramètres d'induction ;
  - source d'angle : table Hall, codeur, injection (HFI, VSS) et démarrage I/f selon le capteur choisi ; plus d'observateur avec un codeur seul ;
  - options : sonde de température, compensation de saturation et de température, filtre de phase, défluxage ;
  - applications : réglages propres au type de commande PPM et ADC (centre, ADC2, PID, marche arrière), antipatinage seulement en multi-contrôleurs, messages CAN seulement si leur fréquence n'est pas nulle.

  Les valeurs cachées restent transmises et conservées. Après un changement de type de moteur ou de capteur, écrire la configuration puis se reconnecter pour mettre la liste à jour. Les conditions sont dans `tools/resc_conf/desc_vis.py`.
- **Affichage dynamique (bit 7 de la commande 250, profil RESC) :** avec l'octet `0x80 | conf`, le firmware répond avec le même octet et envoie tous les onglets, chacun avec son attribut `visibleIf`, plus une section `<Pages>`. RESC Tool évalue ces conditions pendant l'édition, sans écrire ni se reconnecter. La syntaxe est `|| && ! == != < <= > >= in [..] &` avec parenthèses. Un enum se compare à sa position dans la liste (réduite pour la carte), un booléen à 0 / 1. Seules les conditions qui ne dépendent que de la carte sont évaluées par le firmware. Sans le bit, la réponse est la même qu'avant. Le profil VESC700 renvoie la même description dans les deux cas. Exemples de flux : `tools/resc_conf/samples/` (`conf_xml_*.bin` sans le bit, `vis_conf_xml_*.bin` avec).
- **Stockage en flash :** champ par champ, par nom. On peut passer d'un profil à l'autre, ou changer de version, sans perdre les réglages.
- **Ancien format de RESC 0.1 :** relu au premier démarrage, puis réécrit. `status` affiche « MIGRÉE ».

**Paramètres visibles.**
- **Profil RESC :** 178 paramètres moteur et 111 paramètres d'application, tous utilisés par le firmware et tous visibles (selon les réglages en cours).
- **Choix selon la carte (profil RESC) :** les choix qui n'existent pas sur la carte compilée sont retirés des listes (V0 et V7 sans shunts en ligne, modes à trois capteurs, codeurs SPI1 / PWM / sin/cos sans leurs broches, IMU interne). Règles : `tools/resc_conf/board_enum.py`.
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

## 9. Fonctions reprises de VESC 7.00 (version 1.1)

Toutes passent par les paramètres et les commandes de VESC Tool, avec les mêmes noms. Validées en simulation et par tests PC, pas encore sur la carte.

**Cœur FOC**
- **`foc_control_sample_mode`** : V0 seul ; V0 + V7 interpolé (mise à jour de demi-période au creux du compteur avec l'angle avancé d'une demi-période) ; V0 et V7 (boucle complète deux fois par période, cartes à shunts en ligne seulement, retiré des listes sinon).
- **`foc_current_sample_mode`** : plus longue fenêtre (la phase à la fenêtre la plus courte est reconstruite), tous les capteurs, fort courant (la phase au plus fort courant est reconstruite).
- **`foc_overmod_factor`** (surmodulation), **`foc_mag_vd_max`** (part de la tension pour l'axe d), **`foc_current_filter_const`** (filtre des courants affichés et des limites), **`foc_fw_backoff`** (défluxage réduit quand iq dépasse sa consigne).
- **`foc_short_ls_on_zero_duty`** : tension nulle = low-sides passants en continu.
- **Frein (`SET_CURRENT_BRAKE`) comme VESC :** courant opposé à la vitesse ; quand il n'est plus atteignable (basse vitesse, changement de sens), phases en court-circuit, jusqu'à l'arrêt et à l'arrêt. Avec `foc_short_ls_on_zero_duty`, court-circuit par les low-sides.
- **Frein à main (`SET_HANDBRAKE`)** : courant continu à angle figé. Les apps PPM et ADC ne l'écrasent plus quand les gaz sont relâchés.
- **Démarrage I/f et VSS** : courant de démarrage = la consigne, bornée par `foc_sl_openloop_boost_q` et `foc_sl_openloop_max_q`. Auparavant, environ 30 % du courant max (≈ 20 A) quelle que soit la consigne. Après le relais vers l'observateur, la rampe douce ne limite plus que la montée du courant : une baisse de consigne est suivie tout de suite.
- **HFI :** `foc_hfi_reset_erpm` (reprise sur l'observateur au-dessus, recherche de position en dessous) et `foc_hfi_obs_ovr_sec` (angle HFI recalé sur l'observateur après le retour à basse vitesse).
- **`l_in_current_map_start`, `l_in_current_map_filter`** : courant moteur réduit à l'approche de la limite batterie. **`p_pid_kd_proc`** : dérivée sur la position mesurée.
- **Limites batterie :** `l_in_current_min` (régénération) et `l_watt_min` sont désormais appliqués ; les limites batterie sont converties en limite de courant moteur (I batterie ≈ I moteur × duty), puis corrigées par une boucle lente.

**Codeurs (`m_sensor_port_mode`)** : les 16 modes de VESC 7.00 sauf « Custom » (Lisp) : ABI, AS5047, AMT22, TLE5012 (port Hall), MT6816, AS5x47U, MA782, BiSS-C (SPI1, CRC6), TS5700N8501 et sa version multitour (RS485 à 2,5 Mbit/s : réception par l'UART du port COMM, requête émise par logiciel, sens du transceiver sur ADC1), TLE5012 en SSC sur les broches SPI du port COMM, AD2S1205, sin/cos (`m_encoder_sin_*`, `m_encoder_cos_*`, filtre, correction de phase), PWM (entrée servo) et PWM + ABI (ABI recalé par la position absolue PWM). Lecture à chaque période, contrôle de parité ou de CRC, échantillon invalide ignoré. Un mode dont la carte n'a pas les broches est retiré de la liste.

**IMU (`imu_conf.*`)** : BMI160 interne (Ubox) ou externe sur le port UART ; filtres, rotation de montage, décalages ; attitude Madgwick, Mahony ou Madgwick Fusion ; page IMU et calibration de VESC Tool.

**BMS (`bms.*`)** : BMS VESC sur le bus CAN ; limites par température des cellules, état de charge, cellule la plus basse et la plus haute ; page BMS et commandes de l'outil relayées ; relais des trames (`bms.fwd_can_mode`).

**Outil et protocole**
- **Échantillonnage (page Sampled Data)** : 1000 périodes (courants, tensions de phase, point milieu, courant filtré, état Hall, angle), modes immédiat, au démarrage, sur démarrage ou sur défaut avec historique.
- **Affichage de la position** (observateur, capteur, PID position et écarts), **statistiques** (moyennes et maxima), **odomètre** enregistré à l'arrêt et au redémarrage, **coupure batterie** (`SET/GET_BATTERY_CUT`, diffusée par CAN), **limites temporaires** (`SET_MCCONF_TEMP(_SETUP)`), **arrêt et redémarrage** (`COMM_SHUTDOWN`), **informations firmware**.
- **Détection automatique (assistant de VESC Tool)** : R/L, flux, Hall puis codeur ; courant max = √(P / 1,5 R) d'après la puissance de pertes ; autres contrôleurs du bus en parallèle ; configuration précédente remise en cas d'échec.
- **CAN :** trames brutes (`CAN_FWD_FRAME`), changement de vitesse pour tout le bus, arrêt par CAN, `can_mode` (VESC, pont vers l'outil, inutilisé), mise à jour du firmware sur tout le bus, effacement et écriture du bootloader.
- **Sortie servo** (`servo_out_enable`, `SET_SERVO_POS`) et **UART permanent** (`permanent_uart_enabled`).

**Non repris :** Lisp (et ce qui en dépend : paquets, QML, codeur personnalisé), limite lente du courant absolu (`l_slow_abs_current` : la limite reste échantillon par échantillon), DRV8301, UAVCAN, NRF24, IMU MPU9x50 / ICM20948 / LSM6DS3, programmateur SWD (BM_*), cartes CAN GNSS / IO / PSW, compression LZO (l'outil envoie sans compression), modes BLDC et DC.
