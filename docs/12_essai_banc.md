# 12. Premier essai au banc — Ubox SINGLE 85_200

> **À lire avant de brancher.** Ce firmware a été compilé et validé **en simulation** (51 vérifications : identification, démarrage, observateur, Hall, codeur, frein, vitesse, boucle ouverte, timeout, défauts) et par des tests PC de configuration (30 vérifications). **Il n'a jamais tourné sur la vraie carte.** Les drivers matériels (TIM1, ADC, USB, CAN, flash) sont écrits d'après le manuel RM0090 et n'ont pas encore été vérifiés en réel. L'essai se fait donc **par paliers**, avec une alimentation de labo **limitée en courant**.

## 0. Matériel

- Alimentation de labo **24 à 48 V, limitée à 2-3 A** pour les étapes 1 à 7. Pas de batterie au début.
- Moteur **fixé, sans charge**, roue en l'air.
- Sonde ST-Link (V2 ou V3) sur le connecteur SWD de l'Ubox (SWDIO, SWCLK, GND ; 3,3 V en option).
- Câble USB vers le PC.
- Multimètre.

Outil de flashage déjà installé :
`C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe`
(désigné ci-dessous par `ST-LINK_CLI`).

## 1. Sauvegarder le firmware d'origine (obligatoire)

La carte doit être alimentée : appuyer sur le bouton pour l'allumer.

```bash
"/c/Program Files (x86)/STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility/ST-LINK_CLI.exe" -c SWD UR -Dump 0x08000000 0x100000 vesc_ubox_backup.bin
```

Le fichier fait exactement 1 048 576 octets. Il contient le firmware VESC, sa configuration et le **bootloader** (secteur 11). Il faut le garder précieusement.

## 2. Flasher RESC

```bash
"/c/Program Files (x86)/STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility/ST-LINK_CLI.exe" -c SWD UR -P firmware/release/resc_ubox_single_85_200.bin 0x08000000 -V -Rst
```

Le `.bin` doit être écrit **à l'adresse 0x08000000**. Il couvre 0x08000000 à 0x08023DF8 (secteurs 0 à 5) ; la zone de configuration (secteurs 1-2) y est à 0xFF, donc RESC démarre sur ses valeurs par défaut. Le bootloader VESC du secteur 11 n'est pas touché.

Avec l'interface graphique de ST-LINK Utility : *Target → Program & Verify*, fichier `resc_ubox_single_85_200.bin`, adresse de départ `0x08000000`, case *Reset after programming* cochée.

Pour revenir au firmware d'origine :

```bash
"/c/Program Files (x86)/STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility/ST-LINK_CLI.exe" -c SWD UR -ME -P vesc_ubox_backup.bin 0x08000000 -V -Rst
```

## 3. Premier démarrage (pont toujours coupé)

1. Allumer la carte. RESC maintient l'alimentation (PA15) dès les premières instructions. La carte doit **rester allumée** quand on relâche le bouton.
2. Environ 0,4 s plus tard (calibration des offsets) : **LED verte à ~30 %** = attente (prêt). **Vert 100 %** = pont actif (marche, démarrage, détection). **Rouge clignotant** = défaut.
3. Ouvrir la console :

```bash
python tools/resc_cli.py
```

(ou `python tools/resc_cli.py COM5`). Le message attendu est : `Connecte : firmware RESC 7.00 sur UBOX_SINGLE_85_200`.

4. Taper dans l'ordre :

| Commande | Attendu | Si ce n'est pas le cas |
|---|---|---|
| `status` | état `IDLE`, défaut `aucun`, **moteur NON IDENTIFIE** | noter le code de défaut |
| `hw` | offsets courant entre ~1950 et ~2150 | offsets très différents : ampli de courant, VREF |
| `values` | **Vbus = tension de l'alimentation ±0,5 V**, T_FET ≈ température ambiante | Vbus faux : VREF (3,44 V ?) ou diviseur |
| `rt` | ISR de quelques µs, charge < 30 %, 0 dépassement | noter les valeurs |

Tant que le moteur n'est pas identifié, **les commandes de marche sont refusées**. Seules les détections peuvent piloter le pont.

Arrêt de la carte : appui long (> 2 s) sur le bouton, moteur à l'arrêt.

## 4. Calibration des offsets (moteur arrêté)

```
offsets_cal
```

La commande mesure les offsets pont coupé, puis sous PWM au vecteur nul (le pont commute, aucun courant). Elle mesure les **courants** et les **tensions de phase** dans les deux cas : `foc_offsets_current_*`, `foc_offsets_voltage_*` (sous PWM) et `foc_offsets_voltage_undriven_*` (pont coupé). Les valeurs sont **sauvegardées** et utilisées aux démarrages suivants.
Les offsets de tension sont l'écart de chaque phase à la moyenne des trois (les trois phases sont au même potentiel pendant la mesure) : quelques mV sont normaux.
Échec `-6` (offsets instables sous PWM) : bruit de mesure, à signaler avant d'aller plus loin.

## 5. Identification du moteur (manuelle : R/L, puis flux, puis Hall)

1. `detect_rl` : le moteur **ne tourne pas** (il se cale une fois). Durée environ 6 s :
   - recherche du courant : paliers ×1,5 de 0,4 s jusqu'à au moins 1,5 V et 8 A (ou 80 % du courant autorisé) ;
   - trois paliers montants de 0,8 s à ½, ¾ et 1 × ce courant : R = pente ΔV/ΔI ;
   - sifflement (créneaux de tension) sur l'axe d puis sur l'axe q : Ld, Lq ;
   - retour du courant à zéro.
   Le courant est envoyé entre les phases B et C (la phase A ne porte rien). La console affiche les paliers, la chute ohmique et la tension perdue dans le temps mort. Résultat attendu pour un moteur de moyeu : R de quelques dizaines de mΩ, L de quelques dizaines à centaines de µH.
2. `detect_lambda 5 0.3` : le moteur **va tourner** en boucle ouverte à 5 A jusqu'à un rapport cyclique de 0,3. Il doit être libre.
   - Erreur `-5` : le rotor n'a pas suivi. Recommencer avec plus de courant (`detect_lambda 8 0.3`) ou un moteur moins freiné.
Après chaque mesure réussie, RESC **calcule et sauvegarde automatiquement** ce qui en dépend :
   - après R/L : `kp = L × 1000`, `ki = R × 1000` (bande passante de courant 1000 rad/s) ;
   - après λ : gain d'observateur `1000 / λ²`.
   Les valeurs sont affichées. Pour une autre bande passante : `foc_gains 1500` par exemple.
   Avec VESC Tool, les boutons « R/L » et « Flux linkage » font la même chose (l'assistant automatique, lui, est désactivé).
3. Limites prudentes pour les premiers tours :

```
set l_current_max 10
set l_current_min -10
set l_in_current_max 5
set l_in_current_min -3
set l_abs_current_max 40
store
```

Sauvegarder avec `store`, puis vérifier avec `status` : le moteur doit apparaître comme `identifie`.

## 6. Premiers tours (alimentation toujours limitée à 3 A)

| Commande | Attendu |
|---|---|
| `current 3` | démarrage en boucle ouverte (~0,5 s), puis relais sur l'observateur ; le moteur accélère |
| `values` / `watch` | ERPM stable, Iq ≈ 3 A, pas de défaut |
| `stop` | roue libre |
| `current -3` | démarrage en sens inverse |
| `rpm 3000` | vitesse régulée |
| `brake 3` | freinage puis coupure du pont à basse vitesse |
| `rt` | charge ISR, nombre de reconstructions de courant |

Boucle ouverte (essai, comme sur VESC) :

| Commande | Effet |
|---|---|
| `foc_openloop 5 1000` | courant de 5 A imposé, angle tournant à 1000 ERPM (ERPM négatif = sens inverse) |
| `foc_openloop_duty 0.1 1000` | rapport cyclique 0,1 imposé, **sans régulation de courant** : choisir un rapport proche de la FEM à cette vitesse, sinon le courant n'est limité que par la coupure absolue |
| `stop` | arrêt |

L'observateur n'est jamais utilisé dans ce mode ; il reste calculé, ce qui permet de comparer son angle à l'angle imposé.

`resc_cli` envoie un maintien toutes les 250 ms : une consigne reste active tant que la console est ouverte. `stop` pour arrêter. Fermer la console, ou perdre la liaison USB, relâche le moteur en 1 s (timeout) : c'est une sécurité « homme mort ».

## 7. Capteurs (si le moteur a des sondes Hall)

```
set foc_sensor_mode 0
detect_hall 5
```

Le moteur tourne lentement : 3 tours dans un sens, 3 dans l'autre, 3 à nouveau. Résultat attendu : 6 valeurs entre 0 et 200, et 255 pour les états 0 et 7. La table est sauvegardée automatiquement. Puis :

```
set foc_sensor_mode 2
store
current 5
```

Le moteur démarre **directement sur les Hall**, sans boucle ouverte. L'observateur prend le relais au-dessus de `foc_sl_erpm` (3500 ERPM par défaut). La commande `sensor` affiche la source d'angle active.

Codeur ABI : `set m_encoder_counts <fronts par tour, x4>`, puis `detect_enc 5`, puis `set foc_sensor_mode 1`.

## 8. VESC Tool

Brancher l'USB et se connecter. RESC se présente en **firmware 7.00**, nom `RESC`, carte `UBOX_SINGLE_85_200`.

| Fonction de VESC Tool | Comportement RESC |
|---|---|
| Données temps réel | oui (GET_VALUES / SETUP) |
| Lecture / écriture de la configuration moteur et application | oui. Format 7.00 exact ; les valeurs au-delà des limites de la carte sont bornées |
| Boutons « mesurer R/L », « flux linkage », « Hall », « codeur » | oui, avec les séquences RESC |
| « Detect and calculate parameters » (assistant automatique) | **non supporté, volontairement** : message dans le terminal |
| Terminal | oui (mêmes commandes que `resc_cli`) |
| Contrôleurs sur le bus CAN (FORWARD_CAN) | oui |
| Mise à jour du firmware par l'outil | effacement et écriture du tampon : oui ; version compressée : non. Préférer le ST-Link |

**Non vérifié** : la compatibilité exacte avec la version de VESC Tool installée. Si l'outil refuse la connexion ou passe en « limited mode », utiliser `resc_cli.py` : toutes les fonctions sont accessibles depuis le terminal.

## 9. Monter en puissance

Seulement après les étapes 1 à 7 réussies :
1. Alimentation de labo à 10 A, puis batterie **avec fusible**.
2. Augmenter `l_current_max` par paliers de 10 A, avec `l_abs_current_max` à environ 1,5 × `l_current_max`.
3. **Plafond actif** : tant que la carte est au statut `REVIEWED`, RESC limite à 25 % des limites de la carte (75 A moteur, 105 A de coupure). Ce plafond ne sera levé qu'après validation au banc.

## 10. Codes de défaut

| Code | Signification | Action |
|---|---|---|
| 1 / 2 | surtension / sous-tension | vérifier Vbus, `l_max_vin`, `l_min_vin` |
| 4 | surintensité (logicielle ou watchdog ADC) | `faults` pour la crête vue ; réduire le courant ; vérifier les paramètres moteur |
| 5 / 6 | température FET / moteur | refroidir ; vérifier la lecture des NTC |
| 10 | redémarrage par watchdog | à signaler (ISR bloquée ?) |
| 15-17 | offset de courant anormal au démarrage | ampli de mesure, alimentation |
| 32 | échec de démarrage ou perte de synchronisation | λ ou R faux, courant de démarrage trop faible, charge trop forte |

Le réarmement est automatique 0,5 s après relâchement de la commande (au plus 3 surintensités en 10 s). Sinon, utiliser `clear`.

## 11. Ce qu'il faut me rapporter

- Sortie de `status`, `hw`, `rt` et `conf` ;
- valeurs de `detect_rl` et `detect_lambda` ;
- comportement au démarrage (bruits, à-coups) ;
- tout code de défaut, et ce que l'on faisait à ce moment-là ;
- si possible, l'allure du courant de phase à l'oscilloscope (sortie d'ampli) près d'une commutation, pour mesurer T_win.
