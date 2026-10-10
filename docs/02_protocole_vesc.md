# 2. Protocoles CAN et UART VESC (`release_7_00`)

Sources consultées : `datatypes.h`, `comm/comm_can.c`, `comm/commands.c`, `comm/packet.c` de la branche `release_7_00` de https://github.com/vedderb/bldc.
Le code VESC est sous licence GPLv3. On ne reprend **que le format des messages**, réimplémenté de zéro ; aucun code n'est copié. Le statut juridique d'une réimplémentation de protocole est à faire confirmer si le firmware est distribué sous une autre licence [V].

## 2.1 Encodage commun [C]

- Ordre des octets : **big-endian**.
- `buffer_append_float16(v, scale)` = `int16(v·scale)`. Ce n'est **pas** un IEEE half.
- `buffer_append_float32(v, scale)` = `int32(v·scale)`.
- `float32_auto` (utilisé dans la sérialisation de configuration) est un format flottant propre à VESC, à réimplémenter si on émule MCCONF [V].

## 2.2 Trame UART [C]

```
[start][len][payload ...][crc16_hi][crc16_lo][0x03]
start = 2 → len sur 1 octet (1..255)
start = 3 → len sur 2 octets BE (≥ 255)
start = 4 → len sur 3 octets BE (≥ 65535)
CRC16 calculé sur le payload seul
```
- CRC : CRC-16/XMODEM (poly 0x1021, init 0x0000, sans réflexion) [C de mémoire du `crc.c` VESC, **à confirmer par un vecteur de test**].
- Taille max du payload : `PACKET_MAX_PL_LEN` (512 octets en 7.00 [V]).
- Le décodeur VESC n'a pas de timeout et se resynchronise octet par octet. **Amélioration retenue :** ajouter un timeout inter-octet (par ex. 50 ms) qui abandonne une trame partielle. Cela reste compatible.
- Le premier octet du payload est le `COMM_PACKET_ID`.

## 2.3 CAN [C]

- Trames **étendues 29 bits** : `EID = (CAN_PACKET_ID << 8) | controller_id`. ID 255 = diffusion.
- Débit par défaut 500 kbit/s (configurable ; `CAN_PACKET_UPDATE_BAUD` = 63).

### Commandes reçues (consignes)

| ID | Message | Payload | Échelle |
|---|---|---|---|
| 0 | SET_DUTY | int32 | duty × 1e5 |
| 1 | SET_CURRENT | int32 (+ int16 off_delay optionnel) | A × 1e3 ; off_delay s × 1e3 |
| 2 | SET_CURRENT_BRAKE | int32 | A × 1e3 |
| 3 | SET_RPM | int32 | ERPM |
| 4 | SET_POS | int32 | degrés × 1e6 |
| 10 | SET_CURRENT_REL | int32 (+ int16 off_delay) | fraction × 1e5 |
| 11 | SET_CURRENT_BRAKE_REL | int32 | fraction × 1e5 |
| 12 | SET_CURRENT_HANDBRAKE | int32 | A × 1e3 |
| 13 | SET_CURRENT_HANDBRAKE_REL | int32 | fraction × 1e5 |

Chaque commande réarme le **timeout de commande**.

> ⚠ **Incohérence observée dans le source 7.00 [V] :** pour `SET_CURRENT` avec `off_delay` (len = 6), l'émetteur `comm_can_set_current_off_delay` écrit le courant (4 o) puis `off_delay` (2 o). Le récepteur, d'après le code lu, lit `off_delay` en premier. Ce point est à reconfirmer ligne à ligne. Décision provisoire : décoder selon **l'ordre de l'émetteur** (courant en tête), car c'est ce que les maîtres VESC envoient, puis valider par un essai d'interopérabilité avec un VESC réel.

### Statuts émis (diffusion périodique, 8 octets)

| ID | Message | Contenu |
|---|---|---|
| 9 | STATUS | ERPM int32 ; courant moteur int16 ×10 ; duty int16 ×1000 |
| 14 | STATUS_2 | Ah int32 ×1e4 ; Ah chargés int32 ×1e4 |
| 15 | STATUS_3 | Wh int32 ×1e4 ; Wh chargés int32 ×1e4 |
| 16 | STATUS_4 | T_FET ×10 ; T_moteur ×10 ; I_entrée ×10 ; pos PID ×50 (int16) |
| 27 | STATUS_5 | tachymètre int32 ; V_in int16 ×10 ; réservé int16 = 0 |
| 58 | STATUS_6 | ADC1, ADC2, ADC3, PPM (int16 ×1e3) |

La période et la sélection des statuts sont configurables.

### Transport de paquets « longs » (passerelle vers les commandes UART)

| ID | Rôle | Format |
|---|---|---|
| 5 | FILL_RX_BUFFER | [offset u8][≤ 7 octets] |
| 6 | FILL_RX_BUFFER_LONG | [offset u16 BE][≤ 6 octets] |
| 7 | PROCESS_RX_BUFFER | [id émetteur][send_mode][len u16][crc16 u16] |
| 8 | PROCESS_SHORT_BUFFER | [id émetteur][send_mode][payload ≤ 6] |

`send_mode` : 0 = traiter et répondre par CAN ; 1 = relayer vers la dernière interface ; 2 = traiter sans réponse ; 3 = variante de 0 (voir source).
C'est par ce mécanisme que VESC Tool atteint les contrôleurs esclaves via `COMM_FORWARD_CAN` (34).

### Découverte

- PING (17) : [id émetteur]. PONG (18) : [id répondeur][HW_TYPE]. HW_TYPE_VESC = 0.

### Messages de configuration rapide (à implémenter en phase 2)

21–26 (limites de courant et ERPM FOC, avec ou sans stockage), 29–30 (battery cut), 31 SHUTDOWN, 19–20 (DETECT_APPLY_ALL_FOC).
**Règle :** toute écriture « STORE » est différée jusqu'à l'arrêt moteur (doc 1 §1.4) et bornée par `hardware_limits.h`.

### Non implémentés (hors périmètre)

IO board (32–37), BMS (38–54, 64–68), GNSS (59–62), PSW (46–47), TS5700N8501 (28).
Ils sont ignorés silencieusement, mais **pas filtrés au niveau matériel**, afin de pouvoir être relayés.

## 2.4 Commandes UART / paquets (COMM_PACKET_ID) [C]

### Ensemble minimal (phase 2)

| ID | Commande | Réponse / effet |
|---|---|---|
| 0 | FW_VERSION | major, minor, HW_NAME\0, UUID 12 o, pairing, test_ver, HW_TYPE, n_custom_cfg, phase_filters, qml_hw, qml_app, nrf_flags, FW_NAME\0, hw_crc u32 |
| 4 | GET_VALUES | voir tableau ci-dessous |
| 50 | GET_VALUES_SELECTIVE | masque u32 + champs sélectionnés |
| 5–9 | SET_DUTY / CURRENT / CURRENT_BRAKE / RPM / POS | consignes (même échelle qu'en CAN) |
| 10 | SET_HANDBRAKE | — |
| 84 | SET_CURRENT_REL | — |
| 30 | ALIVE | réarme le timeout, pas de réponse |
| 34 | FORWARD_CAN | [id cible][paquet] → relayé par CAN (§2.3) |
| 62 | PING_CAN | liste des contrôleurs présents |
| 29 | REBOOT | **seulement si le pont est désactivé** |
| 20/21 | TERMINAL_CMD / PRINT | console texte de diagnostic (nos propres commandes) |
| 159 | MOTOR_ESTOP | arrêt d'urgence |

### GET_VALUES : champs dans l'ordre des bits du masque [C]

| Bit | Champ | Type × échelle |
|---|---|---|
| 0 | T_FET filtrée | i16 ×10 |
| 1 | T_moteur filtrée | i16 ×10 |
| 2 | I moteur moyen | i32 ×100 |
| 3 | I entrée moyen | i32 ×100 |
| 4 | Id moyen | i32 ×100 |
| 5 | Iq moyen | i32 ×100 |
| 6 | duty | i16 ×1000 |
| 7 | ERPM | i32 |
| 8 | V_in | i16 ×10 |
| 9–12 | Ah, Ah chargés, Wh, Wh chargés | i32 ×1e4 |
| 13–14 | tachymètre, tachymètre abs | i32 |
| 15 | code de défaut | u8 (enum `mc_fault_code`) |
| 16 | position PID | i32 ×1e6 |
| 17 | controller_id | u8 |
| 18 | T MOS 1..3 | 3 × i16 ×10 |
| 19–20 | Vd, Vq moyens | i32 ×1000 |
| 21 | statut | u8 (b0 = timeout, b1 = kill switch) |

### Codes de défaut VESC (`mc_fault_code`) [C]

0 NONE, 1 OVER_VOLTAGE, 2 UNDER_VOLTAGE, 3 DRV, 4 ABS_OVER_CURRENT, 5 OVER_TEMP_FET, 6 OVER_TEMP_MOTOR, 7/8 GATE_DRIVER_OVER/UNDER_VOLTAGE, 9 MCU_UNDER_VOLTAGE, 10 BOOTING_FROM_WATCHDOG_RESET, 11 ENCODER_SPI, 14 FLASH_CORRUPTION, 15–17 HIGH_OFFSET_CURRENT_SENSOR_1..3, 18 UNBALANCED_CURRENTS, 19 BRK, 23/24 FLASH_CORRUPTION_APP/MC_CFG, 27 PHASE_FILTER, 28 ENCODER_FAULT, 30 ENCODER_SLIP, 31 OVERSPEED, 33 ABS_OVERSPEED…

**Nos défauts internes sont plus détaillés** (source du break, perte de synchro, ADC invalide, échéance manquée…). La couche protocole les **projette** sur ce jeu de codes. Le détail reste lisible par nos propres commandes (terminal, custom config).

## 2.5 Compatibilité VESC Tool — ce qui dépasse les messages CAN/UART

Il ne faut **pas** annoncer de compatibilité complète. Analyse :

| Fonction VESC Tool | Prérequis au-delà du protocole | Faisabilité |
|---|---|---|
| Connexion, détection | Réponse FW_VERSION cohérente. VESC Tool compare `major.minor` à sa liste ; en cas de désaccord il passe en **« limited mode »** (mise à jour firmware seulement) | [V] Choisir de déclarer 7.00 ou non est structurant (voir ci-dessous) |
| Données temps réel | GET_VALUES / SELECTIVE, GET_VALUES_SETUP (47/51) | ✅ Faisable |
| Pages de configuration moteur et app | GET/SET_MCCONF et APPCONF : sérialisation **binaire exacte** de `mc_configuration`/`app_configuration` 7.00, avec **signature** (`MCCONF_SIGNATURE`) contrôlée par les deux parties et liée au XML embarqué dans VESC Tool | ⚠ Lourd : il faut une couche de traduction entre nos paramètres et ~300 champs VESC, dont beaucoup sont sans équivalent |
| Configuration personnalisée | `COMM_GET_CUSTOM_CONFIG_XML` (92), GET/SET_CUSTOM_CONFIG (93–95) + `conf_custom_cfg_num` dans FW_VERSION : le firmware **fournit son propre XML** et VESC Tool génère les pages | ✅ **Voie recommandée** pour exposer nos paramètres natifs [V : vérifier l'accès en limited mode] |
| Assistants de détection (R, L, λ, codeur, hall) | COMM_DETECT_* (24–28, 57, 58) avec la sémantique et les résultats VESC | ⚠ Possible avec nos propres algorithmes d'identification, en mappant les résultats |
| Mise à jour firmware | Disposition flash et bootloader VESC (zone new-app, LZO, CRC) | ⚠ Nécessite un bootloader compatible. Sinon SWD/DFU |
| Graphiques expérimentaux | COMM_PLOT_* (75–78) | ✅ Facile, utile pour le diagnostic |
| QML UI, Lisp, packages, fichiers, logs | — | ❌ Exclus (Lisp/packages hors périmètre) |
| USB CDC | Même protocole paquet que l'UART | Hors périmètre initial. **Recommandé** plus tard : c'est le lien principal de VESC Tool |

**Stratégie proposée :**
1. Phase 2 : consignes, statuts CAN, GET_VALUES, FORWARD_CAN, PING. Validation avec un display et un VESC maître réels.
2. Phase 2b : custom config XML pour nos paramètres. Test avec VESC Tool en version figée (6.0x, correspondant à FW 7.00).
3. Phase 2c, optionnelle : émulation MCCONF par traduction, seulement si l'usage le justifie.

## 2.6 Couche d'abstraction

```
transport (uart_dma / bxcan)  →  vesc_packet (trame, CRC)  →  vesc_dispatch (ID → handler)
        →  motor_api (interface stable, unités SI)  →  command_arbiter  →  cœur moteur
```
- `motor_api` ne connaît **pas** VESC. Un autre protocole (CANopen, DroneCAN) peut s'y brancher sans toucher au cœur.
- `command_arbiter` gère la source active, le timeout, les rampes, le bornage par les limites utilisateur et matérielles, et la priorité des commandes d'arrêt.
