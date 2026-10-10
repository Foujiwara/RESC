#!/usr/bin/env python3
"""
Verifie toutes les descriptions de carte de hw/boards sans compilateur :
  1. evalue hw_config.h (donc hw_check.h) pour chaque carte -> #error / #warning
  2. detecte les broches affectees deux fois (hors partages connus et voulus)
  3. affiche un resume (statut, protection, topologie, limites)

Usage : python -I tools/hw_import/check_boards.py [repertoire_racine_du_projet] [--quiet]
Code de sortie : 1 si au moins une carte REVIEWED/VALIDATED a une erreur.
"""
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from minipp import MiniPP  # noqa: E402

PORTS = "ABCDE"
ADC_CH_PIN = {**{i: 0x00 + i for i in range(8)}, 8: 0x10, 9: 0x11, **{i: 0x20 + i - 10 for i in range(10, 16)}}
STATUS = {0: "IMPORTED", 1: "REVIEWED", 2: "VALIDATED"}
OCP = {0: "aucune", 1: "driver interne", 2: "GPIO", 3: "BKIN"}
CS = {0: "2 low-side", 1: "3 low-side", 2: "3 en ligne"}

# Partages voulus : fonctions du connecteur COMM, mutuellement exclusives
# (l'application configuree en choisit une), et sondes FET multiples lues
# sur la meme voie. Tout autre partage est une erreur.
COMM_PORT = {"BOARD_UART_TX_PIN", "BOARD_UART_RX_PIN", "BOARD_I2C_SCL_PIN", "BOARD_I2C_SDA_PIN",
             "BOARD_UART2_TX_PIN", "BOARD_UART2_RX_PIN", "BOARD_SPI_SCK_PIN", "BOARD_SPI_MISO_PIN",
             "BOARD_SPI_MOSI_PIN", "BOARD_SPI_NSS_PIN", "BOARD_ADC_EXT1_CH", "BOARD_ADC_EXT2_CH",
             "BOARD_ADC_EXT3_CH", "BOARD_SERVO_PIN"}
SHARED_OK = [COMM_PORT, {"BOARD_TFET_CH", "BOARD_TFET2_CH", "BOARD_TFET3_CH"}]


def pin_name(p):
    return f"P{PORTS[p >> 4]}{p & 15}" if p is not None and p != 0xFF else "-"


def check(root, header_rel):
    hw = os.path.join(root, "hw")
    pp = MiniPP(include_dirs=[hw], predefined={"RESC_BOARD_HEADER": f'"{header_rel}"'})
    try:
        pp.run_file(os.path.join(hw, "hw_config.h"))
    except Exception as exc:  # erreur d'evaluation = erreur de carte
        pp.errors.append((header_rel, 0, f"evaluation impossible : {exc}"))
    users = defaultdict(set)
    for name in pp.macros:
        if name.startswith("BOARD_") and (name.endswith("_PIN") or name.endswith("_CH")):
            v = pp.value(name)
            if not isinstance(v, int):
                continue
            if name.endswith("_CH"):
                if v not in ADC_CH_PIN:
                    continue
                v = ADC_CH_PIN[v]
            elif v == 0xFF:
                continue
            elif v > 0x2F and v != 0x32:
                pp.errors.append((header_rel, 0, f"{name} = {pin_name(v)} absent du boitier LQFP64"))
            users[v].add(name)
    dups = []
    for p, names in users.items():
        # Les macros ADC d'une meme voie (CH) et leur broche peuvent apparaitre via l'alias commun
        names = set(names)
        for ok in SHARED_OK:
            inside = names & ok
            if len(inside) > 1:
                names = (names - ok) | {sorted(inside)[0]}
        if len(names) > 1:
            dups.append(f"{pin_name(p)}: {', '.join(sorted(names))}")
    return pp, dups


def main():
    root = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else ".")
    quiet = "--quiet" in sys.argv
    boards_dir = os.path.join(root, "hw", "boards")
    headers = []
    for d, _, files in os.walk(boards_dir):
        for f in files:
            if f.startswith("board_") and f.endswith(".h") and not f.endswith("_core.h") and "common" not in d:
                headers.append(os.path.relpath(os.path.join(d, f), os.path.join(root, "hw")).replace("\\", "/"))
    bad_reviewed, n_err = 0, 0
    for h in sorted(headers):
        pp, dups = check(root, h)
        st = STATUS.get(pp.value("BOARD_STATUS"), "?")
        nm = pp.macros.get("BOARD_NAME", (None, "?"))[1]
        if pp.errors or dups:
            n_err += 1
            if st != "IMPORTED":
                bad_reviewed += 1
        if quiet and not pp.errors and not dups and st == "IMPORTED":
            continue
        print(f"{h}  {nm}  [{st}]  OCP={OCP.get(pp.value('BOARD_HW_OCP'), '?')}  "
              f"CS={CS.get(pp.value('BOARD_CS_TOPOLOGY'), '?')}  "
              f"I_abs={pp.value('HWLIM_I_ABS_TRIP_A')} A  Vbus={pp.value('HWLIM_VBUS_MIN_V')}-{pp.value('HWLIM_VBUS_MAX_V')} V  "
              f"f_pwm={pp.value('DEF_F_PWM_HZ')} Hz")
        for f, ln, msg in pp.errors:
            print(f"    ERREUR  {os.path.basename(f)}:{ln}  {msg}")
        for dup in dups:
            print(f"    DOUBLON {dup}")
        if not quiet:
            for f, ln, msg in pp.warnings:
                print(f"    note    {os.path.basename(f)}:{ln}  {msg}")
    print(f"\n{len(headers)} cartes, {n_err} avec erreurs/doublons, dont {bad_reviewed} non IMPORTED")
    sys.exit(1 if bad_reviewed else 0)


if __name__ == "__main__":
    main()
