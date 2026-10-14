#!/usr/bin/env python3
"""
bench_reprise.py — Essai de reprise au vol au banc (RESC 1.8), alimentation stabilisee.

  python tools/bench_reprise.py                 port USB detecte, reglages par defaut
  python tools/bench_reprise.py COM5 --erpm 5000,10000,15000 --obs 3,8

Contraintes du banc, appliquees par construction :
  - JAMAIS DE FREINAGE : courant moteur minimal 0 A, regeneration batterie 0 A, aucune commande 'brake' ni
    'duty' (le mode duty freine en descente) ; le moteur n'est que lance (courant positif) puis relache.
  - TENSION : refus au-dessus de 48 V au depart ; coupure du firmware reglee a 50 V ; arret si le bus depasse 49 V.
  - VITESSE : plafond 22000 ERPM. Au-dela de ~29700 ERPM sous 48 V (moteur 8.9 mWb, 7 paires de poles), la FEM
    entre phases depasse le bus : les diodes du pont conduisent meme pont coupe et le moteur renvoie du courant
    vers l'alimentation (= freinage).
  - Aucun reglage n'est enregistre en flash (jamais de 'store') : les valeurs d'origine sont remises a la fin, et
    un redemarrage de la carte les rend de toute facon.

Deroulement, pour chaque observateur (--obs) et chaque vitesse (--erpm, croissantes) :
  lancer le moteur a --ispin A jusqu'a la vitesse, relacher (roue libre), mesurer les diviseurs de phase
  (vph_check), armer l'enregistreur, reprendre a --ireq A pendant 0.3 s, relacher, lire l'enregistrement
  (256 periodes autour de la reprise) et les defauts.
Tout est ecrit dans bench_reprise_<date>.txt (a renvoyer tel quel). Ctrl+C : relache le moteur et restaure.

Avant : moteur seul (sans charge ni roue) solidement fixe, accelerateur / recepteur debranches, alimentation
36 a 48 V limitee en courant (10 A suffisent). Necessite pyserial.
"""
import argparse
import os
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from resc_cli import COMM_FW_VERSION, COMM_GET_VALUES, COMM_PRINT, COMM_TERMINAL_CMD, Link, find_port  # noqa: E402

ERPM_CAP = 22000.0
VBUS_START_MAX = 48.5
VBUS_ABORT = 49.0

# reglages de securite (RAM seulement) ; valeurs d'origine relevees puis remises
SAFE = [
    ("l_current_min", 0.0),          # aucun courant de freinage
    ("l_in_current_min", 0.0),       # aucune regeneration vers l'alimentation
    ("l_max_vin", 50.0),             # coupure de surtension a 50 V
    ("l_current_max", 15.0),
    ("l_in_current_max", 10.0),      # courant pris a l'alimentation
    ("app_to_use", 0.0),             # aucune application (accelerateur) ne commande
    ("l_min_vin", 11.0),             # batterie 72 V configuree : sinon le courant est coupe a 48 V
    ("l_battery_cut_start", 13.0),
    ("l_battery_cut_end", 12.0),
]


class Bench:
    def __init__(self, port, log):
        self.link = Link(port)
        self.log = log
        self.lines = []
        self.lock = threading.Lock()
        self.values = None
        self.fw = ""
        self.link.on_packet = self._on_packet

    def _on_packet(self, pl):
        if pl[0] == COMM_PRINT:
            t = pl[1:].decode("latin-1")
            with self.lock:
                self.lines.append(t)
            self.out(t, echo=False)
        elif pl[0] == COMM_GET_VALUES:
            self.values = self._decode(pl)
        elif pl[0] == COMM_FW_VERSION:
            name_end = pl.index(0, 3)
            self.fw = f"{pl[1]}.{pl[2]:02d} sur {pl[3:name_end].decode('latin-1')}"

    @staticmethod
    def _decode(pl):
        i = 1 + 2 + 2 + 4 * 4                      # t_fet, t_mot, i_mot, i_in, id, iq
        duty = struct.unpack(">h", pl[i:i + 2])[0] / 1000.0
        erpm = struct.unpack(">i", pl[i + 2:i + 6])[0]
        vin = struct.unpack(">h", pl[i + 6:i + 8])[0] / 10.0
        i_mot = struct.unpack(">i", pl[5:9])[0] / 100.0
        return {"erpm": float(erpm), "vin": vin, "duty": duty, "i_mot": i_mot}

    def out(self, t, echo=True):
        self.log.write(t + "\n")
        self.log.flush()
        if echo:
            print(t)

    def cmd(self, line, wait=0.4, quiet=0.25):
        """Commande terminal ; rend les lignes recues jusqu'a un silence de 'quiet' s (au plus 'wait' + 5 s)."""
        with self.lock:
            self.lines.clear()
        self.out("> " + line)
        self.link.send([COMM_TERMINAL_CMD] + list(line.encode("latin-1")))
        t0 = time.time()
        time.sleep(wait)
        last_n, last_t = -1, time.time()
        while time.time() - t0 < wait + 5.0:
            with self.lock:
                n = len(self.lines)
            if n != last_n:
                last_n, last_t = n, time.time()
            elif time.time() - last_t > quiet:
                break
            time.sleep(0.05)
        with self.lock:
            got = list(self.lines)
        for g in got:
            print("  " + g)
        return got

    def read_values(self):
        self.values = None
        self.link.send([COMM_GET_VALUES])
        t0 = time.time()
        while self.values is None and time.time() - t0 < 0.5:
            time.sleep(0.01)
        return self.values

    def get_param(self, name):
        for g in self.cmd("get " + name, wait=0.2):
            parts = g.split("=")
            if len(parts) == 2 and parts[0].strip() == name:
                return float(parts[1])
        return None

    def check_vbus(self):
        v = self.read_values()
        if v and v["vin"] > VBUS_ABORT:
            raise RuntimeError(f"bus a {v['vin']:.1f} V > {VBUS_ABORT} V")
        return v


def parse_dump(lines):
    """Lignes CSV de fault_dump -> resume (crete de phase, plages id / iq apres la reprise)."""
    rows = []
    for g in lines:
        f = g.split(",")
        if len(f) == 13:
            try:
                rows.append([float(x) for x in f])
            except ValueError:
                pass
    after = [r for r in rows if r[0] >= 0]
    if not after:
        return None
    pk = max(max(abs(r[1]), abs(r[2]), abs(r[3])) for r in after)
    return {"n": len(rows), "peak": pk, "id": (min(r[4] for r in after), max(r[4] for r in after)),
            "iq": (min(r[5] for r in after), max(r[5] for r in after))}


def main():
    ap = argparse.ArgumentParser(description="Essai de reprise au vol au banc (sans freinage, 50 V max)")
    ap.add_argument("port", nargs="?")
    ap.add_argument("--erpm", default="5000,10000,15000,20000", help="vitesses de reprise (ERPM), croissantes")
    ap.add_argument("--obs", default="3,8", help="observateurs essayes (foc_observer_type)")
    ap.add_argument("--resume", default="0", help="modes de reprise essayes (resume_mode 0 et/ou 1)")
    ap.add_argument("--ispin", type=float, default=5.0, help="courant de lancement (A)")
    ap.add_argument("--ireq", type=float, default=2.0, help="courant demande a la reprise (A)")
    ap.add_argument("--nrep", type=int, default=2, help="reprises par vitesse")
    a = ap.parse_args()

    speeds = sorted(float(x) for x in a.erpm.split(","))
    if max(speeds) > ERPM_CAP:
        sys.exit(f"Refuse : {max(speeds):.0f} ERPM > plafond {ERPM_CAP:.0f} (freinage par les diodes au-dela de ~29700 a 48 V)")
    if a.ispin > 10 or a.ireq > 10:
        sys.exit("Refuse : courants limites a 10 A pour l'essai")
    port = a.port or find_port()
    if not port:
        sys.exit("Aucun port RESC detecte (indiquer le port, ex. COM5)")

    name = time.strftime("bench_reprise_%Y%m%d_%H%M%S.txt")
    log = open(name, "w", encoding="utf-8")
    b = Bench(port, log)
    b.link.send([COMM_FW_VERSION])
    time.sleep(0.4)
    b.out(f"Firmware {b.fw} ; port {port} ; essai {time.strftime('%Y-%m-%d %H:%M:%S')}")
    hw = b.cmd("hw")
    if not any("RESC 1.8" in g for g in hw):
        sys.exit("Refuse : ce script demande RESC 1.8 (commandes vph_check, resume_mode, fault_dump arm ; voir 'hw')")
    v = b.read_values()
    if not v:
        sys.exit("Pas de mesures de la carte")
    b.out(f"Bus {v['vin']:.1f} V")
    if v["vin"] > VBUS_START_MAX or v["vin"] < 12.0:
        sys.exit(f"Refuse : bus a {v['vin']:.1f} V (attendu 12 a 48 V)")
    if abs(v["erpm"]) > 300:
        sys.exit("Refuse : le moteur tourne deja")

    orig = {}
    summary = []
    try:
        b.cmd("faults")
        for p in ("foc_observer_type", "l_max_erpm") + tuple(n for n, _ in SAFE):
            orig[p] = b.get_param(p)
        b.out("Valeurs d'origine : " + ", ".join(f"{k}={v}" for k, v in orig.items()))
        for n, val in SAFE:
            b.cmd(f"set {n} {val:g}", wait=0.3)
        b.cmd(f"set l_max_erpm {min(max(speeds) * 1.1, ERPM_CAP):.0f}", wait=0.3)
        b.cmd("reset")

        for obs in [int(x) for x in a.obs.split(",")]:
            b.cmd(f"set foc_observer_type {obs}", wait=0.3)
            for mode in [int(x) for x in a.resume.split(",")]:
                b.cmd(f"resume_mode {mode}", wait=0.2)
                for target in speeds:
                    for rep in range(a.nrep):
                        b.out(f"\n=== observateur {obs}, reprise {mode}, {target:.0f} ERPM, essai {rep + 1}")
                        # moteur encore trop rapide (essai precedent) : roue libre jusqu'a 90 % de la cible, sans freiner
                        t0 = time.time()
                        while True:
                            v = b.check_vbus()
                            if not v or v["erpm"] <= 0.9 * target:
                                break
                            if time.time() - t0 > 180:
                                raise RuntimeError("le moteur ne ralentit pas en roue libre (180 s)")
                            if int(time.time() - t0) % 10 == 0:
                                print(f"  roue libre : {v['erpm']:.0f} ERPM, attente sous {0.9 * target:.0f}", end="\r")
                            time.sleep(0.2)
                        # lancement (courant positif seulement)
                        b.cmd(f"current {a.ispin:g}", wait=0.1)
                        t0 = time.time()
                        while True:
                            v = b.check_vbus()
                            if v and v["erpm"] >= target:
                                break
                            if time.time() - t0 > 20:
                                b.cmd("stop", wait=0.1)
                                raise RuntimeError(f"vitesse {target:.0f} non atteinte en 20 s (ERPM {v and v['erpm']})")
                            time.sleep(0.05)
                        b.cmd("stop", wait=0.3)                    # roue libre (observateur sur les tensions mesurees)
                        v = b.check_vbus()
                        erpm0 = v["erpm"] if v else 0.0
                        b.cmd("fault_dump arm", wait=0.05)
                        b.cmd(f"current {a.ireq:g}", wait=0.3)    # reprise
                        b.cmd("stop", wait=0.3)
                        b.cmd("vph_check", wait=0.05)              # diviseurs de phase, pendant la roue libre qui suit
                        time.sleep(0.6)
                        b.cmd("vph_check", wait=0.2)
                        dump = b.cmd("fault_dump", wait=1.0, quiet=0.6)
                        flt = b.cmd("faults", wait=0.3)
                        r = parse_dump(dump)
                        fault = next((g for g in flt if g.startswith("Defaut actif")), "")
                        if "aucun" not in fault:
                            b.cmd("clear", wait=0.3)
                        summary.append((obs, mode, target, erpm0, r, fault))
                        b.check_vbus()
                        time.sleep(0.5)
        b.cmd("hw")
        b.cmd("prof")
    except KeyboardInterrupt:
        b.out("\nInterrompu (Ctrl+C)")
    except RuntimeError as e:
        b.out(f"\nARRET : {e}")
    finally:
        b.link.send([COMM_TERMINAL_CMD] + list(b"stop"))
        time.sleep(0.5)
        for _ in range(100):                            # attendre l'arret avant de restaurer
            v = b.read_values()
            if not v or abs(v["erpm"]) < 300:
                break
            time.sleep(0.2)
        for k, val in orig.items():
            if val is not None:
                b.cmd(f"set {k} {val:g}", wait=0.3)
        b.cmd("resume_mode 0", wait=0.2)
        b.out("\n=== Resume (crete de phase et plages apres la reprise, enregistreur)")
        b.out("obs mode   cible   ERPM  | crete (A) | id min..max   | iq min..max   | defaut")
        for obs, mode, target, erpm0, r, fault in summary:
            if r:
                b.out(f"{obs:3d} {mode:4d} {target:7.0f} {erpm0:7.0f} | {r['peak']:8.1f}  | {r['id'][0]:6.1f}..{r['id'][1]:5.1f} | "
                      f"{r['iq'][0]:6.1f}..{r['iq'][1]:5.1f} | {fault}")
            else:
                b.out(f"{obs:3d} {mode:4d} {target:7.0f} {erpm0:7.0f} | pas d'enregistrement | {fault}")
        b.out(f"\nReglages d'origine remis (rien n'a ete enregistre en flash). Journal : {name}")
        b.link.alive = False
        log.close()


if __name__ == "__main__":
    main()
