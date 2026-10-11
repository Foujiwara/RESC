#!/usr/bin/env python3
"""
Audit des parametres de configuration : pour chaque profil, liste les parametres serialises
que le firmware ne lit pas (inutilises) et ceux qui ne sont ranges dans aucun onglet (invisibles).

Usage : python -I tools/resc_conf/audit_params.py [--strict]
  --strict : code de sortie 1 si un parametre RESC est inutilise mais visible, ou utilise mais invisible.
"""
import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
GENERATED = ("vesc_conf.c", "vesc_conf.h", "conf_xml.c", "vesc_defaults.h", "vesc_compat_defaults.h")
# lus par l'outil seulement (affichage vitesse/batterie, assistant) : a garder visibles
TOOL_ONLY = {"si_gear_ratio", "si_wheel_diameter", "si_battery_type", "si_battery_cells", "si_battery_ah",
             "si_motor_nl_current"}
# transmis mais volontairement invisibles (lus par l'outil pour choisir ses pages)
HIDDEN_OK = {"motor_type"}


def firmware_source():
    txt = []
    for f in glob.glob(os.path.join(ROOT, "firmware", "src", "**", "*.[ch]"), recursive=True):
        if os.path.basename(f) in GENERATED:
            continue
        t = open(f, encoding="utf-8", errors="replace").read()
        t = re.sub(r"/\*.*?\*/", "", t, flags=re.S)       # commentaires
        t = re.sub(r"//[^\n]*", "", t)
        t = re.sub(r'"(?:\\.|[^"\\])*"', '""', t)        # chaines (listes du terminal) : pas un usage
        t = re.sub(r"\bFIN\(\w+\);", "", t)               # remplacement des NaN : pas un usage
        txt.append(t)
    return "\n".join(txt)


def flat(n):
    return re.sub(r"__(\d+)", r"_\1", n).replace(".", "_")


def audit(path, src):
    r = ET.parse(path).getroot()
    ser = [s.text for s in r.find("SerOrder")]
    grouped = {p.text for p in r.iter("param")}
    # usage = acces a un champ de la configuration (g_mcconf.x, mc->x, c->x, m->x, a->x, app->x)
    acc = r"(?:\bg_mcconf|\bg_appconf|\bmc|\bc|\bm|\ba|\bapp|\btmp|\bz)\s*(?:->|\.)\s*%s\b"
    used = {n for n in ser if n in TOOL_ONLY or re.search(acc % re.escape(flat(n)), src)}
    return ser, used, grouped


def main():
    strict = "--strict" in sys.argv
    src = firmware_source()
    bad = 0
    for prof, files in (("vesc700", ("vesc700/parameters_mcconf.xml", "vesc700/parameters_appconf.xml")),
                        ("vesc700 (vue RESC Tool)", ("vesc700_view/parameters_mcconf.xml", "vesc700_view/parameters_appconf.xml")),
                        ("resc", ("resc_mcconf.xml", "resc_appconf.xml"))):
        for f in files:
            p = os.path.join(ROOT, "firmware", "conf", f)
            if not os.path.exists(p):
                continue
            ser, used, grouped = audit(p, src)
            unused_visible = [n for n in ser if n not in used and n in grouped]
            used_hidden = [n for n in ser if n in used and n not in grouped and n not in HIDDEN_OK]
            print("%-24s %-26s %3d serialises, %3d utilises" % (prof, os.path.basename(f), len(ser), len(used)))
            print("    inutilises mais visibles :", ", ".join(unused_visible) or "aucun")
            print("    utilises mais invisibles :", ", ".join(used_hidden) or "aucun")
            if prof != "vesc700":
                bad += len(unused_visible) + len(used_hidden)
    return 1 if strict and bad else 0


if __name__ == "__main__":
    sys.exit(main())
