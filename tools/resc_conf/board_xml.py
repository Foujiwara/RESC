#!/usr/bin/env python3
"""
Description de configuration propre a une carte, pour la commande 250.

Part des XML du profil (RESC : firmware/conf/resc_*.xml ; VESC700 : firmware/conf/vesc700_view/)
et y inscrit, pour la carte compilee :
  - les vraies valeurs par defaut du firmware (defauts + reglages prudents de la carte + bornes),
    relevees par firmware/tests/dump_defaults.c ;
  - les plages limitees au materiel (courants, tension, ERPM, frequence PWM, duty, temperature) ;
  - une note sur le plafond de courant tant que la carte n'est pas validee au banc.
Les noms, types, encodages et choix ne changent pas : signature et serialisation identiques
(verifie). Ecrit le fichier C des tableaux compresses (qCompress) a compiler avec le firmware.

Usage : python -I board_xml.py <defauts.json> <profil vesc700|resc> <sortie.c> [<dossier des XML ecrits>]
"""
import json
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
import zlib

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CONF = os.path.join(ROOT, "firmware", "conf")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_resc_conf import crc32c  # noqa: E402
import desc_vis  # noqa: E402
import board_enum  # noqa: E402

SKIP = {"controller_id"}          # propre a chaque controleur (UID) : la valeur lue fait foi


def flat(n):
    return re.sub(r"__(\d+)", r"_\1", n).replace(".", "_")


def num(v):
    s = "%.6g" % v
    return "0" if s in ("-0", "-0.0") else s


def signature(root):
    params = root.find("Params")
    sig = ""
    for s in root.find("SerOrder"):
        e = params.find(s.text)
        vtx = e.findtext("vTx") or "0"
        sig += s.text + e.findtext("type") + vtx + "".join(x.text or "" for x in e.findall("enumNames"))
    return crc32c(sig.encode("utf-8"))


def set_text(e, tag, val):
    x = e.find(tag)
    if x is None:
        x = ET.SubElement(e, tag)
    x.text = val


def patch(root, dflt, lim, emap):
    params = root.find("Params")
    n = 0
    for s in root.find("SerOrder"):
        name = s.text
        e = params.find(name)
        v = dflt.get(flat(name))
        if v is None or name in SKIP:
            continue
        t = int(e.findtext("type"))
        m = emap.get(flat(name))
        if m:
            iv = int(round(v))
            if m["bits"]:
                v = sum(1 << j for j, k in enumerate(m["keep"]) if iv & (1 << k))
            else:
                v = m["keep"].index(iv) if iv in m["keep"] else 0
        if t == 1:
            set_text(e, "valDouble", num(v))
        else:
            set_text(e, "valInt", str(int(round(v))))
        n += 1

    def rng(name, lo=None, hi=None):
        e = params.find(name)
        if e is None:
            return
        integer = e.findtext("type") != "1"
        if lo is not None:
            set_text(e, "minInt" if integer else "minDouble", str(int(lo)) if integer else num(lo))
        if hi is not None:
            set_text(e, "maxInt" if integer else "maxDouble", str(int(hi)) if integer else num(hi))

    rng("l_current_max", 0, lim["i_motor"])
    rng("l_current_min", -lim["i_motor"], 0)
    rng("l_in_current_max", 0, lim["i_input"])
    rng("l_in_current_min", -lim["i_input"], 0)
    rng("l_abs_current_max", 1, lim["i_abs"])
    rng("l_max_vin", lim["vbus_min"] + 1, lim["vbus_max"])
    rng("l_min_vin", lim["vbus_min"], lim["vbus_max"] - 1)
    rng("l_max_erpm", 0, lim["erpm"])
    rng("l_min_erpm", -lim["erpm"], 0)
    rng("foc_f_zv", 2 * lim["f_pwm_min"], 2 * lim["f_pwm_max"])
    rng("l_max_duty", 0.1, lim["duty_max"])
    rng("l_temp_fet_end", 0, lim["t_fet"])
    rng("l_temp_fet_start", 0, lim["t_fet"])
    if lim["cap"] < 0.999:
        note = ("<p><b>Board not yet validated on the bench: the firmware caps the effective current to "
                "%d %% of the hardware limit (%s A motor, %s A absolute).</b></p>" %
                (round(lim["cap"] * 100), num(lim["i_motor"] * lim["cap"]), num(lim["i_abs"] * lim["cap"])))
        for name in ("l_current_max", "l_current_min", "l_abs_current_max", "l_in_current_max"):
            e = params.find(name)
            if e is not None:
                set_text(e, "description", note + (e.findtext("description") or ""))
    return n


def main():
    if len(sys.argv) < 4:
        raise SystemExit(__doc__)
    data = json.load(open(sys.argv[1]))
    prof, out = sys.argv[2], sys.argv[3]
    files = {"mc": "resc_mcconf.xml", "app": "resc_appconf.xml"} if prof == "resc" else \
            {"mc": "vesc700_view/parameters_mcconf.xml", "app": "vesc700_view/parameters_appconf.xml"}
    lines = ["/* GENERE par tools/resc_conf/board_xml.py pour %s (profil %s) - ne pas editer. */" % (data["board"], prof),
             '#include "hw_config.h"', '#include "vesc_conf.h"', '#include "conf_desc.h"', '']
    for k, f in files.items():
        root = ET.parse(os.path.join(CONF, f)).getroot()
        emap = json.load(open(os.path.join(CONF, "resc_enum_map.json"))) if prof == "resc" else {}
        if prof == "resc":
            # choix absents sur cette carte : retires de la liste (signature et table de la carte)
            emap = board_enum.reduce(root, emap, set(data.get("caps", [])), k)
        sig0 = signature(root)
        n = patch(root, data[k], data["limits"], emap)
        if signature(root) != sig0:
            raise SystemExit("signature modifiee : interdit")
        if prof == "resc":
            lines += board_enum.c_lines(k, emap, sig0)
        ET.indent(root, space="    ")
        raw = ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"
        if len(sys.argv) > 4:
            os.makedirs(sys.argv[4], exist_ok=True)
            open(os.path.join(sys.argv[4], os.path.basename(f)), "wb").write(raw)
        vis = (desc_vis.VIS_MC if k == "mc" else desc_vis.VIS_APP) if prof == "resc" else {}
        lines += desc_vis.c_lines(raw, root, k, vis)
        print("  %s : %d valeurs par defaut de la carte, XML %d octets, %d parametres a affichage conditionnel" % (
            k, n, len(raw), len(vis)))
    open(out, "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
