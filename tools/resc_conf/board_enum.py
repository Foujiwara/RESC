#!/usr/bin/env python3
"""
Choix de configuration qui dependent de la carte (profil RESC).

Certains choix n'existent que sur une partie du materiel : V0 et V7 (boucle a deux fois la
frequence PWM) demande des shunts en ligne ; les modes de mesure qui combinent trois courants
demandent trois capteurs. La description de la carte (board_xml.py) retire ces choix de la
liste quand la carte ne les a pas ; la signature et la traduction position <-> valeur de la
carte en tiennent compte (variables g_rescp_mcconf_sig, g_keepb_* compilees avec le firmware).

Capacites (relevees par firmware/tests/dump_defaults.c) :
  inline : courants mesures en permanence (shunts en ligne ou de phase)
  three  : trois capteurs de courant
  spi    : broches SPI1 du port COMM
  servo  : entree servo sur TIM4 (libre du codeur ABI)
  adc2   : deux entrees ADC externes (sin/cos)
"""
import copy

# champ -> { valeur interne : capacite exigee }
BOARD_ENUM = {
    "foc_control_sample_mode": {1: "inline"},
    "foc_current_sample_mode": {1: "three", 2: "three"},
    "m_sensor_port_mode": {3: "spi", 7: "spi", 8: "spi", 15: "spi", 13: "servo", 4: "adc2"},
}


def kept(field, keep, caps):
    need = BOARD_ENUM.get(field, {})
    return [v for v in keep if need.get(v) is None or need[v] in caps]


def reduce(root, emap, caps):
    """Retire de <Params> les choix non pris en charge par la carte. Retourne la table
    position <-> valeur de la carte (copie de emap)."""
    emap = copy.deepcopy(emap)
    params = root.find("Params")
    for field in BOARD_ENUM:
        e = params.find(field)
        if e is None or field not in emap:
            continue
        keep = emap[field]["keep"]
        new = kept(field, keep, caps)
        if new == keep:
            continue
        names = [x.text for x in e.findall("enumNames")]
        old_def = keep[int(e.findtext("valInt") or "0")]
        for x in e.findall("enumNames"):
            e.remove(x)
        import xml.etree.ElementTree as ET
        for v in new:
            ET.SubElement(e, "enumNames").text = names[keep.index(v)]
        e.find("valInt").text = str(new.index(old_def) if old_def in new else 0)
        if e.find("maxInt") is not None:
            e.find("maxInt").text = str(len(new) - 1)
        emap[field] = {"bits": emap[field]["bits"], "keep": new}
    return emap


def c_lines(emap, sig, weak=False):
    """Variables de la carte : signature RESC mcconf et listes des champs ci-dessus."""
    w = "__attribute__((weak)) " if weak else ""
    L = ["/* choix dependant de la carte (tools/resc_conf/board_enum.py) */",
         "%sconst uint32_t g_rescp_mcconf_sig = %uu;" % (w, sig)]
    for field in BOARD_ENUM:
        keep = emap[field]["keep"]
        L.append("%sconst uint8_t g_keepb_%s[] = { %s };" % (w, field, ", ".join(str(v) for v in keep)))
        L.append("%sconst uint8_t g_keepb_%s_n = %d;" % (w, field, len(keep)))
    return L
