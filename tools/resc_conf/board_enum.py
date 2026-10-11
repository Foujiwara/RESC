#!/usr/bin/env python3
"""
Choix de configuration qui dependent de la carte (profil RESC).

Certains choix n'existent que sur une partie du materiel : V0 et V7 (boucle a deux fois la
frequence PWM) demande des shunts en ligne ; les modes de mesure qui combinent trois courants
demandent trois capteurs ; les codeurs SPI1, PWM et sin/cos demandent leurs broches ; l'IMU
interne n'existe que sur certaines cartes. La description de la carte (board_xml.py) retire ces
choix de la liste quand la carte ne les a pas ; la signature et la traduction position <-> valeur
de la carte en tiennent compte (g_rescp_mcconf_sig, g_rescp_appconf_sig, g_keepb_*, compiles
avec le firmware ; valeurs generiques faibles dans vesc_conf.c).

Capacites (relevees par firmware/tests/dump_defaults.c) :
  inline : courants mesures en permanence (shunts en ligne ou de phase)
  three  : trois capteurs de courant
  spi    : broches SPI1 du port COMM
  servo  : entree servo sur TIM4 (libre du codeur ABI)
  adc2   : deux entrees ADC externes (sin/cos)
  imu    : IMU interne (I2C logiciel de la carte)
  i2c    : I2C sur les broches du port UART (IMU externe)
  rs485  : UART du port COMM et entree ADC 1 (sens du transceiver, TS5700N8501)
"""
import copy
import re
import xml.etree.ElementTree as ET

# nom XML -> { valeur interne : capacite exigee }
BOARD_ENUM = {
    "mc": {
        "foc_control_sample_mode": {1: "inline"},
        "foc_current_sample_mode": {1: "three", 2: "three"},
        "m_sensor_port_mode": {3: "spi", 7: "spi", 8: "spi", 9: "spi", 11: "spi", 15: "spi", 13: "servo",
                               14: "servo", 4: "adc2", 5: "rs485", 6: "rs485"},
    },
    "app": {
        "imu_conf.type": {1: "imu", 4: "i2c"},
    },
}


def flat(n):
    return re.sub(r"__(\d+)", r"_\1", n).replace(".", "_")


FLAT = {k: {flat(n) for n in t} for k, t in BOARD_ENUM.items()}
ALL_FLAT = FLAT["mc"] | FLAT["app"]


def reduce(root, emap, caps, k):
    """Retire de <Params> les choix non pris en charge par la carte. Retourne la table
    position <-> valeur de la carte (copie de emap)."""
    emap = copy.deepcopy(emap)
    params = root.find("Params")
    for field, need in BOARD_ENUM[k].items():
        e = params.find(field)
        f = flat(field)
        if e is None or f not in emap:
            continue
        keep = emap[f]["keep"]
        new = [v for v in keep if need.get(v) is None or need[v] in caps]
        if not new:
            new = keep[:1]
        if new == keep:
            continue
        names = [x.text for x in e.findall("enumNames")]
        old_def = keep[int(e.findtext("valInt") or "0")]
        for x in e.findall("enumNames"):
            e.remove(x)
        for v in new:
            ET.SubElement(e, "enumNames").text = names[keep.index(v)]
        e.find("valInt").text = str(new.index(old_def) if old_def in new else 0)
        if e.find("maxInt") is not None:
            e.find("maxInt").text = str(len(new) - 1)
        emap[f] = {"bits": emap[f]["bits"], "keep": new}
    return emap


def c_lines(k, emap, sig, weak=False):
    """Variables de la carte pour la configuration k (mc ou app) : signature et listes."""
    w = "__attribute__((weak)) " if weak else ""
    name = "mcconf" if k == "mc" else "appconf"
    L = ["/* choix dependant de la carte (%s, tools/resc_conf/board_enum.py) */" % name,
         "%sconst uint32_t g_rescp_%s_sig = %uu;" % (w, name, sig)]
    for f in sorted(FLAT[k]):
        keep = emap[f]["keep"]
        L.append("%sconst uint8_t g_keepb_%s[] = { %s };" % (w, f, ", ".join(str(v) for v in keep)))
        L.append("%sconst uint8_t g_keepb_%s_n = %d;" % (w, f, len(keep)))
    return L
