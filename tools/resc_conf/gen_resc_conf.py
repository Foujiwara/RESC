#!/usr/bin/env python3
"""
Genere toute la couche de configuration de RESC a partir des descriptions XML (format
ConfigParams de VESC Tool), pour les deux profils :

  VESC700 : firmware/conf/vesc700/parameters_{mcconf,appconf}.xml  (officiels 7.00, non modifies)
  RESC    : firmware/conf/resc_{mcconf,appconf}.xml                 (parametres propres a RESC)

Sorties (ne pas editer a la main) :
  firmware/src/vesc/vesc_conf.h   structures internes (union des deux profils), signatures,
                                  table des champs (stockage par nom, terminal), prototypes
  firmware/src/vesc/vesc_conf.c   defauts, serialisation / deserialisation des deux profils,
                                  table des champs
  firmware/src/vesc/conf_xml.c    XML compresses (qCompress) du profil compile, commande 250

Signature (identique a ConfigParams::getSignature de VESC Tool) :
  crc32c( pour chaque nom de <SerOrder> : nom + str(type) + str(vTx) + enumNames concatenes )
La generation s'arrete si la signature VESC700 calculee differe de la valeur de reference.

Types C : ceux du firmware de reference pour les champs VESC 7.00 (vesc700_ctypes.json),
sinon deduits du XML. Defauts : expressions de reference (vesc700_defaults.json) pour les
champs VESC 7.00, valeur du XML RESC pour les autres ; dans le profil RESC, les champs dont
le defaut RESC differe du defaut VESC prennent le defaut RESC.

Usage : python -I tools/resc_conf/gen_resc_conf.py
"""
import json
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import desc_vis  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HERE = os.path.dirname(os.path.abspath(__file__))
CONF = os.path.join(ROOT, "firmware", "conf")
OUT = os.path.join(ROOT, "firmware", "src", "vesc")

REF_SIG = {"mc": 1470992211, "app": 2384027283}
VTX_NONE = 0


def crc32c(data):
    c = 0xFFFFFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ (0x82F63B78 & -(c & 1))
    return (~c) & 0xFFFFFFFF


def crc32_name(s):
    return zlib.crc32(s.encode("utf-8")) & 0xFFFFFFFF


def flat(name):
    return re.sub(r"__(\d+)", r"_\1", name).replace(".", "_")


class Param:
    def __init__(self, name, e):
        self.name = name
        self.c = flat(name)
        self.type = int(e.findtext("type"))
        vtx = e.findtext("vTx")
        self.vtx = int(vtx) if vtx else VTX_NONE
        sc = e.findtext("vTxDoubleScale")
        self.scale = float(sc) if sc else 1.0
        self.enums = [x.text or "" for x in e.findall("enumNames")]
        self.val_double = e.findtext("valDouble")
        self.val_int = e.findtext("valInt")
        self.cdefine = e.findtext("cDefine")


def load(path):
    raw = open(path, "rb").read()
    root = ET.fromstring(raw)
    params = root.find("Params")
    order = [s.text for s in root.find("SerOrder")]
    ps = [Param(n, params.find(n)) for n in order]
    sig = "".join(p.name + str(p.type) + str(p.vtx) + "".join(p.enums) for p in ps)
    return raw, ps, crc32c(sig.encode("utf-8"))


def ctype_from_xml(p):
    if p.type == 1:
        return "float"
    if p.type in (4, 5, 6):
        return "uint8_t"
    return {1: "uint8_t", 2: "int8_t", 3: "uint16_t", 4: "int16_t", 5: "uint32_t", 6: "int32_t"}.get(p.vtx, "int32_t")


def fmt_scale(s):
    return ("%g" % s) if s != int(s) else str(int(s))


EMAP = {}     # profil RESC : listes reduites (resc_enum_map.json)


def ser_line(p, ct, rmap=False):
    c = "c->" + p.c
    m = EMAP.get(p.c) if rmap else None
    if m:
        f = "map_bits_out" if m["bits"] else "map_out"
        return "buf[i++] = %s((uint8_t)%s, KEEP_%s, %d);" % (f, c, p.c.upper(), len(m["keep"]))
    if p.type == 1:
        if p.vtx == 7:
            return "vb_put_f16(buf, %s, %s, &i);" % (c, fmt_scale(p.scale))
        if p.vtx == 8:
            return "vb_put_f32(buf, %s, %s, &i);" % (c, fmt_scale(p.scale))
        if p.vtx == 9:
            return "vb_put_f32_auto(buf, %s, &i);" % c
    if p.vtx in (VTX_NONE, 1, 2) or p.type in (4, 5, 6):
        return "buf[i++] = (uint8_t)%s;" % c
    return {3: "vb_put_u16(buf, %s, &i);", 4: "vb_put_i16(buf, %s, &i);", 5: "vb_put_u32(buf, %s, &i);",
            6: "vb_put_i32(buf, %s, &i);"}[p.vtx] % c


def deser_line(p, ct, rmap=False):
    c = "c->" + p.c
    m = EMAP.get(p.c) if rmap else None
    if m:
        f = "map_bits_in" if m["bits"] else "map_in"
        return "%s = (%s)%s(buf[i++], KEEP_%s, %d);" % (c, ct, f, p.c.upper(), len(m["keep"]))
    if p.type == 1:
        if p.vtx == 7:
            return "%s = vb_get_f16(buf, %s, &i);" % (c, fmt_scale(p.scale))
        if p.vtx == 8:
            return "%s = vb_get_f32(buf, %s, &i);" % (c, fmt_scale(p.scale))
        if p.vtx == 9:
            return "%s = vb_get_f32_auto(buf, &i);" % c
    if p.vtx in (VTX_NONE, 1, 2) or p.type in (4, 5, 6):
        return "%s = (%s)buf[i++];" % (c, ct)
    return {3: "%s = vb_get_u16(buf, &i);", 4: "%s = vb_get_i16(buf, &i);", 5: "%s = vb_get_u32(buf, &i);",
            6: "%s = vb_get_i32(buf, &i);"}[p.vtx] % c


def ser_size(p):
    if p.type == 1 and p.vtx == 7:
        return 2
    if p.type == 1 and p.vtx in (8, 9):
        return 4
    if p.vtx in (VTX_NONE, 1, 2) or p.type in (4, 5, 6):
        return 1
    return {3: 2, 4: 2, 5: 4, 6: 4}[p.vtx]


def xml_default(p):
    if p.type == 1:
        return p.val_double or "0"
    return p.val_int or "0"


CT_CODE = {"float": 0, "uint8_t": 1, "int8_t": 2, "uint16_t": 3, "int16_t": 4, "uint32_t": 5, "int32_t": 6}


def main():
    ctypes = json.load(open(os.path.join(HERE, "vesc700_ctypes.json")))
    EMAP.update(json.load(open(os.path.join(CONF, "resc_enum_map.json"))))
    dflt = json.load(open(os.path.join(HERE, "vesc700_defaults.json")))
    prof = {}
    for pname, files in (("vesc700", ("vesc700/parameters_mcconf.xml", "vesc700/parameters_appconf.xml")),
                         ("resc", ("resc_mcconf.xml", "resc_appconf.xml"))):
        prof[pname] = {k: load(os.path.join(CONF, f)) for k, f in zip(("mc", "app"), files)}
    for k in ("mc", "app"):
        sig = prof["vesc700"][k][2]
        if sig != REF_SIG[k]:
            raise SystemExit("signature VESC700 %s calculee %u != reference %u : XML non officiel ?" % (k, sig, REF_SIG[k]))
    # vue RESC Tool du profil VESC700 : embarquee pour la commande 250, meme signature exigee
    view = {k: load(os.path.join(CONF, "vesc700_view", f)) for k, f in
            (("mc", "parameters_mcconf.xml"), ("app", "parameters_appconf.xml"))}
    for k in ("mc", "app"):
        if view[k][2] != REF_SIG[k] or [p.name for p in view[k][1]] != [p.name for p in prof["vesc700"][k][1]]:
            raise SystemExit("vue VESC700 %s : signature ou ordre de serialisation different de l'officiel" % k)

    h, c = [], []
    h.append("/* GENERE par tools/resc_conf/gen_resc_conf.py depuis les XML de firmware/conf - ne pas editer. */")
    h.append("#ifndef VESC_CONF_H\n#define VESC_CONF_H\n\n#include <stdint.h>\n#include <stdbool.h>\n")
    h.append("/* Profils de configuration : VESC700 (defaut, VESC Tool officiel) ou RESC (RESC Tool).")
    h.append(" * Les structures internes contiennent les champs des deux profils ; chaque profil")
    h.append(" * ne serialise que les siens. Les champs hors profil gardent leur valeur interne. */")
    c.append("/* GENERE par tools/resc_conf/gen_resc_conf.py depuis les XML de firmware/conf - ne pas editer. */")
    c.append('#include <stddef.h>\n#include "vesc_conf.h"\n#include "vesc_buffer.h"\n#include "vesc_compat_defaults.h"\n')
    c.append("/* Profil RESC : listes reduites aux choix pris en charge. Position affichee <-> valeur interne")
    c.append(" * (valeur du format 7.00) ; pour les champs de bits, bit affiche <-> bit interne. */")
    c.append("static uint8_t map_out(uint8_t v, const uint8_t *k, uint8_t n) { for (uint8_t j = 0; j < n; j++) if (k[j] == v) return j; return 0; }")
    c.append("static uint8_t map_in(uint8_t j, const uint8_t *k, uint8_t n) { return j < n ? k[j] : k[0]; }")
    c.append("static uint8_t map_bits_out(uint8_t v, const uint8_t *k, uint8_t n) { uint8_t o = 0; for (uint8_t j = 0; j < n; j++) if (v & (1u << k[j])) o |= (uint8_t)(1u << j); return o; }")
    c.append("static uint8_t map_bits_in(uint8_t o, const uint8_t *k, uint8_t n) { uint8_t v = 0; for (uint8_t j = 0; j < n; j++) if (o & (1u << j)) v |= (uint8_t)(1u << k[j]); return v; }")
    for name, m in sorted(EMAP.items()):
        c.append("static const uint8_t KEEP_%s[] = { %s };" % (name.upper(), ", ".join(str(v) for v in m["keep"])))
    c.append("")

    for k, sname, lname in (("mc", "vesc_mcconf_t", "mcconf"), ("app", "vesc_appconf_t", "appconf")):
        v_ps, r_ps = prof["vesc700"][k][1], prof["resc"][k][1]
        v_names = [p.c for p in v_ps]
        fields = list(v_ps) + [p for p in r_ps if p.c not in v_names]
        ct = {}
        for p in fields:
            ct[p.c] = ctypes[sname].get(p.c) or ctype_from_xml(p)
        rmap = {p.c: p for p in r_ps}
        vmap = {p.c: p for p in v_ps}

        h.append("\ntypedef struct {")
        for p in fields:
            h.append("    %-9s %s;" % (ct[p.c], p.c))
        h.append("} %s;\n" % sname)
        sig_v, sig_r = prof["vesc700"][k][2], prof["resc"][k][2]
        up = lname.upper()
        h.append("#define VESC700_%s_SIGNATURE %uu" % (up, sig_v))
        h.append("#define RESCP_%s_SIGNATURE   %uu" % (up, sig_r))
        sz_v = 4 + sum(ser_size(p) for p in v_ps)
        sz_r = 4 + sum(ser_size(p) for p in r_ps)
        h.append("#define VESC700_%s_SER_SIZE %d" % (up, sz_v))
        h.append("#define RESCP_%s_SER_SIZE   %d" % (up, sz_r))
        h.append("#ifdef RESC_CONF_PROFILE_RESC")
        h.append("#define VESC_%s_SIGNATURE RESCP_%s_SIGNATURE" % (up, up))
        h.append("#else")
        h.append("#define VESC_%s_SIGNATURE VESC700_%s_SIGNATURE" % (up, up))
        h.append("#endif")
        h.append("#define VESC_%s_SER_MAX %d" % (up, max(sz_v, sz_r)))
        h.append("")
        for pre in ("vesc700", "rescp", "vesc"):
            h.append("int32_t %s_%s_serialize(uint8_t *buf, const %s *c);" % (pre, lname, sname))
            h.append("bool    %s_%s_deserialize(const uint8_t *buf, int32_t len, %s *c);" % (pre, lname, sname))
        h.append("void    vesc_%s_defaults(%s *c);" % (lname, sname))

        # serialisation des deux profils
        for pre, ps, sigm in (("vesc700", v_ps, "VESC700_%s_SIGNATURE" % up), ("rescp", r_ps, "RESCP_%s_SIGNATURE" % up)):
            c.append("int32_t %s_%s_serialize(uint8_t *buf, const %s *c)\n{" % (pre, lname, sname))
            c.append("    int32_t i = 0;")
            c.append("    vb_put_u32(buf, %s, &i);" % sigm)
            for p in ps:
                c.append("    " + ser_line(p, ct[p.c], pre == "rescp"))
            c.append("    return i;\n}\n")
            c.append("bool %s_%s_deserialize(const uint8_t *buf, int32_t len, %s *c)\n{" % (pre, lname, sname))
            c.append("    int32_t i = 0;")
            c.append("    if (len < %d) return false;" % (4 + sum(ser_size(p) for p in ps)))
            c.append("    if (vb_get_u32(buf, &i) != %s) return false;" % sigm)
            for p in ps:
                c.append("    " + deser_line(p, ct[p.c], pre == "rescp"))
            c.append("    return true;\n}\n")
        c.append("#ifdef RESC_CONF_PROFILE_RESC")
        c.append("int32_t vesc_%s_serialize(uint8_t *buf, const %s *c) { return rescp_%s_serialize(buf, c); }" % (lname, sname, lname))
        c.append("bool vesc_%s_deserialize(const uint8_t *buf, int32_t len, %s *c) { return rescp_%s_deserialize(buf, len, c); }" % (lname, sname, lname))
        c.append("#else")
        c.append("int32_t vesc_%s_serialize(uint8_t *buf, const %s *c) { return vesc700_%s_serialize(buf, c); }" % (lname, sname, lname))
        c.append("bool vesc_%s_deserialize(const uint8_t *buf, int32_t len, %s *c) { return vesc700_%s_deserialize(buf, len, c); }" % (lname, sname, lname))
        c.append("#endif\n")

        # defauts
        c.append("void vesc_%s_defaults(%s *c)\n{" % (lname, sname))
        for p in fields:
            if p.c in dflt[sname]:
                c.append("    c->%s = %s;" % (p.c, dflt[sname][p.c]))
            else:
                macro = p.cdefine or ("RESC_DEF_" + p.c.upper())
                c.append("#ifndef %s\n#define %s %s\n#endif" % (macro, macro, xml_default(p)))
                c.append("    c->%s = (%s)(%s);" % (p.c, ct[p.c], macro))
        c.append("#ifdef RESC_CONF_PROFILE_RESC")
        c.append("    /* defauts propres au profil RESC (valeurs du XML RESC) */")
        for p in r_ps:
            vp = vmap.get(p.c)
            if vp is not None and p.c not in EMAP and xml_default(p) != xml_default(vp):
                c.append("    c->%s = (%s)(%s);" % (p.c, ct[p.c], xml_default(p)))
        c.append("#endif")
        c.append("}\n")

        # table des champs (stockage par nom, terminal)
        c.append("const conf_field_t vesc_%s_fields[] = {" % lname)
        for p in fields:
            c.append('    { 0x%08Xu, (uint16_t)offsetof(%s, %s), %d, "%s" },' % (crc32_name(p.c), sname, p.c, CT_CODE[ct[p.c]], p.c))
        c.append("};")
        c.append("const uint16_t vesc_%s_nfields = %d;\n" % (lname, len(fields)))
        h.append("extern const conf_field_t vesc_%s_fields[];" % lname)
        h.append("extern const uint16_t vesc_%s_nfields;" % lname)

    # type de la table, avant les structures
    i0 = h.index("/* Profils de configuration : VESC700 (defaut, VESC Tool officiel) ou RESC (RESC Tool).")
    h[i0:i0] = ["/* Champ de configuration : crc32 du nom, decalage, type (0 float, 1 u8, 2 i8, 3 u16, 4 i16, 5 u32, 6 i32) */",
                "typedef struct {\n    uint32_t hash;\n    uint16_t offset;\n    uint8_t  ctype;\n    const char *name;\n} conf_field_t;\n"]
    h.append("\n#endif")

    # description de la commande 250 : partie fixe compressee + onglets conditionnels (desc_vis.py)
    x = ['/* GENERE par tools/resc_conf/gen_resc_conf.py - description de la commande 250. */',
         '#include "vesc_conf.h"', '#include "conf_desc.h"', '']
    sizes = {}
    for pname, macro in (("resc", "#ifdef RESC_CONF_PROFILE_RESC"), ("vesc700", "#else")):
        x.append(macro)
        for k in ("mc", "app"):
            raw = view[k][0] if pname == "vesc700" else prof[pname][k][0]
            vis = (desc_vis.VIS_MC if k == "mc" else desc_vis.VIS_APP) if pname == "resc" else {}
            sizes["%s_%s" % (pname, k)] = (len(raw), len(zlib.compress(raw, 9)))
            x += desc_vis.c_lines(raw, ET.fromstring(raw), k, vis)
    x.append("#endif")

    open(os.path.join(OUT, "vesc_conf.h"), "w", encoding="utf-8", newline="\n").write("\n".join(h) + "\n")
    open(os.path.join(OUT, "vesc_conf.c"), "w", encoding="utf-8", newline="\n").write("\n".join(c) + "\n")
    open(os.path.join(OUT, "conf_xml.c"), "w", encoding="utf-8", newline="\n").write("\n".join(x) + "\n")
    for pname in ("vesc700", "resc"):
        for k in ("mc", "app"):
            print("%-8s %-4s signature %10u  %3d parametres  XML %6d -> %6d octets" % (
                pname, k, prof[pname][k][2], len(prof[pname][k][1]), *sizes["%s_%s" % (pname, k)]))


if __name__ == "__main__":
    sys.exit(main())
