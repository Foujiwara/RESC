#!/usr/bin/env python3
"""
Genere la couche de compatibilite de configuration VESC Tool pour RESC.

VESC Tool lit/ecrit la configuration moteur (MCCONF) et application (APPCONF)
sous forme d'un flux binaire dont l'ordre des champs, les encodages et la
signature doivent etre EXACTEMENT ceux du firmware de reference (7.00).
Ce script relit le format depuis bldc/confgenerator.{c,h}, les valeurs par
defaut depuis motor/mcconf_default.h et applications/appconf_default.h, et les
enumerations depuis datatypes.h, puis ecrit :

  firmware/src/vesc/vesc_conf.h      structures (champs aplatis) + prototypes
  firmware/src/vesc/vesc_conf.c      serialisation / deserialisation / defauts
  firmware/src/vesc/vesc_defaults.h  macros de defaut et enumerations

Le format est une donnee d'interface : aucune logique de controle VESC n'est
reprise. Le coeur RESC n'utilise que les champs qu'il traduit (vesc_map.c).

Usage : python -I tools/vesc_compat/gen_conf.py <bldc_root> <firmware_src_dir>
"""
import os
import re
import sys


def read(p):
    with open(p, encoding="utf-8", errors="replace") as fh:
        return fh.read()


def strip_comments(t):
    t = re.sub(r"/\*.*?\*/", "", t, flags=re.S)
    return re.sub(r"//[^\n]*", "", t)


def func_body(src, name):
    m = re.search(r"\b%s\s*\([^)]*\)\s*\{" % re.escape(name), src)
    if not m:
        raise SystemExit(f"fonction {name} introuvable")
    i, depth = m.end(), 1
    while depth:
        if src[i] == "{": depth += 1
        elif src[i] == "}": depth -= 1
        i += 1
    return src[m.end():i - 1]


def flat(field):
    return re.sub(r"\[(\d+)\]", r"_\1", field).replace(".", "_")


SER_PATTERNS = [
    (re.compile(r"^buffer_append_uint32\(buffer,\s*(MC|APP)CONF_SIGNATURE,\s*&ind\);$"), "sig"),
    (re.compile(r"^buffer\[ind\+\+\]\s*=\s*\(uint8_t\)conf->([\w.\[\]]+);$"), "u8f"),
    (re.compile(r"^buffer\[ind\+\+\]\s*=\s*conf->([\w.\[\]]+);$"), "u8"),
    (re.compile(r"^buffer_append_float32_auto\(buffer,\s*conf->([\w.\[\]]+),\s*&ind\);$"), "f32a"),
    (re.compile(r"^buffer_append_float16\(buffer,\s*conf->([\w.\[\]]+),\s*([\w.e+-]+),\s*&ind\);$"), "f16"),
    (re.compile(r"^buffer_append_float32\(buffer,\s*conf->([\w.\[\]]+),\s*([\w.e+-]+),\s*&ind\);$"), "f32s"),
    (re.compile(r"^buffer_append_uint16\(buffer,\s*conf->([\w.\[\]]+),\s*&ind\);$"), "u16"),
    (re.compile(r"^buffer_append_int16\(buffer,\s*conf->([\w.\[\]]+),\s*&ind\);$"), "i16"),
    (re.compile(r"^buffer_append_uint32\(buffer,\s*conf->([\w.\[\]]+),\s*&ind\);$"), "u32"),
    (re.compile(r"^buffer_append_int32\(buffer,\s*conf->([\w.\[\]]+),\s*&ind\);$"), "i32"),
]


def parse_ser(body):
    fields = []
    for raw in body.split(";"):
        line = " ".join(raw.split()) + ";"
        if line in (";", "int32_t ind = 0;", "return ind;"):
            continue
        for pat, kind in SER_PATTERNS:
            m = pat.match(line)
            if m:
                if kind == "sig":
                    break
                fields.append((kind, m.group(1), m.group(2) if kind in ("f16", "f32s") else None))
                break
        else:
            raise SystemExit(f"ligne de serialisation non reconnue : {line}")
    return fields


def signed_u8(deser_body):
    """Champs lus avec (int8_t)buffer[ind++] dans le deserialiseur."""
    return set(re.findall(r"conf->([\w.\[\]]+)\s*=\s*\(int8_t\)\s*buffer\[ind\+\+\]", deser_body))


def parse_defaults(body):
    out = {}
    for m in re.finditer(r"conf->([\w.\[\]]+)\s*=\s*([^;]+);", body):
        out[m.group(1)] = " ".join(m.group(2).split())
    return out


CTYPE = {"u8": "uint8_t", "u8f": "float", "f32a": "float", "f16": "float", "f32s": "float",
         "u16": "uint16_t", "i16": "int16_t", "u32": "uint32_t", "i32": "int32_t"}


def gen_kind(name, fields, sint8, defaults, sig_macro):
    st = f"vesc_{name}_t"
    h, c = [], []
    h.append(f"typedef struct {{")
    for kind, f, _ in fields:
        t = "int8_t" if (kind == "u8" and f in sint8) else CTYPE[kind]
        h.append(f"    {t:<9} {flat(f)};")
    h.append(f"}} {st};")
    h.append("")
    h.append(f"int32_t vesc_{name}_serialize(uint8_t *buf, const {st} *c);")
    h.append(f"bool    vesc_{name}_deserialize(const uint8_t *buf, int32_t len, {st} *c);")
    h.append(f"void    vesc_{name}_defaults({st} *c);")
    h.append(f"#define VESC_{name.upper()}_SER_MAX  {4 + sum(4 if k in ('f32a','f32s','u32','i32') else 2 if k in ('f16','u16','i16') else 1 for k,_,_ in fields)}")
    h.append("")

    c.append(f"int32_t vesc_{name}_serialize(uint8_t *buf, const {st} *c)\n{{\n    int32_t i = 0;")
    c.append(f"    vb_put_u32(buf, {sig_macro}, &i);")
    for kind, f, sc in fields:
        n = flat(f)
        if kind == "u8": c.append(f"    buf[i++] = (uint8_t)c->{n};")
        elif kind == "u8f": c.append(f"    buf[i++] = (uint8_t)c->{n};")
        elif kind == "f32a": c.append(f"    vb_put_f32_auto(buf, c->{n}, &i);")
        elif kind == "f16": c.append(f"    vb_put_f16(buf, c->{n}, {sc}, &i);")
        elif kind == "f32s": c.append(f"    vb_put_f32(buf, c->{n}, {sc}, &i);")
        elif kind == "u16": c.append(f"    vb_put_u16(buf, c->{n}, &i);")
        elif kind == "i16": c.append(f"    vb_put_i16(buf, c->{n}, &i);")
        elif kind == "u32": c.append(f"    vb_put_u32(buf, c->{n}, &i);")
        elif kind == "i32": c.append(f"    vb_put_i32(buf, c->{n}, &i);")
    c.append("    return i;\n}\n")

    c.append(f"bool vesc_{name}_deserialize(const uint8_t *buf, int32_t len, {st} *c)\n{{\n    int32_t i = 0;")
    c.append(f"    if (len < VESC_{name.upper()}_SER_MAX) return false;")
    c.append(f"    if (vb_get_u32(buf, &i) != {sig_macro}) return false;")
    for kind, f, sc in fields:
        n = flat(f)
        if kind == "u8": c.append(f"    c->{n} = ({'int8_t' if f in sint8 else 'uint8_t'})buf[i++];")
        elif kind == "u8f": c.append(f"    c->{n} = (float)buf[i++];")
        elif kind == "f32a": c.append(f"    c->{n} = vb_get_f32_auto(buf, &i);")
        elif kind == "f16": c.append(f"    c->{n} = vb_get_f16(buf, {sc}, &i);")
        elif kind == "f32s": c.append(f"    c->{n} = vb_get_f32(buf, {sc}, &i);")
        elif kind == "u16": c.append(f"    c->{n} = vb_get_u16(buf, &i);")
        elif kind == "i16": c.append(f"    c->{n} = vb_get_i16(buf, &i);")
        elif kind == "u32": c.append(f"    c->{n} = vb_get_u32(buf, &i);")
        elif kind == "i32": c.append(f"    c->{n} = vb_get_i32(buf, &i);")
    c.append("    return true;\n}\n")

    c.append(f"void vesc_{name}_defaults({st} *c)\n{{")
    for kind, f, _ in fields:
        if f in defaults:
            c.append(f"    c->{flat(f)} = {defaults[f]};")
        else:
            c.append(f"    c->{flat(f)} = 0; /* pas de defaut dans la reference */")
    c.append("}\n")
    return h, c


def extract_default_macros(text):
    """Blocs '#ifndef X / #define X val / #endif' -> lignes recopiees."""
    out = []
    text = strip_comments(text)
    for m in re.finditer(r"#ifndef\s+(\w+)\s*\n\s*#define\s+\1\b([^\n]*)\n\s*#endif", text):
        out.append((m.group(1), m.group(2).strip()))
    return out


def extract_enums(text):
    text = strip_comments(text)
    return re.findall(r"typedef\s+enum\s*\{([^}]*)\}\s*(\w+)\s*;", text, flags=re.S)


def main():
    root, outdir = sys.argv[1], sys.argv[2]
    src = strip_comments(read(os.path.join(root, "confgenerator.c")))
    hdr = read(os.path.join(root, "confgenerator.h"))
    sigs = dict(re.findall(r"#define\s+(MCCONF_SIGNATURE|APPCONF_SIGNATURE)\s+(\d+)", hdr))

    mc = parse_ser(func_body(src, "confgenerator_serialize_mcconf"))
    app = parse_ser(func_body(src, "confgenerator_serialize_appconf"))
    mc_s8 = signed_u8(func_body(src, "confgenerator_deserialize_mcconf"))
    app_s8 = signed_u8(func_body(src, "confgenerator_deserialize_appconf"))
    mc_def = parse_defaults(func_body(src, "confgenerator_set_defaults_mcconf"))
    app_def = parse_defaults(func_body(src, "confgenerator_set_defaults_appconf"))

    vdir = os.path.join(outdir, "vesc")
    os.makedirs(vdir, exist_ok=True)

    # --- defauts + enums
    d = ["/* GENERE par tools/vesc_compat/gen_conf.py depuis bldc release_7_00 - ne pas editer. */",
         "/* Valeurs par defaut de la configuration VESC (interface VESC Tool) et enumerations. */",
         "#ifndef VESC_DEFAULTS_H", "#define VESC_DEFAULTS_H", ""]
    for body, name in extract_enums(read(os.path.join(root, "datatypes.h"))):
        items = [x.strip() for x in body.split(",") if x.strip()]
        d.append(f"typedef enum {{ {', '.join(items)} }} {name};")
    d.append("")
    for fn in ("motor/mcconf_default.h", "applications/appconf_default.h"):
        d.append(f"/* --- {fn} --- */")
        for k, v in extract_default_macros(read(os.path.join(root, fn))):
            d.append(f"#ifndef {k}\n#define {k} {v}\n#endif")
    d.append("\n#endif")
    with open(os.path.join(vdir, "vesc_defaults.h"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(d) + "\n")

    hm, cm = gen_kind("mcconf", mc, mc_s8, mc_def, "VESC_MCCONF_SIGNATURE")
    ha, ca = gen_kind("appconf", app, app_s8, app_def, "VESC_APPCONF_SIGNATURE")
    h = ["/* GENERE par tools/vesc_compat/gen_conf.py depuis bldc release_7_00 - ne pas editer. */",
         "#ifndef VESC_CONF_H", "#define VESC_CONF_H", "#include <stdint.h>", "#include <stdbool.h>", "",
         f"#define VESC_MCCONF_SIGNATURE   {sigs['MCCONF_SIGNATURE']}u",
         f"#define VESC_APPCONF_SIGNATURE  {sigs['APPCONF_SIGNATURE']}u", ""] + hm + ha + ["#endif"]
    with open(os.path.join(vdir, "vesc_conf.h"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(h) + "\n")
    c = ["/* GENERE par tools/vesc_compat/gen_conf.py depuis bldc release_7_00 - ne pas editer. */",
         '#include "vesc_conf.h"', '#include "vesc_buffer.h"', '#include "vesc_compat_defaults.h"', ""] + cm + ca
    with open(os.path.join(vdir, "vesc_conf.c"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(c) + "\n")
    print(f"mcconf : {len(mc)} champs, appconf : {len(app)} champs ; signatures {sigs}")


if __name__ == "__main__":
    main()
