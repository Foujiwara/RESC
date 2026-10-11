#!/usr/bin/env python3
"""
build.py — Compilation du firmware RESC (sans make, fonctionne sous Windows).

  python build.py                         carte par defaut (Ubox SINGLE 85_200)
  python build.py --board ubox/board_ubox_single_80
  python build.py --sim                   compile et lance la simulation PC
  python build.py --clean

Produit build/<carte>/resc.elf, .bin, .hex et .map.
Chaines : arm-none-eabi-gcc (Arm GNU Toolchain) ; gcc hote pour --sim.
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

ARM_DIRS = [r"C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin"]
HOST_DIRS = glob.glob(os.path.expandvars(r"%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs*\mingw64\bin"))


def find_tool(name, dirs):
    p = shutil.which(name)
    if p:
        return p
    for d in dirs:
        c = os.path.join(d, name + (".exe" if os.name == "nt" else ""))
        if os.path.exists(c):
            return c
    sys.exit(f"outil introuvable : {name}")


SRC_DIRS = ["src", "src/core", "src/bsp", "src/comm", "src/app", "src/vesc"]
INC = ["src", "src/core", "src/bsp", "src/comm", "src/app", "src/vesc", "cmsis", "../hw"]

CFLAGS = [
    "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=hard", "-mfpu=fpv4-sp-d16",
    "-std=gnu11", "-O2", "-g3",
    "-ffunction-sections", "-fdata-sections", "-fno-common",
    "-fno-math-errno", "-fsingle-precision-constant",
    "-Wall", "-Wextra", "-Wdouble-promotion", "-Wno-unused-parameter",
    "-DSTM32F405xx",
]
LDFLAGS = [
    "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=hard", "-mfpu=fpv4-sp-d16",
    "--specs=nano.specs", "--specs=nosys.specs", "-u", "_printf_float",
    "-Wl,--gc-sections", "-Wl,--print-memory-usage", "-Wl,--no-warn-rwx-segments",
]


def run(cmd, fatal=True):
    r = subprocess.run(cmd, cwd=HERE, capture_output=True, text=True)
    out = (r.stdout + r.stderr).strip()
    if out:
        print(out)
    if r.returncode != 0 and fatal:
        sys.exit(f"ECHEC : {' '.join(cmd[:2])} ...")
    return r.returncode == 0


def board_description(board, conf, out):
    """Description de configuration (commande 250) avec les vraies valeurs par defaut et plages
    de la carte : defauts releves en executant le code de configuration du firmware sur PC."""
    gcc = find_tool("gcc", HOST_DIRS)
    defs = [f'-DRESC_BOARD_HEADER="boards/{board}.h"', "-DRESC_HOST"] + (["-DRESC_CONF_PROFILE_RESC"] if conf == "resc" else [])
    exe = os.path.join(out, "dump_defaults.exe")
    srcs = ["tests/dump_defaults.c", "src/app/resc_conf.c", "src/vesc/vesc_conf.c", "src/core/foc.c",
            "src/core/observer.c", "src/core/angle_sensors.c", "src/core/ctrl.c"]
    run([gcc, "-std=gnu11", "-O1"] + defs + ["-I../hw", "-Isrc", "-Isrc/core", "-Isrc/app", "-Isrc/vesc", "-Isrc/comm",
                                             "-Isrc/bsp"] + srcs + ["-lm", "-o", exe])
    js = subprocess.run([exe], cwd=HERE, capture_output=True, text=True, check=True).stdout
    jpath = os.path.join(out, "defaults.json")
    open(jpath, "w").write(js)
    c = os.path.join(out, "conf_xml_board.c")
    print("Description de configuration de la carte (commande 250) :")
    run([sys.executable, "-I", os.path.join(ROOT, "tools", "resc_conf", "board_xml.py"), jpath, conf, c,
         os.path.join(out, "conf_xml")])
    return c


def build(board, conf):
    gcc = find_tool("arm-none-eabi-gcc", ARM_DIRS)
    objcopy = find_tool("arm-none-eabi-objcopy", ARM_DIRS)
    size = find_tool("arm-none-eabi-size", ARM_DIRS)
    out = os.path.join(HERE, "build", board.replace("/", "_") + ("_resc" if conf == "resc" else ""))
    os.makedirs(out, exist_ok=True)
    defs = [f'-DRESC_BOARD_HEADER="boards/{board}.h"'] + (["-DRESC_CONF_PROFILE_RESC"] if conf == "resc" else [])
    incs = ["-I" + i for i in INC]
    objs = []
    failed = []
    board_xml = board_description(board, conf, out)
    for d in SRC_DIRS:
        for c in sorted(glob.glob(os.path.join(HERE, d, "*.c"))) + ([board_xml] if d == "src/vesc" else []):
            if os.path.basename(c) == "conf_xml.c":
                continue                    # remplace par la description propre a la carte
            o = os.path.join(out, os.path.relpath(c, HERE).replace(os.sep, "_")[:-2] + ".o")
            if not run([gcc] + CFLAGS + defs + incs + ["-c", c, "-o", o], fatal=False):
                failed.append(os.path.relpath(c, HERE))
            objs.append(o)
    if failed:
        sys.exit("ECHEC de compilation : " + ", ".join(failed))
    elf = os.path.join(out, "resc.elf")
    run([gcc] + LDFLAGS + ["-T", "linker/stm32f405_resc.ld", "-Wl,-Map=" + os.path.join(out, "resc.map")] + objs +
        ["-lm", "-o", elf])
    # trou des secteurs de configuration (0x08004000-0x0800BFFF) rempli a 0xFF = flash effacee
    run([objcopy, "-O", "binary", "--gap-fill", "0xFF", elf, os.path.join(out, "resc.bin")])
    run([objcopy, "-O", "ihex", elf, os.path.join(out, "resc.hex")])
    # flashage sans effacer la configuration (secteurs 1-2) : vecteurs (secteur 0) et application
    full = open(os.path.join(out, "resc.bin"), "rb").read()
    open(os.path.join(out, "resc_vec.bin"), "wb").write(full[:0x4000])      # 0x08000000
    open(os.path.join(out, "resc_app.bin"), "wb").write(full[0xC000:])      # 0x0800C000
    run([size, elf])
    check_ramfunc(os.path.join(out, "resc.map"))
    print(f"\nOK : {os.path.relpath(elf, ROOT)}")


def check_ramfunc(mapfile):
    """Le chemin FOC doit etre en RAM : verifie que les fonctions critiques y sont."""
    txt = open(mapfile, encoding="utf-8", errors="replace").read()
    need = ["ADC_IRQHandler", "foc_isr", "obs_update", "hall_update", "enc_update"]
    start = txt.find(".ramfunc")
    seg = txt[start:start + 20000] if start >= 0 else ""
    missing = [f for f in need if f not in seg]
    if missing:
        print("ATTENTION : hors RAM :", ", ".join(missing))
    else:
        print("Chemin FOC en RAM : OK (" + ", ".join(need) + ")")


def sim():
    gcc = find_tool("gcc", HOST_DIRS)
    out = os.path.join(HERE, "build", "sim")
    os.makedirs(out, exist_ok=True)
    exe = os.path.join(out, "sim.exe")
    srcs = ["sim/sim_main.c", "sim/sim_motor.c", "src/core/foc.c", "src/core/observer.c", "src/core/angle_sensors.c", "src/core/ctrl.c"]
    run([gcc, "-std=gnu11", "-O2", "-Wall", "-Wextra", "-DRESC_HOST", "-Isrc", "-Isrc/core", "-Isim"] + srcs + ["-lm", "-o", exe])
    r = subprocess.run([exe], cwd=HERE)
    sys.exit(r.returncode)


def check_cmd250(conf):
    """Decompresse les XML reconstitues par le test de la commande 250 et les compare aux sources."""
    import struct
    import zlib
    files = {"mc": "resc_mcconf.xml", "app": "resc_appconf.xml"} if conf == "resc" else \
            {"mc": "vesc700_view/parameters_mcconf.xml", "app": "vesc700_view/parameters_appconf.xml"}
    ok = True
    for k, f in files.items():
        blob = open(os.path.join(HERE, "build", f"cmd250_{k}.bin"), "rb").read()
        src = open(os.path.join(HERE, "conf", f), "rb").read()
        n = struct.unpack(">I", blob[:4])[0]
        xml = zlib.decompress(blob[4:])
        good = n == len(src) and xml == src
        print(f"  [{' OK ' if good else 'ECHEC'}] commande 250, {k} : XML decompresse identique a conf/{f}")
        ok &= good
    return ok


def tests(board):
    """Tests PC : configuration (serialisation, bornage), configuration generee (deux profils,
    commande 250) et applications (entrees simulees)."""
    gcc = find_tool("gcc", HOST_DIRS)
    out = os.path.join(HERE, "build")
    os.makedirs(out, exist_ok=True)
    flags = ["-std=gnu11", "-O1", "-Wall", "-Wextra", "-DRESC_HOST", f'-DRESC_BOARD_HEADER="boards/{board}.h"',
             "-I../hw", "-Isrc", "-Isrc/core", "-Isrc/app", "-Isrc/vesc", "-Isrc/comm", "-Isrc/bsp"]
    suites = {
        "test_conf": ["tests/test_conf.c", "src/app/resc_conf.c", "src/vesc/vesc_conf.c", "src/comm/packet.c",
                      "src/core/foc.c", "src/core/observer.c", "src/core/angle_sensors.c", "src/core/ctrl.c"],
        "test_apps": ["tests/test_apps.c", "src/app/apps.c", "src/vesc/vesc_conf.c"],
    }
    rc = 0
    for name, srcs in suites.items():
        exe = os.path.join(out, name + ".exe")
        run([gcc] + flags + srcs + ["-lm", "-o", exe])
        print(f"== {name}")
        rc |= subprocess.run([exe], cwd=HERE).returncode
    gen = ["tests/test_conf_gen.c", "src/vesc/vesc_conf.c", "src/vesc/conf_xml.c", "src/comm/conf_desc.c",
           "tests/ref/vesc_conf_ref.c"]
    for conf in ("vesc700", "resc"):
        exe = os.path.join(out, f"test_conf_gen_{conf}.exe")
        run([gcc] + flags + (["-DRESC_CONF_PROFILE_RESC"] if conf == "resc" else []) + gen + ["-lm", "-o", exe])
        print(f"== test_conf_gen (profil {conf})")
        rc |= subprocess.run([exe], cwd=HERE).returncode
        rc |= 0 if check_cmd250(conf) else 1
        exe = os.path.join(out, f"test_migrate_{conf}.exe")
        mig = ["tests/test_migrate.c", "src/app/resc_conf.c", "src/vesc/vesc_conf.c", "src/core/foc.c",
               "src/core/observer.c", "src/core/angle_sensors.c", "src/core/ctrl.c"]
        run([gcc] + flags + (["-DRESC_CONF_PROFILE_RESC"] if conf == "resc" else []) + mig + ["-lm", "-o", exe])
        print(f"== test_migrate (profil {conf})")
        rc |= subprocess.run([exe], cwd=HERE).returncode
    print("== audit des parametres (inutilises invisibles, utilises visibles)")
    rc |= subprocess.run([sys.executable, "-I", os.path.join(ROOT, "tools", "resc_conf", "audit_params.py"), "--strict"],
                         cwd=ROOT).returncode
    sys.exit(rc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", default="ubox/board_ubox_single_85_200")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--tests", action="store_true")
    ap.add_argument("--conf", choices=["vesc700", "resc"], default="vesc700",
                    help="profil de configuration : vesc700 (VESC Tool officiel) ou resc (RESC Tool)")
    a = ap.parse_args()
    if a.clean:
        shutil.rmtree(os.path.join(HERE, "build"), ignore_errors=True)
        return
    if a.sim:
        sim()
    if a.tests:
        tests(a.board)
    build(a.board, a.conf)


if __name__ == "__main__":
    main()
