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


def build(board):
    gcc = find_tool("arm-none-eabi-gcc", ARM_DIRS)
    objcopy = find_tool("arm-none-eabi-objcopy", ARM_DIRS)
    size = find_tool("arm-none-eabi-size", ARM_DIRS)
    out = os.path.join(HERE, "build", board.replace("/", "_"))
    os.makedirs(out, exist_ok=True)
    defs = [f'-DRESC_BOARD_HEADER="boards/{board}.h"']
    incs = ["-I" + i for i in INC]
    objs = []
    failed = []
    for d in SRC_DIRS:
        for c in sorted(glob.glob(os.path.join(HERE, d, "*.c"))):
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


def tests(board):
    """Tests PC : configuration (serialisation, bornage) et applications (entrees simulees)."""
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
    sys.exit(rc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", default="ubox/board_ubox_single_85_200")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--tests", action="store_true")
    a = ap.parse_args()
    if a.clean:
        shutil.rmtree(os.path.join(HERE, "build"), ignore_errors=True)
        return
    if a.sim:
        sim()
    if a.tests:
        tests(a.board)
    build(a.board)


if __name__ == "__main__":
    main()
