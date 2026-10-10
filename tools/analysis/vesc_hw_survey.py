#!/usr/bin/env python3
"""
Recensement du brochage des cartes VESC (dossier hwconf de bldc release_7_00).

Lit uniquement les fichiers comme des donnees (aucune execution). Un mini
preprocesseur C resout #define / #ifdef / #if defined() / #elif / #else et les
#include "locaux", puis extrait par carte : gate driver, shunts, broches
d'enable/fault, affectation ADC (depuis hw_setup_adc_channels), UART, halls,
CAN, BRK, TIM1/TIM8, limites.

Usage : python -I vesc_hw_survey.py <chemin/vers/bldc/hwconf> <sortie.csv>
"""
import csv
import os
import re
import sys
from collections import Counter

ADC_PIN = {**{i: f"PA{i}" for i in range(8)}, 8: "PB0", 9: "PB1",
           **{i: f"PC{i-10}" for i in range(10, 16)}}

def index_files(root):
    idx = {}
    for d, _, files in os.walk(root):
        for f in files:
            idx.setdefault(f, os.path.join(d, f))
    return idx

def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)

def join_continuations(text):
    return re.sub(r"\\\s*\n", " ", text)

class PP:
    def __init__(self, idx):
        self.idx = idx
        self.macros = {}
        self.out = []          # lignes de code actives (pour les .c)

    def eval_cond(self, expr):
        e = re.sub(r"defined\s*\(\s*(\w+)\s*\)", lambda m: "1" if m.group(1) in self.macros else "0", expr)
        e = re.sub(r"defined\s+(\w+)", lambda m: "1" if m.group(1) in self.macros else "0", e)
        def sub(m):
            v = self.macros.get(m.group(0), "0").strip() or "1"
            return v if re.fullmatch(r"[-+0-9.eExX()a-fA-F ]+", v) else "0"
        e = re.sub(r"\b[A-Za-z_]\w*\b", sub, e)
        e = e.replace("&&", " and ").replace("||", " or ")
        e = re.sub(r"!(?!=)", " not ", e)
        try:
            return bool(eval(e, {"__builtins__": {}}, {}))
        except Exception:
            return False

    def run(self, path, depth=0, collect=False):
        if depth > 12 or not os.path.exists(path):
            return
        with open(path, encoding="utf-8", errors="replace") as fh:
            text = join_continuations(strip_comments(fh.read()))
        stack = []  # (active, any_taken)
        for raw in text.split("\n"):
            line = raw.strip()
            active = all(s[0] for s in stack)
            m = re.match(r"#\s*(\w+)\s*(.*)", line)
            if m:
                d, rest = m.group(1), m.group(2).strip()
                if d in ("ifdef", "ifndef", "if"):
                    if not active:
                        stack.append((False, True)); continue
                    if d == "ifdef": c = rest.split()[0] in self.macros if rest else False
                    elif d == "ifndef": c = rest.split()[0] not in self.macros if rest else True
                    else: c = self.eval_cond(rest)
                    stack.append((c, c)); continue
                if d == "elif" and stack:
                    act, taken = stack.pop()
                    parent = all(s[0] for s in stack)
                    c = parent and not taken and self.eval_cond(rest)
                    stack.append((c, taken or c)); continue
                if d == "else" and stack:
                    act, taken = stack.pop()
                    parent = all(s[0] for s in stack)
                    stack.append((parent and not taken, True)); continue
                if d == "endif" and stack:
                    stack.pop(); continue
                if not active:
                    continue
                if d == "define":
                    mm = re.match(r"(\w+)(\([^)]*\))?\s*(.*)", rest)
                    if mm:
                        self.macros[mm.group(1)] = mm.group(3).strip()
                elif d == "undef":
                    self.macros.pop(rest.split()[0], None)
                elif d == "include":
                    inc = re.match(r'"([^"]+)"', rest)
                    if inc:
                        p = self.idx.get(os.path.basename(inc.group(1)))
                        if p and os.path.basename(p) not in ("hw.h",):
                            self.run(p, depth + 1, collect=False)
                continue
            if active and collect:
                self.out.append(line)

def expand(pp, tok, n=6):
    for _ in range(n):
        if tok in pp.macros:
            tok = pp.macros[tok].strip()
        else:
            break
    return tok

def pin_of(pp, port_macro, pin_macro):
    port = expand(pp, port_macro); pin = expand(pp, pin_macro)
    m = re.match(r"GPIO([A-E])", port or "")
    return f"P{m.group(1)}{pin}" if m and pin.isdigit() else ""

def macro_pin(pp, macro):
    """Broche d'une macro type palSetPad(GPIOB, 5)."""
    v = pp.macros.get(macro, "")
    for _ in range(4):
        m = re.search(r"pal\w+Pad\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)", v)
        if m:
            return pin_of(pp, m.group(1), m.group(2))
        if v in pp.macros: v = pp.macros[v]
        else: break
    return ""

def survey_board(header, idx, hwconf_root):
    pp = PP(idx)
    pp.run(header)
    m = pp.macros
    if "HW_NAME" not in m:
        return None
    # source .c associe : meme radical, ou radical _core
    stem = os.path.splitext(os.path.basename(header))[0]
    cands = [stem + ".c"]
    core = re.sub(r"(_no_limits)$", "", stem)
    cands += [core + ".c", core + "_core.c"]
    for inc in re.findall(r'#\s*include\s+"(hw_[^"]+)\.h"', open(header, errors="replace").read()):
        cands += [os.path.basename(inc) + ".c"]
    src = next((idx[c] for c in cands if c in idx), None)
    code = []
    if src:
        ppc = PP(idx); ppc.macros = dict(m); ppc.run(src, collect=True); code = ppc.out
    code_txt = "\n".join(code)

    # ADC regulier : ADC_RegularChannelConfig(ADCx, ADC_Channel_N, rank, ...)
    reg = {}
    for a, ch, rk in re.findall(r"ADC_RegularChannelConfig\s*\(\s*ADC(\d)\s*,\s*ADC_Channel_(\w+)\s*,\s*(\d+)", code_txt):
        reg[(int(a), int(rk))] = ch
    adc_detail = {}
    def adc_ind(name):
        v = expand(pp, name)
        try: i = int(v)
        except ValueError: return ""
        ch = reg.get((i % 3 + 1, i // 3 + 1))
        if ch is None: return f"idx{i}"
        if ch.isdigit():
            adc_detail[name] = (int(ch), i % 3 + 1)
            return ADC_PIN.get(int(ch), ch)
        return ch

    tim1 = sorted(set(f"P{p}{n}" for p, n in re.findall(r"palSetPadMode\s*\(\s*GPIO([A-E])\s*,\s*(\d+)\s*,\s*PAL_MODE_ALTERNATE\s*\(\s*GPIO_AF_TIM1\s*\)", code_txt)))
    tim8 = sorted(set(f"P{p}{n}" for p, n in re.findall(r"palSetPadMode\s*\(\s*GPIO([A-E])\s*,\s*(\d+)\s*,\s*PAL_MODE_ALTERNATE\s*\(\s*GPIO_AF_TIM8\s*\)", code_txt)))
    drv = [k[7:] for k in m if re.fullmatch(r"HW_HAS_DRV\w+", k)]
    row = {
        "file": os.path.relpath(header, hwconf_root).replace("\\", "/"),
        "hw_name": m["HW_NAME"].strip('"'),
        "src": os.path.basename(src) if src else "",
        "gate_driver": "+".join(sorted(drv)) or "discret/autre",
        "shunts": "3" if "HW_HAS_3_SHUNTS" in m else "2",
        "phase_shunts": int("HW_HAS_PHASE_SHUNTS" in m),
        "dual_motor": int("HW_HAS_DUAL_MOTORS" in m or "HW_HAS_DUAL_PARALLEL" in m),
        "inverted_shunt": int("INVERTED_SHUNT_POLARITY" in m),
        "use_brk": int("HW_USE_BRK" in m),
        "brk_pin": pin_of(pp, "BRK_GPIO", "BRK_PIN") if "HW_USE_BRK" in m else "",
        "tim1_pins": " ".join(tim1),
        "tim8_pins": " ".join(tim8),
        "en_gate": macro_pin(pp, "ENABLE_GATE"),
        "drv_fault": macro_pin(pp, "IS_DRV_FAULT"),
        "dccal": int(bool(m.get("DCCAL_ON", "").strip())),
        "curr1": adc_ind("ADC_IND_CURR1"), "curr2": adc_ind("ADC_IND_CURR2"), "curr3": adc_ind("ADC_IND_CURR3"),
        "sens1": adc_ind("ADC_IND_SENS1"), "sens2": adc_ind("ADC_IND_SENS2"), "sens3": adc_ind("ADC_IND_SENS3"),
        "vin": adc_ind("ADC_IND_VIN_SENS"), "temp_mos": adc_ind("ADC_IND_TEMP_MOS"),
        "temp_motor": adc_ind("ADC_IND_TEMP_MOTOR"), "ext1": adc_ind("ADC_IND_EXT"), "ext2": adc_ind("ADC_IND_EXT2"),
        "shunt_ohm": expand(pp, "CURRENT_SHUNT_RES"), "amp_gain": expand(pp, "CURRENT_AMP_GAIN"),
        "vin_r1": expand(pp, "VIN_R1"), "vin_r2": expand(pp, "VIN_R2"), "v_reg": expand(pp, "V_REG"),
        "uart": expand(pp, "HW_UART_DEV"),
        "uart_tx": pin_of(pp, "HW_UART_TX_PORT", "HW_UART_TX_PIN"),
        "uart_rx": pin_of(pp, "HW_UART_RX_PORT", "HW_UART_RX_PIN"),
        "hall1": pin_of(pp, "HW_HALL_ENC_GPIO1", "HW_HALL_ENC_PIN1"),
        "hall2": pin_of(pp, "HW_HALL_ENC_GPIO2", "HW_HALL_ENC_PIN2"),
        "hall3": pin_of(pp, "HW_HALL_ENC_GPIO3", "HW_HALL_ENC_PIN3"),
        "enc_tim": expand(pp, "HW_ENC_TIM"),
        "can_rx": pin_of(pp, "HW_CANRX_PORT", "HW_CANRX_PIN") or ("" if "HW_HAS_NO_CAN" in m else "PB8*"),
        "can_tx": pin_of(pp, "HW_CANTX_PORT", "HW_CANTX_PIN") or ("" if "HW_HAS_NO_CAN" in m else "PB9*"),
        "servo_pin": pin_of(pp, "HW_ICU_GPIO", "HW_ICU_PIN"),
        "spi_sck": pin_of(pp, "HW_SPI_PORT_SCK", "HW_SPI_PIN_SCK"),
        "spi_miso": pin_of(pp, "HW_SPI_PORT_MISO", "HW_SPI_PIN_MISO"),
        "spi_mosi": pin_of(pp, "HW_SPI_PORT_MOSI", "HW_SPI_PIN_MOSI"),
        "spi_nss": pin_of(pp, "HW_SPI_PORT_NSS", "HW_SPI_PIN_NSS"),
        "lim_current": expand(pp, "HW_LIM_CURRENT"), "lim_current_abs": expand(pp, "HW_LIM_CURRENT_ABS"),
        "lim_vin": expand(pp, "HW_LIM_VIN"), "lim_erpm": expand(pp, "HW_LIM_ERPM"),
        "lim_temp_fet": expand(pp, "HW_LIM_TEMP_FET"),
        "f_zv": expand(pp, "MCCONF_FOC_F_ZV"), "max_abs_current": expand(pp, "MCCONF_L_MAX_ABS_CURRENT"),
        "uses_gpiod_gt2": int(bool(re.search(r"GPIO[DE]\s*,\s*([3-9]|1\d)\b", code_txt + " ".join(m.values())))),
        "has_no_can": int("HW_HAS_NO_CAN" in m),
        "dead_time_ns": expand(pp, "HW_DEAD_TIME_NSEC") if "HW_DEAD_TIME_NSEC" in m else "360.0(defaut VESC)",
        "lim_duty_max": expand(pp, "HW_LIM_DUTY_MAX"), "lim_current_in": expand(pp, "HW_LIM_CURRENT_IN"),
        "def_vmin": expand(pp, "MCCONF_L_MIN_VOLTAGE"), "def_vmax": expand(pp, "MCCONF_L_MAX_VOLTAGE"),
        "drv8301_oc_adj": expand(pp, "MCCONF_M_DRV8301_OC_ADJ"),
        "ntc_mos_beta": (re.search(r"/\s*([0-9.]+)\s*\)\s*\+\s*\(1\.0\s*/\s*298\.15", m.get("NTC_TEMP", "")) or [None, ""])[1]
                        or ("custom:" + m.get("NTC_TEMP", "").split("(")[0] if m.get("NTC_TEMP") else ""),
        "ntc_mos_res": m.get("NTC_RES", ""),
        "temp_mos2": adc_ind("ADC_IND_TEMP_MOS_2"), "temp_mos3": adc_ind("ADC_IND_TEMP_MOS_3"),
        "ext3": adc_ind("ADC_IND_EXT3"),
        "phase_filter": pin_of(pp, "PHASE_FILTER_GPIO", "PHASE_FILTER_PIN") or macro_pin(pp, "PHASE_FILTER_ON"),
        "current_filter": macro_pin(pp, "CURRENT_FILTER_ON"),
        "shutdown": pin_of(pp, "HW_SHUTDOWN_GPIO", "HW_SHUTDOWN_PIN"),
        "sensor_5v": pin_of(pp, "SENSOR_VOLTAGE_GPIO", "SENSOR_VOLTAGE_PIN"),
        "led_green": pin_of(pp, "LED_GREEN_GPIO", "LED_GREEN_PIN") or macro_pin(pp, "LED_GREEN_ON"),
        "led_red": pin_of(pp, "LED_RED_GPIO", "LED_RED_PIN") or macro_pin(pp, "LED_RED_ON"),
        "dccal_pin": macro_pin(pp, "DCCAL_ON"),
        "uart_p": expand(pp, "HW_UART_P_DEV"),
        "uart_p_tx": pin_of(pp, "HW_UART_P_TX_PORT", "HW_UART_P_TX_PIN"),
        "uart_p_rx": pin_of(pp, "HW_UART_P_RX_PORT", "HW_UART_P_RX_PIN"),
        "drv_mosi": next((pin_of(pp, f"{d}_MOSI_GPIO", f"{d}_MOSI_PIN") for d in ("DRV8301", "DRV8323S", "DRV8320S", "DRV8305", "DRV8316") if f"{d}_MOSI_GPIO" in m), ""),
        "drv_miso": next((pin_of(pp, f"{d}_MISO_GPIO", f"{d}_MISO_PIN") for d in ("DRV8301", "DRV8323S", "DRV8320S", "DRV8305", "DRV8316") if f"{d}_MISO_GPIO" in m), ""),
        "drv_sck": next((pin_of(pp, f"{d}_SCK_GPIO", f"{d}_SCK_PIN") for d in ("DRV8301", "DRV8323S", "DRV8320S", "DRV8305", "DRV8316") if f"{d}_SCK_GPIO" in m), ""),
        "drv_cs": next((pin_of(pp, f"{d}_CS_GPIO", f"{d}_CS_PIN") for d in ("DRV8301", "DRV8323S", "DRV8320S", "DRV8305", "DRV8316") if f"{d}_CS_GPIO" in m), ""),
        "spi_miso": pin_of(pp, "HW_SPI_PORT_MISO", "HW_SPI_PIN_MISO"),
        "i2c_scl": pin_of(pp, "HW_I2C_SCL_PORT", "HW_I2C_SCL_PIN"),
        "i2c_sda": pin_of(pp, "HW_I2C_SDA_PORT", "HW_I2C_SDA_PIN"),
        "hw_major": expand(pp, "HW_MAJOR"), "hw_minor": expand(pp, "HW_MINOR"),
        "drv_fault_raw": m.get("IS_DRV_FAULT", ""), "en_gate_raw": m.get("ENABLE_GATE", ""),
        "brk_high": int("BRK_HIGH" in m),
        "def_in_max": expand(pp, "MCCONF_L_IN_CURRENT_MAX"), "def_in_min": expand(pp, "MCCONF_L_IN_CURRENT_MIN"),
    }
    for sig, macro in [("curr1", "ADC_IND_CURR1"), ("curr2", "ADC_IND_CURR2"), ("curr3", "ADC_IND_CURR3"),
                       ("sens1", "ADC_IND_SENS1"), ("sens2", "ADC_IND_SENS2"), ("sens3", "ADC_IND_SENS3"),
                       ("vin", "ADC_IND_VIN_SENS"), ("temp_mos", "ADC_IND_TEMP_MOS"),
                       ("temp_mos2", "ADC_IND_TEMP_MOS_2"), ("temp_mos3", "ADC_IND_TEMP_MOS_3"),
                       ("temp_motor", "ADC_IND_TEMP_MOTOR"), ("ext1", "ADC_IND_EXT"),
                       ("ext2", "ADC_IND_EXT2"), ("ext3", "ADC_IND_EXT3")]:
        ch, adc = adc_detail.get(macro, ("", ""))
        row[sig + "_ch"], row[sig + "_adc"] = ch, adc
    return row

def main():
    root, out = sys.argv[1], sys.argv[2]
    idx = index_files(root)
    headers = []
    for d, _, files in os.walk(root):
        for f in files:
            if f.startswith("hw_") and f.endswith(".h") and not f.endswith("_core.h") and "no_limits" not in f:
                headers.append(os.path.join(d, f))
    rows = [r for r in (survey_board(h, idx, root) for h in sorted(headers)) if r]
    with open(out, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"{len(rows)} cartes analysees -> {out}\n")
    for key in ["tim1_pins", "tim8_pins", "use_brk", "brk_pin", "gate_driver", "shunts", "phase_shunts",
                "dual_motor", "inverted_shunt", "en_gate", "drv_fault", "curr1", "curr2", "curr3", "sens1", "sens2",
                "sens3", "vin", "temp_mos", "temp_motor", "ext1", "ext2", "uart", "uart_tx", "uart_rx",
                "hall1", "hall2", "hall3", "enc_tim", "can_rx", "can_tx", "servo_pin", "spi_sck", "spi_nss",
                "shunt_ohm", "amp_gain", "vin_r1", "vin_r2", "f_zv", "uses_gpiod_gt2"]:
        c = Counter(r[key] for r in rows)
        top = ", ".join(f"{k or '-'}:{v}" for k, v in c.most_common(6))
        print(f"{key:15s} {top}")

if __name__ == "__main__":
    main()
