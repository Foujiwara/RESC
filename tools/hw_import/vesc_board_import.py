#!/usr/bin/env python3
"""
Genere des descriptions de carte RESC (statut IMPORTED) a partir du
recensement VESC (docs/data/vesc_hw_survey.csv, produit par
tools/analysis/vesc_hw_survey.py).

Ne genere que les cartes compatibles avec la cible : STM32F405 LQFP64,
mono-moteur, TIM1 sur PA8-PA10/PB13-PB15. Les cartes deja decrites a la main
(familles listees dans MANUAL) sont ignorees.

Tout ce que le recensement ne peut pas extraire de facon fiable (fonctions
specifiques dans le .c, polarites, sondes personnalisees) est signale par un
commentaire « [V] » et justifie le statut IMPORTED : le firmware refuse
d'activer le pont sur ces cartes tant qu'elles n'ont pas ete relues.

Usage : python -I tools/hw_import/vesc_board_import.py docs/data/vesc_hw_survey.csv hw/boards/imported
"""
import csv
import os
import re
import sys

MANUAL = {"UBOX_SINGLE_85_200", "UBOX_SINGLE_80", "UBOX_SINGLE_100", "UBOX_V2_100"}
STD_TIM1 = "PA10 PA8 PA9 PB13 PB14 PB15"
GD = {"DRV8301": "HW_GD_DRV8301", "DRV8323S": "HW_GD_DRV8323S", "DRV8320S": "HW_GD_DRV8320S",
      "DRV8305": "HW_GD_DRV8305", "DRV8313": "HW_GD_DRV8313"}
SPI_GD = {"HW_GD_DRV8301", "HW_GD_DRV8323S", "HW_GD_DRV8320S", "HW_GD_DRV8305"}
VESC_DEF = {"f_zv": 25000.0, "max_abs_current": 130.0, "def_vmin": 8.0, "def_vmax": 57.0,
            "def_in_max": 99.0, "def_in_min": -60.0}   # motor/mcconf_default.h (7.00)


def num(s):
    try:
        return float(eval(s.replace("f", ""), {"__builtins__": {}}, {}))
    except Exception:
        return None


def rng(s):
    parts = [num(p) for p in s.split(",")] if s else []
    return parts if len(parts) == 2 and None not in parts else None


def pin(s):
    s = (s or "").rstrip("*")
    return f"HW_{s}" if re.fullmatch(r"P[A-E]\d{1,2}", s) else "HW_PIN_NONE"


def ident(name):
    return re.sub(r"\W", "_", name).upper()


def gen(r):
    notes = []
    L = []
    w = L.append
    name = r["hw_name"]
    g = "HW_GD_GENERIC"
    for k, v in GD.items():
        if k in r["gate_driver"]:
            g = v
    if g == "HW_GD_GENERIC" and r["dccal_pin"]:
        g = "HW_GD_DRV8302"
        notes.append("driver deduit DRV8302 (DC_CAL present, pas de SPI)")

    w(f"#define BOARD_NAME              \"{name}\"")
    maj = r["hw_major"] if r["hw_major"].isdigit() else "0"
    mnr = r["hw_minor"] if r["hw_minor"].isdigit() else "0"
    w(f"#define BOARD_HW_MAJOR          {maj}")
    w(f"#define BOARD_HW_MINOR          {mnr}")
    w(f"#define BOARD_ORIGIN            \"vesc-7.00:hwconf/{r['file']}\"")
    w("#define BOARD_STATUS            HW_BOARD_STATUS_IMPORTED")
    w("")
    w("/* Etage de puissance */")
    w(f"#define BOARD_GATE_DRIVER       {g}")
    dt = r["dead_time_ns"]
    dtv = num(dt.split("(")[0]) if dt else None
    if "defaut" in dt:
        notes.append("dead-time non defini par la carte : defaut VESC 360 ns")
    if dtv == 0:
        w("#define BOARD_DEADTIME_NS       0")
        w("#define BOARD_GD_INTERNAL_DEADTIME 1   /* [V] VESC impose 0 ns : le driver doit inserer son propre temps mort */")
        notes.append("dead-time 0 ns dans VESC")
    else:
        w(f"#define BOARD_DEADTIME_NS       {int(round(dtv or 360))}")
    if r["en_gate"]:
        w(f"#define BOARD_GD_EN_PIN         {pin(r['en_gate'])}")
        if "palClearPad" in r["en_gate_raw"]:
            w("#define BOARD_GD_EN_ACTIVE_HIGH 0")
    if r["drv_fault"]:
        w(f"#define BOARD_GD_FAULT_PIN      {pin(r['drv_fault'])}")
        if r["drv_fault_raw"] and "!" not in r["drv_fault_raw"]:
            w("#define BOARD_GD_FAULT_ACTIVE_LOW 0")
    elif r["drv_fault_raw"]:
        notes.append(f"detection de defaut driver par fonction specifique : {r['drv_fault_raw']}")
    if g in SPI_GD:
        for k, c in [("SCK", "drv_sck"), ("MISO", "drv_miso"), ("MOSI", "drv_mosi"), ("CS", "drv_cs")]:
            w(f"#define BOARD_GD_SPI_{k}_PIN{' ' * (6 - len(k))}  {pin(r[c])}")
    if r["dccal_pin"]:
        w(f"#define BOARD_GD_DCCAL_PIN      {pin(r['dccal_pin'])}")
    w("")
    w("/* Protection materielle */")
    if r["use_brk"] == "1":
        w(f"#define BOARD_BKIN_PIN          {pin(r['brk_pin'])}")
        w(f"#define BOARD_BKIN_ACTIVE_HIGH  {1 if r['brk_high'] == '1' else 0}")
        ocp = 3
    elif r["drv_fault"]:
        ocp = 2
    elif g in SPI_GD or g == "HW_GD_DRV8302":
        ocp = 1
    else:
        ocp = 0
    w(f"#define BOARD_HW_OCP            {ocp}   /* [V] deduit du HW VESC */")
    w("")
    w("/* Mesure de courant */")
    topo = "HW_CS_INLINE_3" if r["phase_shunts"] == "1" else ("HW_CS_LOWSIDE_3" if r["shunts"] == "3" else "HW_CS_LOWSIDE_2")
    w(f"#define BOARD_CS_TOPOLOGY       {topo}")
    for ph, c in (("A", "curr1"), ("B", "curr2"), ("C", "curr3")):
        if r[c + "_ch"]:
            w(f"#define BOARD_CS_{ph}_CH           {r[c + '_ch']}")
            w(f"#define BOARD_CS_{ph}_ADC          {r[c + '_adc']}")
        elif ph == "C" and topo == "HW_CS_LOWSIDE_2":
            w("#define BOARD_CS_C_CH           HW_ADC_CH_NONE")
        else:
            notes.append(f"voie ADC du courant {ph} non resolue")
    sh, gn = num(r["shunt_ohm"]), num(r["amp_gain"])
    w(f"#define BOARD_CS_SHUNT_OHM      ((float){r['shunt_ohm']})" if sh else "/* [V] BOARD_CS_SHUNT_OHM non resolu */")
    w(f"#define BOARD_CS_AMP_GAIN       ((float){r['amp_gain']})" if gn else "/* [V] BOARD_CS_AMP_GAIN non resolu */")
    if not (sh and gn):
        notes.append("shunt ou gain non resolu (expression ou macro specifique)")
    w(f"#define BOARD_CS_INVERTED       {r['inverted_shunt'] or 0}")
    if r["current_filter"]:
        w(f"#define BOARD_CS_FILTER_PIN     {pin(r['current_filter'])}")
    w("")
    w("/* Mesure de tension */")
    vref = num(r["v_reg"]) or 3.3
    w(f"#define BOARD_VREF_V            {vref}f")
    r1, r2 = num(r["vin_r1"]), num(r["vin_r2"])
    if r1 and r2:
        w(f"#define BOARD_VBUS_R_TOP_OHM    {r1}f")
        w(f"#define BOARD_VBUS_R_BOT_OHM    {r2}f")
    else:
        notes.append("diviseur Vbus non resolu")
    w("#define BOARD_VPH_R_TOP_OHM     BOARD_VBUS_R_TOP_OHM   /* [V] VESC utilise le diviseur VIN */")
    w("#define BOARD_VPH_R_BOT_OHM     BOARD_VBUS_R_BOT_OHM")
    if r["vin_ch"]:
        w(f"#define BOARD_VBUS_CH           {r['vin_ch']}")
        w(f"#define BOARD_VBUS_ADC          {r['vin_adc']}")
    for ph, c in (("A", "sens1"), ("B", "sens2"), ("C", "sens3")):
        if r[c + "_ch"]:
            w(f"#define BOARD_VPH_{ph}_CH          {r[c + '_ch']}")
            w(f"#define BOARD_VPH_{ph}_ADC         {r[c + '_adc']}")
    if r["phase_filter"]:
        w(f"#define BOARD_PHASE_FILTER_PIN  {pin(r['phase_filter'])}")
    w("")
    w("/* Temperatures */")
    beta = num(r["ntc_mos_beta"]) if not r["ntc_mos_beta"].startswith("custom") else None
    pos = "HW_TEMP_NTC_TO_GND" if "(10000.0 / ((4095.0" in r["ntc_mos_res"] else "HW_TEMP_NTC_TO_VREF"
    if r["temp_mos2_ch"] and r["temp_mos3_ch"]:
        w("#define BOARD_TFET_MODEL        HW_TEMP_MAX_OF_3")
        w(f"#define BOARD_TFET_NTC_POS      {pos}")
        w(f"#define BOARD_TFET2_CH          {r['temp_mos2_ch']}")
        w(f"#define BOARD_TFET3_CH          {r['temp_mos3_ch']}")
    else:
        w(f"#define BOARD_TFET_MODEL        {pos}")
    if r["temp_mos_ch"]:
        w(f"#define BOARD_TFET_CH           {r['temp_mos_ch']}")
    w("#define BOARD_TFET_NTC_R25_OHM  10000.0f")
    w(f"#define BOARD_TFET_NTC_BETA     {beta or 3380.0}f")
    w("#define BOARD_TFET_R_FIXED_OHM  10000.0f")
    if beta is None:
        notes.append(f"sonde FET specifique ({r['ntc_mos_beta'] or 'inconnue'}) : beta 3380 suppose")
    if r["temp_motor_ch"]:
        w(f"#define BOARD_TMOT_CH           {r['temp_motor_ch']}")
    w("")
    w("/* Annexes et communications */")
    for macro, c in (("BOARD_ADC_EXT1_CH", "ext1_ch"), ("BOARD_ADC_EXT2_CH", "ext2_ch"), ("BOARD_ADC_EXT3_CH", "ext3_ch")):
        w(f"#define {macro:<23} {r[c] or 'HW_ADC_CH_NONE'}")
    if not r["temp_motor_ch"]:
        w("#define BOARD_TMOT_CH           HW_ADC_CH_NONE")
    for macro, c in (("BOARD_LED_GREEN_PIN", "led_green"), ("BOARD_LED_RED_PIN", "led_red"),
                     ("BOARD_PWR_HOLD_PIN", "shutdown"), ("BOARD_SENSOR_5V_PIN", "sensor_5v"),
                     ("BOARD_HALL1_PIN", "hall1"), ("BOARD_HALL2_PIN", "hall2"), ("BOARD_HALL3_PIN", "hall3"),
                     ("BOARD_SERVO_PIN", "servo_pin"), ("BOARD_SPI_SCK_PIN", "spi_sck"),
                     ("BOARD_SPI_MISO_PIN", "spi_miso"), ("BOARD_SPI_MOSI_PIN", "spi_mosi"),
                     ("BOARD_SPI_NSS_PIN", "spi_nss"), ("BOARD_I2C_SCL_PIN", "i2c_scl"), ("BOARD_I2C_SDA_PIN", "i2c_sda")):
        w(f"#define {macro:<23} {pin(r[c])}")   # absent dans VESC => HW_PIN_NONE (pas de defaut commun)
    if r["enc_tim"] in ("TIM3", "TIM4"):
        w(f"#define BOARD_ENC_TIM           {r['enc_tim'][-1]}")
    m = re.fullmatch(r"SD(\d)", r["uart"])
    if m:
        w(f"#define BOARD_UART_INST         {m.group(1)}")
        w(f"#define BOARD_UART_TX_PIN       {pin(r['uart_tx'])}")
        w(f"#define BOARD_UART_RX_PIN       {pin(r['uart_rx'])}")
    m = re.fullmatch(r"SD(\d)", r["uart_p"])
    if m and r["uart_p_tx"]:
        w(f"#define BOARD_UART2_INST        {m.group(1)}")
        w(f"#define BOARD_UART2_TX_PIN      {pin(r['uart_p_tx'])}")
        w(f"#define BOARD_UART2_RX_PIN      {pin(r['uart_p_rx'])}")
    if r["has_no_can"] == "1":
        w("#define BOARD_HAS_CAN           0")
    w("")
    w("/* Limites absolues (HW_LIM_* VESC) */")
    lc, li, la, lv, le, ld, lt = (rng(r[k]) for k in ("lim_current", "lim_current_in", "lim_current_abs",
                                                       "lim_vin", "lim_erpm", "lim_duty_max", "lim_temp_fet"))
    if not all((lc, la, lv)):
        notes.append("limites HW_LIM_* incompletes")
    i_mot = int(max(abs(x) for x in lc)) if lc else 0
    i_abs = int(la[1]) if la else 0
    i_in = int(max(abs(x) for x in li)) if li else i_mot
    w(f"#define HWLIM_I_MOTOR_A         {i_mot}")
    w(f"#define HWLIM_I_INPUT_A         {i_in}")
    w(f"#define HWLIM_I_ABS_TRIP_A      {i_abs}")
    vmin, vmax = (int(-(-lv[0] // 1)), int(lv[1])) if lv else (0, 0)
    w(f"#define HWLIM_VBUS_MIN_V        {max(vmin, 0)}")
    w(f"#define HWLIM_VBUS_MAX_V        {vmax}")
    w(f"#define HWLIM_ERPM              {int(le[1]) if le else 200000}")
    w(f"#define HWLIM_DUTY_MAX_PERMILLE {int(round(ld[1] * 1000)) if ld else 950}")
    w(f"#define HWLIM_T_FET_C           {int(lt[1]) if lt else 100}")
    w("")
    w("/* Defauts (MCCONF_* de la carte, sinon mcconf_default.h de VESC 7.00) */")
    def dv(k):
        v = num(r[k])
        return VESC_DEF[k] if v is None else v
    fpwm = int(dv("f_zv") / 2)
    w(f"#define DEF_F_PWM_HZ            {fpwm}   /* f_zv / 2 */")
    abs_def = min(int(dv("max_abs_current")), i_abs) if i_abs else int(dv("max_abs_current"))
    w(f"#define DEF_I_ABS_TRIP_A        {abs_def}")
    w(f"#define DEF_I_INPUT_MAX_A       {min(int(dv('def_in_max')), i_in)}")
    w(f"#define DEF_I_INPUT_MIN_A       ({max(int(dv('def_in_min')), -i_in)})")
    dmin = min(max(int(-(-dv('def_vmin') // 1)), max(vmin, 0)), max(vmax - 1, 0))
    dmax = max(min(int(dv('def_vmax')), vmax), dmin + 1)
    w(f"#define DEF_VBUS_MIN_V          {dmin}")
    w(f"#define DEF_VBUS_MAX_V          {dmax}")
    if dmax != int(dv("def_vmax")) or dmin != int(-(-dv('def_vmin') // 1)):
        notes.append("defauts de tension VESC ramenes dans les limites absolues")
    if r["src"] == "":
        notes.append("source .c introuvable : affectation ADC non verifiee")
    notes.append("fonctions specifiques du .c VESC (alimentation, bouton, capteurs) non reprises")
    return L, notes


def main():
    src, out = sys.argv[1], sys.argv[2]
    rows = list(csv.DictReader(open(src, encoding="utf-8")))
    n = 0
    seen = set()
    for r in rows:
        if r["hw_name"] in MANUAL or r["tim1_pins"] != STD_TIM1 or r["dual_motor"] == "1" or r["uses_gpiod_gt2"] == "1":
            continue
        rel = os.path.splitext(r["file"])[0]
        vendor, base = os.path.dirname(rel).split("/")[0], os.path.basename(rel)
        fn = os.path.join(out, vendor.lower(), "board_" + base.replace("hw_", "", 1).lower() + ".h")
        if fn in seen:
            continue
        seen.add(fn)
        body, notes = gen(r)
        guard = "BOARD_" + ident(vendor + "_" + base) + "_H"
        os.makedirs(os.path.dirname(fn), exist_ok=True)
        with open(fn, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(f"/*\n * {os.path.basename(fn)} — {r['hw_name']}\n *\n")
            fh.write(f" * GENERE AUTOMATIQUEMENT par tools/hw_import/vesc_board_import.py\n")
            fh.write(f" * depuis bldc release_7_00 : hwconf/{r['file']} (+ {r['src'] or '?'}).\n")
            fh.write(" * Statut IMPORTED : etage de puissance desactive tant que la carte n'a\n")
            fh.write(" * pas ete relue contre les sources VESC (puis passee en REVIEWED).\n *\n")
            fh.write(" * Points a verifier [V] :\n")
            for note in notes:
                fh.write(f" *  - {note}\n")
            fh.write(" */\n")
            fh.write(f"#ifndef {guard}\n#define {guard}\n\n")
            fh.write("\n".join(body))
            fh.write(f"\n\n#endif /* {guard} */\n")
        n += 1
    print(f"{n} cartes generees dans {out}")


if __name__ == "__main__":
    main()
