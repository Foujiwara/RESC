#!/usr/bin/env python3
"""
Affichage conditionnel des parametres dans RESC Tool (commande 250).

La description XML envoyee par le firmware est faite de deux parties :
  - <Params> et <SerOrder> : fixes, compresses a la compilation (flux zlib termine par un
    vidage complet, sans bloc final) ;
  - <Grouping> (onglets) : produit par le firmware a chaque demande, d'apres la configuration
    en cours, et ajoute en blocs deflate non compresses, suivis de l'adler32 de l'ensemble.
Un parametre n'est range dans son onglet que si sa condition est vraie pour la configuration
actuelle (type de moteur, source d'angle, application, options...). Les separateurs sans
parametre visible, les onglets et les groupes vides disparaissent. La signature ne depend pas
des onglets : elle ne change pas.

Les conditions sont des expressions C sur la configuration (mc-> : moteur, app-> : application),
avec les valeurs internes (numerotation VESC). Elles ne s'appliquent qu'au profil RESC ; le profil
VESC700 garde ses onglets fixes.

Variante complete (bit 7 de l'octet conf de la requete, 0x80 | conf) : tous les onglets, avec les
conditions traduites pour RESC Tool (attribut visibleIf sur <group>, <subgroup>, <param>) et une
section <Pages>. RESC Tool les evalue lui-meme pendant l'edition. Grammaire : || && ! == != < <= > >=,
NAME in [a, b], NAME & N, parentheses ; NAME = nom XML du parametre ; un enum se compare a sa position
dans la liste (liste reduite par la carte), un booleen a 0 / 1. Les conditions qui ne dependent que
de la carte (BOARD_*) restent evaluees par le firmware : le parametre est present ou absent.
"""
import re
import struct
import zlib

# ------------------------------------------------------------------ conditions (profil RESC)
IM = "(mc->foc_motor_kind == 1)"                     # moteur a induction
PM = "(mc->foc_motor_kind != 1)"                     # moteur a aimants
SM = "mc->foc_sensor_mode"                           # 0 sans capteur, 1 codeur, 2 Hall, 3 HFI, 4 VSS
ENC = "(%s == 1)" % SM
SL = "(%s == 0 || (%s && %s != 1))" % (SM, IM, SM)   # induction : codeur ou sans capteur
HALL = "(%s && %s == 2)" % (PM, SM)
HFI = "(%s && %s == 3)" % (PM, SM)
VSS = "(%s && %s == 4)" % (PM, SM)
IFS = "(%s || %s)" % (SL, VSS)                       # demarrage I/f
# observateur utilise sauf codeur seul (aimants : sauf si le defaut de glissement du codeur le compare)
OBS = "!(%s && (%s || !(mc->l_additional_faults & 1)))" % (ENC, IM)
TT = "mc->m_motor_temp_sens_type"                    # 0 NTC 10k, 1 PTC 1k, 3 NTC 100k, 5 NTCX, 6 PTCX, 8 aucun
TEMP = "(%s != 8)" % TT


def any_of(x, vals):
    return "(" + " || ".join("%s == %d" % (x, v) for v in vals) + ")"


def both(*c):
    return "(" + " && ".join(c) + ")"


PORT = "mc->m_sensor_port_mode"
BMS = "(mc->bms_type == 1)"
VIS_MC = {
    "bms.limit_mode": BMS,
    "bms.fwd_can_mode": BMS,
    "bms.t_limit_start": both(BMS, "(mc->bms_limit_mode & 1)"),
    "bms.t_limit_end": both(BMS, "(mc->bms_limit_mode & 1)"),
    "bms.soc_limit_start": both(BMS, "(mc->bms_limit_mode & 2)"),
    "bms.soc_limit_end": both(BMS, "(mc->bms_limit_mode & 2)"),
    "bms.vmin_limit_start": both(BMS, "(mc->bms_limit_mode & 4)"),
    "bms.vmin_limit_end": both(BMS, "(mc->bms_limit_mode & 4)"),
    "bms.vmax_limit_start": both(BMS, "(mc->bms_limit_mode & 8)"),
    "bms.vmax_limit_end": both(BMS, "(mc->bms_limit_mode & 8)"),
    "m_sensor_port_mode": ENC,
    "m_encoder_counts": both(ENC, "(%s == 1 || %s == 14)" % (PORT, PORT)),
    "m_encoder_sin_amp": both(ENC, "(%s == 4)" % PORT),
    "m_encoder_cos_amp": both(ENC, "(%s == 4)" % PORT),
    "m_encoder_sin_offset": both(ENC, "(%s == 4)" % PORT),
    "m_encoder_cos_offset": both(ENC, "(%s == 4)" % PORT),
    "m_encoder_sincos_filter_constant": both(ENC, "(%s == 4)" % PORT),
    "m_encoder_sincos_phase_correction": both(ENC, "(%s == 4)" % PORT),
    "m_ntc_motor_beta": any_of(TT, (0, 3, 5)),
    "m_ptc_motor_coeff": any_of(TT, (1, 6)),
    "m_ntcx_ptcx_res": any_of(TT, (5, 6)),
    "m_ntcx_ptcx_temp_base": any_of(TT, (5, 6)),
    "l_temp_motor_start": TEMP,
    "l_temp_motor_end": TEMP,
    "foc_temp_comp": TEMP,
    "foc_temp_comp_base_temp": both(TEMP, "mc->foc_temp_comp"),
    # aimants seulement
    "foc_motor_ld_lq_diff": PM,
    "foc_motor_flux_linkage": PM,
    "foc_mtpa_mode": PM,
    "foc_fw_current_max": PM,
    "foc_fw_duty_start": both(PM, "mc->foc_fw_current_max > 0.0f"),
    "foc_fw_ramp_time": both(PM, "mc->foc_fw_current_max > 0.0f"),
    "foc_fw_q_current_factor": both(PM, "mc->foc_fw_current_max > 0.0f"),
    # induction seulement
    "foc_acim_lm": IM, "foc_acim_lr": IM, "foc_acim_rr": IM, "foc_acim_im": IM, "foc_acim_base_erpm": IM,
    # observateur (type impose en induction)
    "foc_observer_gain": OBS,
    "foc_observer_gain_slow": OBS,
    "foc_observer_offset": OBS,
    "foc_pll_kp": OBS,
    "foc_pll_ki": OBS,
    "foc_observer_type": both(PM, OBS),
    # demarrage I/f (alignement : aimants sans capteur ; l'induction se magnetise, VSS part de l'injection)
    "foc_openloop_rpm": IFS,
    "foc_openloop_rpm_low": IFS,
    "foc_sl_openloop_hyst": IFS,
    "foc_sl_openloop_time_ramp": IFS,
    "foc_sl_openloop_time": IFS,
    "foc_sl_openloop_boost_q": IFS,
    "foc_sl_openloop_max_q": IFS,
    "foc_sl_openloop_time_lock": both(PM, "%s == 0" % SM),
    "foc_sat_comp": "mc->foc_sat_comp_mode != 0",
    # sondes Hall
    "foc_sl_erpm": HALL,
    "foc_sl_erpm_start": HALL,
    "foc_hall_interp_erpm": HALL,
    "m_hall_extra_samples": HALL,
    # codeur
    "foc_encoder_offset": ENC,
    "foc_encoder_ratio": ENC,
    "foc_encoder_inverted": ENC,
    # injection : position a l'arret (HFI et VSS), suivi en marche (HFI)
    "foc_hfi_amb_mode": "(%s || %s)" % (HFI, VSS),
    "foc_hfi_amb_current": "(%s || %s)" % (HFI, VSS),
    "foc_hfi_amb_tres": "(%s || %s)" % (HFI, VSS),
    "foc_hfi_samples": "(%s || %s)" % (HFI, VSS),
    "foc_hfi_voltage_start": "(%s || %s)" % (HFI, VSS),
    "foc_hfi_gain": "(%s || %s)" % (HFI, VSS),
    "foc_hfi_max_err": "(%s || %s)" % (HFI, VSS),
    "foc_hfi_start_samples": "(%s || %s)" % (HFI, VSS),
    "foc_hfi_voltage_run": HFI,
    "foc_hfi_voltage_max": HFI,
    "foc_hfi_hyst": HFI,
    "foc_sl_erpm_hfi": HFI,
    "foc_hfi_reset_erpm": "(%s || %s)" % (HFI, VSS),
    "foc_hfi_obs_ovr_sec": HFI,
    "foc_fw_backoff": both(PM, "mc->foc_fw_current_max > 0.0f"),
    "l_in_current_map_filter": "mc->l_in_current_map_start < 0.98f",
    # trois capteurs de courant (sinon un seul mode possible)
    "foc_current_sample_mode": "(BOARD_CS_TOPOLOGY != HW_CS_LOWSIDE_2)",
    # filtre de phase
    "foc_phase_filter_max_erpm": "mc->foc_phase_filter_enable",
    # regulateur de descente du mode duty : remplace par la roue libre en mode duty simple
    "foc_duty_dowmramp_kp": "!mc->foc_resc_duty_basic",
    "foc_duty_dowmramp_ki": "!mc->foc_resc_duty_basic",
    "foc_phase_filter_disable_fault": "mc->foc_phase_filter_enable",
}
for _k in range(8):
    VIS_MC["foc_hall_table__%d" % _k] = HALL

PPM = "app->app_ppm_conf_ctrl_type"      # 2, 5, 7, 11 sans marche arriere ; 6, 7 PID ; 8 hysteresis ; 9 intelligente
ADC = "app->app_adc_conf_ctrl_type"      # centrees : 2, 5, 6, 10, 13 ; frein ADC2 : 4, 8
VIS_APP = {
    "can_status_msgs_r1": "(app->can_status_rate_1 > 0 && app->can_mode == 0)",
    "can_status_msgs_r2": "(app->can_status_rate_2 > 0 && app->can_mode == 0)",
    "can_status_rate_1": "app->can_mode == 0",
    "can_status_rate_2": "app->can_mode == 0",
    "servo_out_enable": "(app->app_to_use != 1 && app->app_to_use != 4)",
    "permanent_uart_enabled": "(BOARD_UART2_INST != 0)",
    "app_ppm_conf.pulse_center": "!" + any_of(PPM, (2, 5, 7, 11)),
    "app_ppm_conf.pid_max_erpm": any_of(PPM, (6, 7)),
    "app_ppm_conf.max_erpm_for_dir": any_of(PPM, (8, 9)),
    "app_ppm_conf.smart_rev_max_duty": "%s == 9" % PPM,
    "app_ppm_conf.smart_rev_ramp_time": "%s == 9" % PPM,
    "app_ppm_conf.tc": "app->app_ppm_conf_multi_esc",
    "app_ppm_conf.tc_max_diff": "(app->app_ppm_conf_multi_esc && app->app_ppm_conf_tc)",
    "app_adc_conf.tc": "app->app_adc_conf_multi_esc",
    "app_adc_conf.tc_max_diff": "(app->app_adc_conf_multi_esc && app->app_adc_conf_tc)",
    "app_adc_conf.resc_smart_rev": any_of(ADC, (2, 3, 4, 5)),
    "app_adc_conf.resc_smart_rev_erpm": both(any_of(ADC, (2, 3, 4, 5)), "app->app_adc_conf_resc_smart_rev"),
    "app_adc_conf.resc_smart_rev_max_duty": both(any_of(ADC, (2, 3, 4, 5)), "app->app_adc_conf_resc_smart_rev"),
    "app_adc_conf.resc_smart_rev_ramp_time": both(any_of(ADC, (2, 3, 4, 5)), "app->app_adc_conf_resc_smart_rev"),
    "app_adc_conf.resc_hybrid": "((%s >= 1 && %s <= 8) || %s == 15)" % (ADC, ADC, ADC),
    "app_adc_conf.resc_hybrid_duty": "(((%s >= 1 && %s <= 8) || %s == 15) && app->app_adc_conf_resc_hybrid)" % (ADC, ADC, ADC),
    "app_adc_conf.resc_hybrid_band": "(((%s >= 1 && %s <= 8) || %s == 15) && app->app_adc_conf_resc_hybrid)" % (ADC, ADC, ADC),
    "app_chuk_conf.smart_rev_max_duty": "app->app_chuk_conf_use_smart_rev",
    "app_chuk_conf.smart_rev_ramp_time": "app->app_chuk_conf_use_smart_rev",
    "app_chuk_conf.coast_brake_ramp_time": "app->app_chuk_conf_coast_brake_level > 0.0f",
    "app_chuk_conf.tc": "app->app_chuk_conf_multi_esc",
    "app_chuk_conf.tc_max_diff": "(app->app_chuk_conf_multi_esc && app->app_chuk_conf_tc)",
}
# ADC2 toujours visible : a calibrer avant de choisir le type de controle
VIS_APP["app_adc_conf.resc_adc2_brake_prio"] = "%s == 15" % ADC
# App > Control Settings
_BRK = "app->resc_ctrl_brk_enable"
_TR = "app->resc_ctrl_tr_enable"
_RV = "app->resc_ctrl_rev_mode"
for _p in ("stop_erpm", "rebrake_margin", "fade_time", "brk_mode", "rev_mode", "rev_repress"):
    VIS_APP["resc_ctrl." + _p] = _BRK
VIS_APP["resc_ctrl.soft_stop_erpm"] = both(_BRK, "app->resc_ctrl_brk_mode == 0")
VIS_APP["resc_ctrl.brk_force_stop"] = both(_BRK, "app->resc_ctrl_brk_mode == 0")
VIS_APP["resc_ctrl.rev_cur_max"] = both(_BRK, any_of(_RV, (0, 3)))
VIS_APP["resc_ctrl.rev_duty_max"] = both(_BRK, "%s != 1" % _RV)
VIS_APP["resc_ctrl.rev_soft_zone"] = both(_BRK, any_of(_RV, (0, 3)))
VIS_APP["resc_ctrl.rev_ramp"] = both(_BRK, "%s != 1" % _RV)
VIS_APP["resc_ctrl.rev_arm_level"] = both(_BRK, "app->resc_ctrl_rev_repress")
VIS_APP["resc_ctrl.rev_disarm_time"] = both(_BRK, "app->resc_ctrl_rev_repress")
for _p in ("tr_mode", "tr_speed_mode", "tr_max_erpm", "tr_free_play", "tr_max_anchor"):
    VIS_APP["resc_ctrl." + _p] = _TR
for _p in ("tr_stiffness", "tr_damping", "tr_max_current"):
    VIS_APP["resc_ctrl." + _p] = both(_TR, "app->resc_ctrl_tr_mode == 0")
for _p in ("tr_duty_stiffness", "tr_duty_damping", "tr_max_duty"):
    VIS_APP["resc_ctrl." + _p] = both(_TR, "app->resc_ctrl_tr_mode == 1")
IMU_ON = "(app->imu_conf_type != 0)"
for _p in ("mode", "filter", "accel_lowpass_filter_x", "accel_lowpass_filter_y", "accel_lowpass_filter_z",
           "gyro_lowpass_filter", "sample_rate_hz", "accel_confidence_decay", "rot_roll", "rot_pitch", "rot_yaw",
           "accel_offsets__0", "accel_offsets__1", "accel_offsets__2", "gyro_offsets__0", "gyro_offsets__1",
           "gyro_offsets__2"):
    VIS_APP["imu_conf." + _p] = IMU_ON
VIS_APP["imu_conf.mahony_kp"] = both(IMU_ON, "app->imu_conf_mode == 1")
VIS_APP["imu_conf.mahony_ki"] = both(IMU_ON, "app->imu_conf_mode == 1")
VIS_APP["imu_conf.madgwick_beta"] = both(IMU_ON, "app->imu_conf_mode != 1")

# ------------------------------------------- variante complete : groupes et pages (RESC Tool)
NEVER = "0"
APP_USE = "app->app_to_use"     # 1 PPM, 2 ADC, 3 UART, 4 PPM+UART, 5 ADC+UART, 6 Nunchuk, 7 NRF, 9 PAS, 10 ADC+PAS
GROUP_APP = {
    "PPM": any_of(APP_USE, (1, 4)),
    "ADC": any_of(APP_USE, (2, 5, 10)),
    "VESC Remote": any_of(APP_USE, (3, 4, 5, 6, 7)),
    "PAS": any_of(APP_USE, (9, 10)),
}
PAGES_MC = [("motor_bldc", NEVER), ("motor_dc", NEVER), ("motor_gpdrive", NEVER)]
PAGES_APP = [("app_ppm", GROUP_APP["PPM"]), ("app_adc", GROUP_APP["ADC"]), ("app_vescremote", GROUP_APP["VESC Remote"]),
             ("app_nrf", NEVER), ("app_pas", GROUP_APP["PAS"])]
FALSE_EXPR = {"mc": "foc_motor_kind < 0", "app": "app_to_use < 0"}    # toujours faux (position >= 0)
FULL = {"mc": ({}, PAGES_MC), "app": (GROUP_APP, PAGES_APP)}

# ------------------------------------------------ traduction C -> visibleIf
_TOK = re.compile(r"\s*(\|\||&&|==|!=|<=|>=|<|>|!|&|\(|\)|(?:mc|app)->\w+|[A-Za-z_]\w*|\d+(?:\.\d*)?f?)")
_OPS = {"==": lambda a, b: a == b, "!=": lambda a, b: a != b, "<": lambda a, b: a < b,
        "<=": lambda a, b: a <= b, ">": lambda a, b: a > b, ">=": lambda a, b: a >= b}


def _parse(expr):
    e = expr.strip()
    toks, i = [], 0
    while i < len(e):
        m = _TOK.match(e, i)
        if not m:
            raise SystemExit("condition illisible : " + expr)
        toks.append(m.group(1))
        i = m.end()
    pos = [0]

    def peek():
        return toks[pos[0]] if pos[0] < len(toks) else None

    def take(x=None):
        v = peek()
        if v is None or (x is not None and v != x):
            raise SystemExit("condition illisible : " + expr)
        pos[0] += 1
        return v

    def orx():
        a = [andx()]
        while peek() == "||":
            take()
            a.append(andx())
        return a[0] if len(a) == 1 else ("or", a)

    def andx():
        a = [unary()]
        while peek() == "&&":
            take()
            a.append(unary())
        return a[0] if len(a) == 1 else ("and", a)

    def unary():
        if peek() == "!":
            take()
            return ("not", unary())
        a = bit()
        if peek() in _OPS:
            return ("cmp", take(), a, bit())
        return a

    def bit():
        a = prim()
        if peek() == "&":
            take()
            return ("bit", a, prim())
        return a

    def prim():
        v = take()
        if v == "(":
            x = orx()
            take(")")
            return x
        if "->" in v:
            return ("id", v.split("->")[1])
        if v[0].isdigit():
            return ("num", float(v.rstrip("f")))
        return ("board", v)

    t = orx()
    if peek() is not None:
        raise SystemExit("condition illisible : " + expr)
    return t


def _kinds(t, acc):
    acc.add(t[0])
    for c in t[1:]:
        if isinstance(c, tuple):
            _kinds(c, acc)
        elif isinstance(c, list):
            for x in c:
                _kinds(x, acc)
    return acc


def _num(v):
    return "%d" % v if v == int(v) else repr(v)


class Vif:
    """Traduit les conditions C (valeurs internes) en conditions RESC Tool (positions dans les listes)."""

    def __init__(self, root, emap, key):
        self.emap = emap or {}
        self.false = FALSE_EXPR[key]
        self.p = {}
        for e in root.find("Params"):
            self.p[flat(e.tag)] = (e.tag, e.findtext("type"), len(e.findall("enumNames")))

    def field(self, f):
        if f not in self.p:
            raise SystemExit("condition : champ inconnu " + f)
        return self.p[f]

    def enum_sat(self, t):
        """(champ, positions satisfaites, nombre de positions) si t compare un enum a une valeur."""
        if t[0] == "cmp" and t[2][0] == "id" and t[3][0] == "num" and self.field(t[2][1])[1] == "4":
            f, op, v = t[2][1], t[1], t[3][1]
            n = self.field(f)[2]
            keep = self.emap[f]["keep"] if f in self.emap else list(range(n))
            return f, {i for i, x in enumerate(keep) if _OPS[op](x, v)}, len(keep)
        return None

    def set_expr(self, f, sat, n):
        name = self.field(f)[0]
        sat = sorted(sat)
        uns = [i for i in range(n) if i not in sat]
        if not uns:
            return True
        if not sat:
            return False
        if len(sat) == 1:
            return "%s == %d" % (name, sat[0])
        if len(uns) == 1:
            return "%s != %d" % (name, uns[0])
        if 2 * len(uns) <= len(sat):
            return "!(%s in [%s])" % (name, ", ".join(str(i) for i in uns))
        return "%s in [%s]" % (name, ", ".join(str(i) for i in sat))

    def cmp(self, f, op, v):
        name, typ, n = self.field(f)
        if typ == "4":
            return self.set_expr(*self.enum_sat(("cmp", op, ("id", f), ("num", v))))
        if typ == "5" and op == "!=" and v == 0:
            return "%s == 1" % name
        return "%s %s %s" % (name, op, _num(v))

    def emit(self, t):
        k = t[0]
        if k in ("or", "and"):
            # comparaisons d'un meme enum regroupees : union (ou) / intersection (et) des positions
            sets, rest = {}, []
            for c in t[1]:
                es = self.enum_sat(c)
                if es is None:
                    rest.append(c)
                    continue
                f, sat, n = es
                if f in sets:
                    sets[f] = (sets[f][0] | sat if k == "or" else sets[f][0] & sat, n)
                else:
                    sets[f] = (sat, n)
                    rest.append(("set", f))
            parts = [self.set_expr(c[1], *sets[c[1]]) if c[0] == "set" else self.emit(c) for c in rest]
            absorb, neutral = (True, False) if k == "or" else (False, True)
            if any(x is absorb for x in parts):
                return absorb
            parts = [x for x in parts if x is not neutral]
            if not parts:
                return neutral
            if len(parts) == 1:
                return parts[0]
            j = " || " if k == "or" else " && "
            return j.join("(%s)" % x if ("||" in x or "&&" in x) else x for x in parts)
        if k == "not":
            x = self.emit(t[1])
            if isinstance(x, bool):
                return not x
            return "!" + x if _wrapped(x) else "!(%s)" % x
        if k == "cmp":
            a, b = t[2], t[3]
            if a[0] != "id" or b[0] != "num":
                raise SystemExit("condition non traduisible : %r" % (t,))
            return self.cmp(a[1], t[1], b[1])
        if k == "bit":
            a, b = t[1], t[2]
            if a[0] != "id" or b[0] != "num" or a[1] not in self.p:
                raise SystemExit("condition non traduisible : %r" % (t,))
            return "(%s & %d)" % (self.p[a[1]][0], int(b[1]))
        if k == "id":
            return self.cmp(t[1], "!=", 0)
        if k == "num":
            return t[1] != 0
        raise SystemExit("condition non traduisible : %r" % (t,))

    def of(self, expr):
        """None : condition de la carte seule (evaluee par le firmware) ; True : toujours visible ;
        sinon texte visibleIf (une condition toujours fausse devient FALSE_EXPR)."""
        t = _parse(expr)
        kinds = _kinds(t, set())
        if "board" in kinds:
            if "id" in kinds:
                raise SystemExit("condition melant carte et configuration : " + expr)
            return None
        x = self.emit(t)
        if x is False:
            return self.false
        return x


def _wrapped(x):
    """x entierement entre une paire de parentheses"""
    if not x.startswith("("):
        return False
    d = 0
    for i, ch in enumerate(x):
        d += ch == "("
        d -= ch == ")"
        if d == 0:
            return i == len(x) - 1
    return False


def _conjuncts(x):
    """termes de premier niveau de x separes par &&, sans parentheses englobantes"""
    out, d, cur, i = [], 0, "", 0
    while i < len(x):
        if d == 0 and x.startswith(" && ", i):
            out.append(cur)
            cur, i = "", i + 4
            continue
        d += x[i] == "("
        d -= x[i] == ")"
        cur += x[i]
        i += 1
    out.append(cur)
    return [c[1:-1] if _wrapped(c) else c for c in out]


def any_or(conds, limit=3):
    """Condition d'un conteneur : OU des conditions de ses elements (toutes conditionnelles, <= limit).
    Une condition de la forme 'c && ...' est absorbee par c."""
    if not conds or any(not isinstance(c, str) for c in conds):
        return None
    u = []
    for c in conds:
        if c not in u:
            u.append(c)
    u = [d for d in u if not any(c != d and c in _conjuncts(d) for c in u)]
    if len(u) > limit:
        return None
    return u[0] if len(u) == 1 else " || ".join("(%s)" % c for c in u)


SUFFIX = b"</Grouping>\n</ConfigParams>\n"


def flat(n):
    return re.sub(r"__(\d+)", r"_\1", n).replace(".", "_")


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def esc_attr(t):
    return esc(t).replace('"', "&quot;")


def cstr(t):
    return '"' + t.replace("\\", "\\\\").replace('"', '\\"') + '"'


def split(raw):
    """Partie fixe (jusqu'a <Grouping> exclu) ; verifie la fin du document."""
    i = raw.index(b"<Grouping>")
    if not raw.rstrip().endswith(SUFFIX.rstrip()):
        raise SystemExit("fin de XML inattendue")
    return raw[:i]


def c_lines(raw, root, key, vis, emap=None):
    """Tableaux C de la description 'key' (mc ou app) : partie fixe compressee, onglets, conditions,
    et conditions traduites de la variante complete (vide pour le profil VESC700)."""
    fixed = split(raw)
    co = zlib.compressobj(9, zlib.DEFLATED, 15)
    z = co.compress(fixed) + co.flush(zlib.Z_FULL_FLUSH)
    fields = {flat(s.text) for s in root.find("SerOrder")}
    conds, entries = [], []

    def cond_id(name):
        e = vis.get(name)
        if not e:
            return 0
        for ref in re.findall(r"\b(?:mc|app)->(\w+)", e):
            if ref not in fields:
                raise SystemExit("condition de %s : champ inconnu %s" % (name, ref))
        if e not in conds:
            conds.append(e)
        return conds.index(e) + 1

    for name in vis:
        if root.find("Params").find(name) is None:
            raise SystemExit("condition pour un parametre absent : " + name)
    # variante complete : visibleIf par entree (None : aucun, "board" : condition de la carte)
    tr = Vif(root, emap, key) if vis else None
    groups, pages = FULL[key] if vis else ({}, [])

    def full_of(name):
        e = vis.get(name)
        if not e:
            return None
        x = tr.of(e)
        return "board" if x is None else (None if x is True else x)

    tree = []
    for g in root.find("Grouping"):
        gname = g.findtext("groupName")
        subs = []
        for sg in g.findall("subgroup"):
            items = [(p.text, cond_id(p.text), full_of(p.text)) for p in sg.find("subgroupParams")]
            subs.append((sg.findtext("subgroupName"), items))
        tree.append((gname, subs))
    vifs = []

    def vif_id(x):
        if x is None:
            return 0
        if x == "board":
            return 0xFFFF
        if x not in vifs:
            vifs.append(x)
        return vifs.index(x) + 1

    for gname, subs in tree:
        sconds = []
        sub_entries = []
        for sname, items in subs:
            plain = [f for n, c, f in items if not n.startswith("::sep::")]
            sc = any_or([f for f in plain if f != "board"]) if all(f not in (None, "board") for f in plain) else None
            sconds.append(sc)
            ents = [("S", 0, vif_id(sc), esc(sname))]
            for i, (n, c, f) in enumerate(items):
                if n.startswith("::sep::"):
                    nxt = []
                    for n2, c2, f2 in items[i + 1:]:
                        if n2.startswith("::sep::"):
                            break
                        nxt.append(f2)
                    f = any_or(nxt) if nxt and all(x not in (None, "board") for x in nxt) else None
                ents.append(("P", c, vif_id(f), esc(n)))
            sub_entries += ents
        gc = tr.of(groups[gname]) if gname in groups else (any_or(sconds) if vis else None)
        if gc is True:
            gc = None
        entries.append(("G", 0, vif_id(gc), esc(gname)))
        entries += sub_entries
    entries.append(("E", 0, 0, ""))
    page_list = []
    for pname, pexpr in pages:
        x = tr.of(pexpr)
        if isinstance(x, str):
            page_list.append((pname, x))

    L = ["/* description %s : partie fixe %d octets -> %d, %d entrees d'onglets, %d conditions, %d visibleIf, %d pages */" %
         (key, len(fixed), len(z), len(entries), len(conds), len(vifs), len(page_list))]
    L.append("static const uint8_t xml_%s_z[] = {" % key)
    for i in range(0, len(z), 20):
        L.append("    " + ", ".join("0x%02X" % b for b in z[i:i + 20]) + ",")
    L.append("};")
    L.append("static const conf_grp_t xml_%s_grp[] = {" % key)
    for k, c, v, t in entries:
        L.append("    { '%s', %d, %du, %s }," % (k, c, v, cstr(t)))
    L.append("};")
    L.append("static const char *const xml_%s_vif[] = {" % key)
    L.append("    0,")
    for x in vifs:
        L.append("    %s," % cstr(esc_attr(x)))
    L.append("};")
    L.append("static const conf_page_t xml_%s_pages[] = {" % key)
    for pname, x in page_list:
        L.append("    { %s, %s }," % (cstr(pname), cstr(esc_attr(x))))
    L.append("    { 0, 0 },")
    L.append("};")
    L.append("static bool xml_%s_vis(uint16_t c, const vesc_mcconf_t *mc, const vesc_appconf_t *app)" % key)
    L.append("{")
    L.append("    (void)mc; (void)app;")
    if conds:
        L.append("    switch (c) {")
        for i, e in enumerate(conds):
            L.append("    case %d: return %s;" % (i + 1, e))
        L.append("    default: return true;")
        L.append("    }")
    else:
        L.append("    (void)c;")
        L.append("    return true;")
    L.append("}")
    L.append("const conf_desc_t conf_desc_%s = { xml_%s_z, %du, %du, 0x%08Xu, xml_%s_grp, xml_%s_vis, xml_%s_vif, "
             "xml_%s_pages };" % (key, key, len(z), len(fixed), zlib.adler32(fixed) & 0xFFFFFFFF, key, key, key, key))
    return L


def file_lines(header, descs):
    """descs : liste de (raw, root, key, vis). En-tete et includes communs."""
    L = [header, '#include "hw_config.h"', '#include "vesc_conf.h"', '#include "conf_desc.h"', ""]
    for raw, root, key, vis in descs:
        L += c_lines(raw, root, key, vis)
        L.append("")
    return L
