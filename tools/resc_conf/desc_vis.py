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


VIS_MC = {
    "m_encoder_counts": ENC,
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
    "foc_phase_filter_disable_fault": "mc->foc_phase_filter_enable",
}
for _k in range(8):
    VIS_MC["foc_hall_table__%d" % _k] = HALL

PPM = "app->app_ppm_conf_ctrl_type"      # 2, 5, 7, 11 sans marche arriere ; 6, 7 PID ; 8 hysteresis ; 9 intelligente
ADC = "app->app_adc_conf_ctrl_type"      # centrees : 2, 5, 6, 10, 13 ; frein ADC2 : 4, 8
VIS_APP = {
    "can_status_msgs_r1": "app->can_status_rate_1 > 0",
    "can_status_msgs_r2": "app->can_status_rate_2 > 0",
    "app_ppm_conf.pulse_center": "!" + any_of(PPM, (2, 5, 7, 11)),
    "app_ppm_conf.pid_max_erpm": any_of(PPM, (6, 7)),
    "app_ppm_conf.max_erpm_for_dir": any_of(PPM, (8, 9)),
    "app_ppm_conf.smart_rev_max_duty": "%s == 9" % PPM,
    "app_ppm_conf.smart_rev_ramp_time": "%s == 9" % PPM,
    "app_ppm_conf.tc": "app->app_ppm_conf_multi_esc",
    "app_ppm_conf.tc_max_diff": "(app->app_ppm_conf_multi_esc && app->app_ppm_conf_tc)",
    "app_adc_conf.voltage_center": any_of(ADC, (2, 5, 6, 10, 13)),
    "app_adc_conf.voltage2_start": any_of(ADC, (4, 8)),
    "app_adc_conf.voltage2_end": any_of(ADC, (4, 8)),
    "app_adc_conf.voltage2_inverted": any_of(ADC, (4, 8)),
    "app_adc_conf.tc": "app->app_adc_conf_multi_esc",
    "app_adc_conf.tc_max_diff": "(app->app_adc_conf_multi_esc && app->app_adc_conf_tc)",
    "app_chuk_conf.smart_rev_max_duty": "app->app_chuk_conf_use_smart_rev",
    "app_chuk_conf.smart_rev_ramp_time": "app->app_chuk_conf_use_smart_rev",
    "app_chuk_conf.coast_brake_ramp_time": "app->app_chuk_conf_coast_brake_level > 0.0f",
    "app_chuk_conf.tc": "app->app_chuk_conf_multi_esc",
    "app_chuk_conf.tc_max_diff": "(app->app_chuk_conf_multi_esc && app->app_chuk_conf_tc)",
}

SUFFIX = b"</Grouping>\n</ConfigParams>\n"


def flat(n):
    return re.sub(r"__(\d+)", r"_\1", n).replace(".", "_")


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def cstr(t):
    return '"' + t.replace("\\", "\\\\").replace('"', '\\"') + '"'


def split(raw):
    """Partie fixe (jusqu'a <Grouping> exclu) ; verifie la fin du document."""
    i = raw.index(b"<Grouping>")
    if not raw.rstrip().endswith(SUFFIX.rstrip()):
        raise SystemExit("fin de XML inattendue")
    return raw[:i]


def c_lines(raw, root, key, vis):
    """Tableaux C de la description 'key' (mc ou app) : partie fixe compressee, onglets, conditions."""
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
    for g in root.find("Grouping"):
        entries.append(("G", 0, esc(g.findtext("groupName"))))
        for sg in g.findall("subgroup"):
            entries.append(("S", 0, esc(sg.findtext("subgroupName"))))
            for p in sg.find("subgroupParams"):
                entries.append(("P", cond_id(p.text), esc(p.text)))
    entries.append(("E", 0, ""))

    L = ["/* description %s : partie fixe %d octets -> %d, %d entrees d'onglets, %d conditions */" %
         (key, len(fixed), len(z), len(entries), len(conds))]
    L.append("static const uint8_t xml_%s_z[] = {" % key)
    for i in range(0, len(z), 20):
        L.append("    " + ", ".join("0x%02X" % b for b in z[i:i + 20]) + ",")
    L.append("};")
    L.append("static const conf_grp_t xml_%s_grp[] = {" % key)
    for k, c, t in entries:
        L.append("    { '%s', %d, %s }," % (k, c, cstr(t)))
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
    L.append("const conf_desc_t conf_desc_%s = { xml_%s_z, %du, %du, 0x%08Xu, xml_%s_grp, xml_%s_vis };" % (
        key, key, len(z), len(fixed), zlib.adler32(fixed) & 0xFFFFFFFF, key, key))
    return L


def file_lines(header, descs):
    """descs : liste de (raw, root, key, vis). En-tete et includes communs."""
    L = [header, '#include "hw_config.h"', '#include "vesc_conf.h"', '#include "conf_desc.h"', ""]
    for raw, root, key, vis in descs:
        L += c_lines(raw, root, key, vis)
        L.append("")
    return L
