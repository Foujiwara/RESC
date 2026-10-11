#!/usr/bin/env python3
"""
Construit les descriptions de configuration du profil RESC a partir des XML VESC 7.00.

  entree : firmware/conf/vesc700/parameters_{mcconf,appconf}.xml   (officiels, non modifies)
  sortie : firmware/conf/resc_mcconf.xml, firmware/conf/resc_appconf.xml

Une fois generes, les XML RESC sont la source unique : gen_resc_conf.py en tire les
structures, la serialisation, la signature, les defauts et les tableaux de la commande 250.
Ce script documente et rejoue la derivation (retraits, renommages, ajouts) ; on peut aussi
editer les XML RESC directement et ne plus le relancer.

Regles du format (VESC Tool, ConfigParams) : un parametre retire l'est de <Params>,
<SerOrder> et <Grouping> ; un parametre ajoute est copie d'un parametre du meme type.
"""
import copy
import os
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC = os.path.join(ROOT, "firmware", "conf", "vesc700")
DST = os.path.join(ROOT, "firmware", "conf")

# ---------------------------------------------------------------- retraits
MC_REMOVE = [
    # commande BLDC / DC (RESC ne pilote qu'en FOC)
    "pwm_mode", "comm_mode", "sensor_mode", "sl_min_erpm", "sl_min_erpm_cycle_int_limit",
    "sl_max_fullbreak_current_dir_change", "sl_cycle_int_limit", "sl_phase_advance_at_br",
    "sl_cycle_int_rpm_br", "sl_bemf_coupling_k", "hall_sl_erpm", "cc_startup_boost_duty", "cc_gain",
    "cc_ramp_step_max", "m_duty_ramp_step", "m_current_backoff_gain", "m_bldc_f_sw_min", "m_bldc_f_sw_max",
    "m_dc_f_sw", "l_max_erpm_fbrake", "l_max_erpm_fbrake_cc",
    # entrees et capteurs absents ou non pris en charge
    "m_encoder_sin_amp", "m_encoder_cos_amp", "m_encoder_sin_offset", "m_encoder_cos_offset",
    "m_encoder_sincos_filter_constant", "m_encoder_sincos_phase_correction", "m_sensor_port_mode",
    "m_drv8301_oc_mode", "m_drv8301_oc_adj",
    # echantillonnage et modulation fixes dans RESC
    "foc_control_sample_mode", "foc_current_sample_mode", "foc_current_filter_const", "foc_overmod_factor",
    "foc_mag_vd_max", "foc_short_ls_on_zero_duty", "foc_fw_backoff", "foc_hfi_reset_erpm",
    "foc_hfi_obs_ovr_sec", "l_in_current_map_start", "l_in_current_map_filter", "p_pid_kd_proc",
] + ["hall_table__%d" % k for k in range(8)] + [
    "bms.type", "bms.limit_mode", "bms.t_limit_start", "bms.t_limit_end", "bms.soc_limit_start",
    "bms.soc_limit_end", "bms.vmin_limit_start", "bms.vmin_limit_end", "bms.vmax_limit_start",
    "bms.vmax_limit_end", "bms.fwd_can_mode",
]
APP_REMOVE = [
    "pairing_done", "permanent_uart_enabled", "can_mode", "uavcan_esc_index", "uavcan_raw_mode",
    "uavcan_raw_rpm_max", "uavcan_status_current_mode", "servo_out_enable", "app_pas_conf.sensor_type",
] + [p for p in (
    "app_nrf_conf.speed app_nrf_conf.power app_nrf_conf.crc_type app_nrf_conf.retry_delay app_nrf_conf.retries "
    "app_nrf_conf.channel app_nrf_conf.address__0 app_nrf_conf.address__1 app_nrf_conf.address__2 "
    "app_nrf_conf.send_crc_ack imu_conf.type imu_conf.mode imu_conf.filter imu_conf.accel_lowpass_filter_x "
    "imu_conf.accel_lowpass_filter_y imu_conf.accel_lowpass_filter_z imu_conf.gyro_lowpass_filter "
    "imu_conf.sample_rate_hz imu_conf.use_magnetometer imu_conf.accel_confidence_decay imu_conf.mahony_kp "
    "imu_conf.mahony_ki imu_conf.madgwick_beta imu_conf.rot_roll imu_conf.rot_pitch imu_conf.rot_yaw "
    "imu_conf.accel_offsets__0 imu_conf.accel_offsets__1 imu_conf.accel_offsets__2 imu_conf.gyro_offsets__0 "
    "imu_conf.gyro_offsets__1 imu_conf.gyro_offsets__2").split()]


def html(*paras):
    return "".join("<p>%s</p>" % p for p in paras)


# --------------------------------------------------- textes et choix propres a RESC
MC_EDIT = {
    "foc_observer_type": dict(
        enums=["Active flux, non-linear correction", "Active flux, radial correction (gain 2|w|)",
               "Active flux non-linear + lambda adaptation", "Active flux radial + lambda adaptation",
               "Back-EMF observer (Luenberger)", "Back-EMF observer + lambda adaptation",
               "Hybrid flux / back-EMF + lambda adaptation"],
        valInt="0",
        description=html(
            "Sensorless position observer (all written for RESC).",
            "<b>Active flux, non-linear</b>: voltage model on the active flux with a gradient correction "
            "of its magnitude (Lee, Ortega et al., written on Boldea's active flux). Works for surface and "
            "interior magnet motors. Recommended.",
            "<b>Radial correction</b>: same model, linear pull of the magnitude with a gain of 2|w|; no gain to tune.",
            "<b>Back-EMF observer</b>: Luenberger current observer, back-EMF integrated (no current derivative), "
            "bandwidth following the speed; angle independent of lambda; for medium and high speed.",
            "<b>Hybrid</b>: active flux at low speed, back-EMF at high speed, linear blend.",
            "<b>Lambda adaptation</b>: flux linkage re-estimated while running from |e|/|w| (time constant 1 s, "
            "limited to +-30 % of the configured value).")),
    "foc_sensor_mode": dict(
        enums=["Sensorless", "Encoder", "Hall Sensors", "HFI", "VSS (HFI start, I/f, observer)"],
        description=html(
            "Angle source. <b>Encoder</b>: ABI encoder (index not used). <b>HFI</b>: high frequency injection "
            "(+ + - - square wave on the estimated d axis, demodulated on q; needs Lq &gt; Ld), observer above "
            "Sensorless ERPM HFI. <b>VSS</b>: rotor position found by injection at standstill (with polarity), "
            "then I/f without alignment and hand-over to the observer.")),
    "foc_observer_offset": dict(valDouble="0", minDouble="-3", maxDouble="3", description=html(
        "Extra angle advance of the observer output, in PWM periods (0 = none). RESC already compensates "
        "the sampling and output delays.")),
    "foc_observer_gain_slow": dict(description=html(
        "Fraction of the observer gain kept at standstill. The gain rises linearly to 100 % at 50 % "
        "voltage level (max of applied modulation and back-EMF / (Vbus/sqrt3)).")),
    "foc_cc_decoupling": dict(valInt="3", description=html(
        "Current loop feed-forward: cross-coupling terms (w L i) and back-EMF (w lambda). The integrators are "
        "pre-loaded with the model voltage at each hand-over, whatever the choice. Recommended: Cross and BEMF.")),
    "foc_openloop_rpm_low": dict(description=html(
        "Hand-over speed at zero current, as a fraction of Openloop ERPM (0 = fixed speed). The hand-over "
        "speed rises linearly with the commanded current up to Openloop ERPM at the motor current limit.")),
    "foc_sl_openloop_time": dict(description=html(
        "Minimum time at hand-over speed with the observer locked before switching to it (min 50 ms).")),
    "foc_sl_openloop_hyst": dict(description=html(
        "Time below the exit speed (half of the hand-over speed) before going back to I/f.")),
    "foc_hfi_amb_mode": dict(description=html(
        "Polarity test (north/south) at HFI start: current pulses on the d axis; saturation makes the "
        "response stronger towards the magnet. <b>Single</b>: +I compared with no bias. "
        "<b>Double</b> and <b>Six vectors</b>: +I compared with -I.")),
    "foc_hfi_amb_tres": dict(description=html("Duration of each polarity pulse (ms).")),
    "foc_hfi_gain": dict(description=html("HFI angle tracking: PLL bandwidth = 2000 x gain rad/s.")),
    "foc_hfi_samples": dict(description=html(
        "Demodulations averaged for the convergence test at start and for the displayed error.")),
    "foc_hfi_start_samples": dict(description=html(
        "Minimum number of averaged blocks before the polarity test (the angle must also be stable).")),
    "foc_phase_filter_enable": dict(description=html(
        "Switch on the RC filter of the phase voltage inputs below Phase Filter Max ERPM and feed the observer "
        "with the measured voltages (free of dead-time error). Only on boards with a phase filter.")),
    "foc_duty_dowmramp_kp": dict(description=html(
        "Duty mode: when the duty request decreases, a PI from duty error to braking current brings the duty "
        "down within the current limits. Proportional gain (A per unit of duty).")),
    "foc_duty_dowmramp_ki": dict(description=html("Duty mode, decreasing duty: integral gain (A/s per unit of duty).")),
    "foc_offsets_cal_mode": dict(description=html(
        "Bit 0 set: offsets measured at every boot. Bit 0 clear: the offsets of the manual calibration "
        "(offsets_cal) are used when they agree with the boot measurement.")),
    "l_slow_abs_current": dict(description=html(
        "Absolute current limit on the filtered current magnitude instead of each sample (the hardware "
        "analog watchdog is then set 50 % higher).")),
}
APP_EDIT = {
    "shutdown_mode": dict(description=html(
        "Power button. <b>ALWAYS_OFF</b>: the board turns off 0.5 s after the button is released (use only with "
        "a latching switch). <b>ALWAYS_ON</b>: never turned off by the firmware. <b>TOGGLE_BUTTON_ONLY</b>: "
        "long press (2 s). <b>OFF_AFTER_x</b>: long press, or no activity (commands, inputs, motion) during x. "
        "The motor must be stopped.")),
    "app_ppm_conf.throttle_exp_mode": dict(description=html(
        "Throttle curve (RESC): a = |input|, k = throttle exponent. Exponential: (e^(k a)-1)/(e^k-1). "
        "Natural: a^(1+k) for k &gt;= 0, 1-(1-a)^(1-k) for k &lt; 0. Polynomial: a + k (a^3 - a), k in [-0.5, 1]. "
        "k &gt; 0 softens the start.")),
}
for p in ("app_adc_conf.throttle_exp_mode", "app_chuk_conf.throttle_exp_mode"):
    APP_EDIT[p] = APP_EDIT["app_ppm_conf.throttle_exp_mode"]

# ------------------------------------------------------------ ajouts RESC
# (nom, modele a copier, champs, groupe, sous-groupe, separateur)
MC_ADD = [
    ("foc_motor_kind", "foc_sat_comp_mode", dict(
        longName="Motor Kind", cDefine="MCCONF_FOC_MOTOR_KIND", valInt="0",
        enums=["Permanent magnet (PMSM / BLDC)", "Induction (ACIM)"],
        description=html("Permanent magnet motor, or induction motor (no magnet): rotor flux oriented control "
                         "with slip from the rotor time constant. For induction motors set the parameters of the "
                         "Induction Motor section (detect_acim in the terminal).")),
     "FOC", "General", "Motor Kind"),
    ("foc_acim_lm", "foc_motor_l", dict(
        longName="Magnetizing Inductance", cDefine="MCCONF_FOC_ACIM_LM", valDouble="0", maxDouble="1",
        description=html("Induction motor: magnetizing inductance Lm (equivalent circuit, per phase).")),
     "FOC", "General", "Induction Motor"),
    ("foc_acim_lr", "foc_motor_l", dict(
        longName="Rotor Inductance", cDefine="MCCONF_FOC_ACIM_LR", valDouble="0", maxDouble="1",
        description=html("Induction motor: rotor inductance Lr = Lm + rotor leakage. Foc Motor L is the stator "
                         "inductance Ls = Lm + stator leakage; the current loop uses sigma Ls = Ls - Lm^2/Lr.")),
     "FOC", "General", None),
    ("foc_acim_rr", "foc_motor_r", dict(
        longName="Rotor Resistance", cDefine="MCCONF_FOC_ACIM_RR", valDouble="0",
        description=html("Induction motor: rotor resistance referred to the stator. Rotor time constant Lr/Rr.")),
     "FOC", "General", None),
    ("foc_acim_im", "foc_sl_openloop_boost_q", dict(
        longName="Magnetizing Current", cDefine="MCCONF_FOC_ACIM_IM", valDouble="0",
        description=html("Induction motor: d axis (flux) current at base speed, peak phase amps. "
                         "Typically the no-load current of the motor.")),
     "FOC", "General", None),
    ("foc_acim_base_erpm", "foc_openloop_rpm", dict(
        longName="Base ERPM", cDefine="MCCONF_FOC_ACIM_BASE_ERPM", valDouble="0",
        description=html("Induction motor: above this electrical speed the flux current is reduced as "
                         "base / speed (field weakening). 0 = no field weakening.")),
     "FOC", "General", None),
    ("foc_resc_cc_bw", "foc_openloop_rpm", dict(
        longName="Current Loop Bandwidth", cDefine="MCCONF_FOC_RESC_CC_BW", valDouble="1000", maxDouble="10000",
        suffix=" rad/s",
        description=html("Bandwidth used to compute kp = L x bw and ki = R x bw after the R/L detection "
                         "(and by foc_gains in the terminal).")),
     "FOC", "Advanced", "RESC"),
    ("foc_resc_rl_frac", "foc_fw_q_current_factor", dict(
        longName="R/L Detection Current", cDefine="MCCONF_FOC_RESC_RL_FRAC", valDouble="0.5",
        description=html("Highest current step of the resistance measurement, as a fraction of Motor Current "
                         "Max. Steps at 1/2, 3/4 and 1 of it; R is the slope dV/dI.")),
     "FOC", "Advanced", None),
]


def ser_names(root):
    return [s.text for s in root.find("SerOrder")]


def remove(root, names):
    params, ser = root.find("Params"), root.find("SerOrder")
    for n in names:
        e = params.find(n)
        if e is None:
            raise SystemExit("parametre absent : " + n)
        params.remove(e)
        for s in list(ser):
            if s.text == n:
                ser.remove(s)
    for sub in root.iter("subgroupParams"):
        kept = [p for p in list(sub) if p.text not in names]
        # separateurs devenus vides (suivis d'un autre separateur ou de la fin)
        for p in list(sub):
            sub.remove(p)
        out = []
        for i, p in enumerate(kept):
            if p.text.startswith("::sep::"):
                nxt = kept[i + 1] if i + 1 < len(kept) else None
                if nxt is None or nxt.text.startswith("::sep::"):
                    continue
            out.append(p)
        for p in out:
            sub.append(p)


def set_text(e, tag, val):
    x = e.find(tag)
    if x is None:
        x = ET.SubElement(e, tag)
    x.text = val


def edit(root, edits):
    params = root.find("Params")
    for name, ch in edits.items():
        e = params.find(name)
        if e is None:
            raise SystemExit("parametre absent : " + name)
        for k, v in ch.items():
            if k == "enums":
                for old in e.findall("enumNames"):
                    e.remove(old)
                for t in v:
                    ET.SubElement(e, "enumNames").text = t
            else:
                set_text(e, k, v)


def find_subgroup(root, group, sub):
    for g in root.find("Grouping"):
        if g.findtext("groupName", "").lower() != group.lower():
            continue
        for sg in g.findall("subgroup"):
            if sg.findtext("subgroupName", "").lower() == sub.lower():
                return sg.find("subgroupParams")
    raise SystemExit("onglet absent : %s / %s" % (group, sub))


def add(root, adds):
    params, ser = root.find("Params"), root.find("SerOrder")
    for name, tpl, ch, group, sub, sep in adds:
        e = copy.deepcopy(params.find(tpl))
        e.tag = name
        params.append(e)
        edit(root, {name: ch})
        ET.SubElement(ser, "ser").text = name
        sp = find_subgroup(root, group, sub)
        if sep:
            ET.SubElement(sp, "param").text = "::sep::" + sep
        ET.SubElement(sp, "param").text = name


def write(root, path):
    ET.indent(root, space="    ")
    data = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    with open(path, "wb") as fh:
        fh.write(data + b"\n")


def main():
    mc = ET.parse(os.path.join(SRC, "parameters_mcconf.xml")).getroot()
    app = ET.parse(os.path.join(SRC, "parameters_appconf.xml")).getroot()
    remove(mc, MC_REMOVE)
    edit(mc, MC_EDIT)
    add(mc, MC_ADD)
    remove(app, APP_REMOVE)
    edit(app, APP_EDIT)
    write(mc, os.path.join(DST, "resc_mcconf.xml"))
    write(app, os.path.join(DST, "resc_appconf.xml"))
    print("mcconf : %d parametres, appconf : %d" % (len(ser_names(mc)), len(ser_names(app))))


if __name__ == "__main__":
    sys.exit(main())
