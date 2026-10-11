#!/usr/bin/env python3
"""
Construit les descriptions de configuration du profil RESC a partir des XML VESC 7.00.

  entree : firmware/conf/vesc700/parameters_{mcconf,appconf}.xml   (officiels, non modifies)
  sortie : firmware/conf/resc_mcconf.xml, firmware/conf/resc_appconf.xml

Il ecrit aussi la vue RESC Tool du profil VESC 7.00 (firmware/conf/vesc700_view/) : memes
<Params>/<SerOrder>/choix que l'officiel (signature et octets identiques), mais les parametres
inutilises sont retires des onglets (invisibles) et les aides decrivent le comportement de RESC.
C'est la description envoyee par la commande 250 en profil VESC700 ; VESC Tool officiel, qui
ne la demande pas, garde sa propre description.

Une fois generes, les XML RESC sont la source unique : gen_resc_conf.py en tire les
structures, la serialisation, la signature, les defauts et les tableaux de la commande 250.
Ce script documente et rejoue la derivation (retraits, renommages, ajouts) ; on peut aussi
editer les XML RESC directement et ne plus le relancer.

Regles du format (VESC Tool, ConfigParams) : un parametre retire l'est de <Params>,
<SerOrder> et <Grouping> ; un parametre ajoute est copie d'un parametre du meme type.
"""
import copy
import json
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
    "m_drv8301_oc_mode", "m_drv8301_oc_adj",
    # limite lente du courant absolu (non reprise : la limite reste echantillon par echantillon)
    "l_slow_abs_current",
] + ["hall_table__%d" % k for k in range(8)] + [
]
APP_REMOVE = [
    "pairing_done", "uavcan_esc_index", "uavcan_raw_mode",
    "uavcan_raw_rpm_max", "uavcan_status_current_mode", "app_pas_conf.sensor_type",
] + [p for p in (
    "app_nrf_conf.speed app_nrf_conf.power app_nrf_conf.crc_type app_nrf_conf.retry_delay app_nrf_conf.retries "
    "app_nrf_conf.channel app_nrf_conf.address__0 app_nrf_conf.address__1 app_nrf_conf.address__2 "
    "app_nrf_conf.send_crc_ack imu_conf.use_magnetometer").split()]


def html(*paras):
    return "".join("<p>%s</p>" % p for p in paras)


# --------------------------------------------------- textes et choix propres a RESC
MC_EDIT = {
    "motor_type": dict(description=html(
        "RESC drives motors in FOC only. Transmitted for RESC Tool (FOC page); the visible Motor Type "
        "(foc_motor_kind) chooses between permanent magnet and induction motors.")),
    "l_additional_faults": dict(description=html(
        "<b>Encoder Slip</b>: fault when the encoder speed differs from the observer speed by more than 30 % "
        "for 0.1 s (above twice the I/f exit speed). <b>Overspeed</b>: above 110 % of the ERPM limit. "
        "<b>Absolute Overspeed</b>: above 130 %.")),
    "sp_pid_loop_rate": dict(description=html(
        "Rate of the speed and position PID. The RESC control layer runs at 1 kHz at most.")),
    "foc_observer_type": dict(
        enums=["ORTEGA_ORIGINAL", "MXLEMMING", "ORTEGA_LAMBDA_COMP", "MXLEMMING_LAMBDA_COMP", "MXV",
               "MXV_LAMBDA_COMP", "MXV_LAMBDA_COMP_LIN", "RESC_ACTIVE_FLUX", "RESC_RADIAL",
               "RESC_ACTIVE_FLUX_LAMBDA", "RESC_RADIAL_LAMBDA", "RESC_BEMF", "RESC_BEMF_LAMBDA",
               "RESC_HYBRID_LAMBDA"],
        valInt="7", maxInt="13",
        description=html(
            "Sensorless position observer. 0 to 6: the VESC 7.00 observers (same equations, coefficients and "
            "limits). 7 to 13: the RESC observers.",
            "<b>ORTEGA_ORIGINAL</b>: non-linear flux observer (Lee, Ortega et al. 2010) on the stator flux.",
            "<b>MXLEMMING</b>: magnet flux integrator, components limited to lambda (David Molony, MESC).",
            "<b>MXV</b>: flux integrator, magnitude scaled back to lambda when exceeded.",
            "<b>..._LAMBDA_COMP</b>: same with the flux linkage estimated while running (0.3 to 2.5 x lambda); "
            "<b>MXV_LAMBDA_COMP_LIN</b>: estimate by low-pass filter of the flux magnitude.",
            "<b>RESC_ACTIVE_FLUX</b>: active flux observer (Boldea) with the non-linear correction, magnitude "
            "lambda + (Ld - Lq) id: for surface and interior magnet motors. Default, tested on the bench.",
            "<b>RESC_RADIAL</b>: same active flux, linear pull of the magnitude with a gain of 2|w| (no gain to tune).",
            "<b>RESC_BEMF</b>: back-EMF observer (Luenberger on the current, no current derivative, bandwidth "
            "following the speed, delay compensated); angle independent of lambda; medium and high speed.",
            "<b>RESC_HYBRID</b>: active flux at low speed, back-EMF at high speed, linear blend.",
            "<b>RESC ..._LAMBDA</b>: flux linkage re-estimated while running from |e|/|w| (1 s, +-30 %).")),
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
        "<b>Double</b>: +I compared with -I.")),
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
    "foc_control_sample_mode": dict(description=html(
        "<b>V0 Only</b>: one current sample and one control update per PWM period (zero vector V0, all "
        "low-side FETs on).",
        "<b>V0 and V7 Interpolation</b>: same sampling; at the opposite end of the period (V7) the modulation "
        "is updated again with the angle advanced by half a period. Smoother at high electrical speed.",
        "<b>V0 and V7</b>: sampling and full control at both ends of the period (control loop at twice the "
        "PWM frequency). Only on boards with in-line or phase shunts (not shown otherwise).")),
    "foc_current_sample_mode": dict(description=html(
        "<b>Longest Zero Time</b>: the phase with the shortest low-side window is rebuilt from the two others "
        "when its window approaches the minimum. <b>All Sensors Combined</b>: every valid phase is used. "
        "<b>High Current</b>: the phase with the highest current is rebuilt from the two others (more range "
        "from the current amplifiers). Phases without a valid window are always rebuilt.")),
    "foc_current_filter_const": dict(description=html(
        "Low-pass filter of the d/q currents used for display, limits and power (per PWM period, 1 = none). "
        "The current loop always uses the unfiltered samples.")),
    "foc_overmod_factor": dict(description=html(
        "Radius of the voltage circle: linear limit x this factor. Above 1 the phase duty cycles are clipped "
        "(overmodulation): more voltage at top speed, more current ripple and harmonics, and shorter low-side "
        "windows (more rebuilt current samples).")),
    "foc_mag_vd_max": dict(description=html(
        "Part of the voltage circle the d axis may use. The d axis has priority (field weakening, stability); "
        "q gets the rest.")),
    "foc_short_ls_on_zero_duty": dict(description=html(
        "When the voltage is zero (braking at low speed, zero duty), keep all low-side FETs on instead of a "
        "modulated short: less losses, no dead-time error and more braking torque at low speed, down to "
        "standstill.")),
    "foc_fw_backoff": dict(description=html(
        "Field weakening backoff: when iq exceeds its target (not enough voltage left on q), the maximum "
        "field weakening current is reduced by gain x (iq - iq target) / FW current max. 0 = off.")),
    "foc_hfi_reset_erpm": dict(description=html(
        "HFI and VSS: when the motor is restarted while still turning above this speed, it restarts from "
        "the observer angle; below, the position and polarity search is done again.")),
    "foc_hfi_obs_ovr_sec": dict(description=html(
        "HFI: after slowing down below Sensorless ERPM HFI, the injection angle follows the observer "
        "during this time (the observer is still good just below the switch speed).")),
    "l_in_current_map_start": dict(description=html(
        "Above this fraction of Battery Current Max, the motor current limit is reduced linearly to zero at "
        "Battery Current Max (smoother than the input current regulation). 1 = off.")),
    "l_in_current_map_filter": dict(description=html(
        "Low-pass filter of the input current used by the map (per PWM period).")),
    "p_pid_kd_proc": dict(description=html(
        "Derivative on the measured position (damping without kick when the setpoint changes).")),
    "bms.type": dict(description=html(
        "VESC BMS on the CAN bus: battery current limited by cell temperature (charge and discharge), state of "
        "charge and lowest cell (discharge), highest cell (regeneration). The values are taken from the hottest, "
        "least charged, lowest and highest BMS seen in the last 2 s. BMS page of the tool and BMS commands "
        "relayed to it.")),
    "m_sensor_port_mode": dict(description=html(
        "Encoder type used when the angle source (FOC, General) is Encoder.",
        "<b>Hall Sensors</b>: the sensor port carries Hall sensors (angle source Hall Sensors); no encoder. "
        "<b>ABI</b>: incremental A/B on the Hall port (H1, H2), Encoder Counts per turn. "
        "<b>PWM + ABI</b>: ABI counts, re-synchronised on the absolute PWM position (servo input). "
        "<b>TS5700N8501</b>: Tamagawa RS485 at 2.5 Mbit/s: RX of the COMM UART, TX driven by software, "
        "transceiver direction on the ADC1 pin (the COMM UART is then reserved). "
        "<b>BISSC</b>: BiSS-C 22 bits on SPI1 (COMM port). <b>TLE5012 SSC HW</b>: SSC on the COMM port pins "
        "(SCK, data on MOSI, CS on NSS). "
        "<b>AS5047</b>, <b>AMT22</b>: SPI on the Hall port (H1 SCK, H2 MISO, H3 CS). "
        "<b>TLE5012</b>: SSC on the Hall port (H1 SCK, H2 data, H3 CS). "
        "<b>MT6816</b>, <b>AS5X47U</b>, <b>MA782</b>: SPI1 on the COMM port (SCK, MISO, MOSI, NSS = CS). "
        "<b>AD2S1205</b>: resolver converter, SPI on the COMM port pins. "
        "<b>Sin/Cos</b>: analog sine on ADC1 and cosine on ADC2. "
        "<b>PWM</b>: PWM output of the encoder on the servo input.",
        "Absolute encoders are read every PWM period; an invalid sample (parity, CRC, magnet, signal "
        "level) keeps the previous angle. Then run the encoder detection (ratio, direction, offset).")),
    "m_encoder_sincos_phase_correction": dict(description=html(
        "Sin/Cos encoder: phase error between the two signals (degrees).")),
}
APP_EDIT = {
    "imu_conf.type": dict(description=html(
        "<b>Internal</b>: BMI160 of the board (software I2C). <b>External BMI160</b>: on the UART port pins "
        "(SCL = TX, SDA = RX), only when no UART application uses them.",
        "Data in the IMU page of the tool (roll, pitch, yaw, accelerations, rates, quaternion); calibration "
        "from the tool (gyro offsets, roll and pitch of the mounting).")),
    "imu_conf.filter": dict(description=html(
        "Internal filter of the BMI160. <b>Low</b>: average of 4 samples. <b>Medium</b>: oversampling x2 at "
        "twice the rate. <b>High</b>: oversampling x4 at four times the rate.")),
    "imu_conf.mode": dict(description=html(
        "Attitude filter. <b>Madgwick</b>: gradient descent, gain Beta. <b>Mahony</b>: PI on the gravity "
        "error, Kp and Ki. <b>Madgwick Fusion</b>: proportional gravity feedback, gain Beta. The accelerometer "
        "weight decreases when the acceleration differs from 1 g (Accelerometer Confidence Decay).")),
    "app_to_use": dict(description=html(
        "Control application. <b>UART</b> also accepts the VESC Remote through packets (radio module on the "
        "permanent UART, BLE). <b>ADC</b> alone uses the UART TX/RX pins for the cruise and reverse buttons; "
        "<b>PAS</b> uses them for the pedal sensor. Servo input on the PPM pin (TIM4).")),
    "can_baud_rate": dict(description=html("CAN bus speed (all rates supported).")),
    "can_mode": dict(description=html(
        "<b>VESC</b>: VESC CAN protocol (status messages, commands, packet forwarding, BMS). "
        "<b>COMM Bridge</b>: every received frame is forwarded to the tool (COMM_CAN_FWD_FRAME), no VESC "
        "processing. <b>Unused</b>: the CAN bus is ignored.")),
    "servo_out_enable": dict(description=html(
        "Servo output on the servo pin (50 Hz, 1 to 2 ms) when no PPM application uses it; position "
        "set by COMM_SET_SERVO_POS.")),
    "permanent_uart_enabled": dict(description=html(
        "Packets accepted on the permanent UART (Bluetooth / NRF module of the board).")),
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

# --------------------------------------------- choix conserves (les autres sont retires)
# Pour chaque liste : (valeur interne, nom affiche) dans l'ordre affiche. Pour les champs de bits,
# la valeur interne est le numero du bit. Le firmware traduit position <-> valeur interne
# (firmware/conf/resc_enum_map.json, lu par gen_resc_conf.py et board_xml.py).
# motor_type et app_to_use restent complets : RESC Tool choisit les pages affichees d'apres leur
# numero (FOC = 2, PPM = 1, ADC = 2, PAS = 9...) ; le firmware ramene les choix sans effet sur un
# choix valide (FOC ; Nunchuk I2C, NRF, app personnalisee -> UART).
ENUM_KEEP = {
    "l_additional_faults": [(0, "Encoder Slip"), (1, "Overspeed"), (3, "Absolute Overspeed")],
    "sp_pid_loop_rate": [(0, "25 Hz"), (1, "50 Hz"), (2, "100 Hz"), (3, "250 Hz"), (4, "500 Hz"), (5, "1000 Hz")],
    "foc_hfi_amb_mode": [(1, "Id Single Pulse"), (2, "Id Double Pulse")],
    "foc_offsets_cal_mode": [(0, "Calibrate on Boot")],
    "bms.limit_mode": [(0, "Overtemp"), (1, "SOC"), (2, "VCell Min"), (3, "VCell Max")],
    "m_sensor_port_mode": [(0, "Hall Sensors"), (1, "ABI Encoder"), (2, "AS5047 Encoder"), (3, "AD2S1205 Resolver"),
                           (4, "Sin/Cos Encoder"), (5, "TS5700N8501 Encoder"), (6, "TS5700N8501 Encoder (Multiturn)"),
                           (7, "MT6816 Encoder (SPI)"), (8, "AS5X47U Encoder (SPI)"), (9, "BISSC Encoder (SPI)"),
                           (10, "TLE5012 Encoder (SSC SW)"), (11, "TLE5012 Encoder (SSC HW)"), (13, "PWM"),
                           (14, "PWM + ABI"), (15, "MA782"), (16, "AMT22 (SW SPI)")],
    "foc_control_sample_mode": [(0, "V0 Only"), (2, "V0 and V7 Interpolation"), (1, "V0 and V7")],
    "foc_current_sample_mode": [(0, "Longest Zero Time"), (1, "All Sensors Combined"), (2, "High Current")],
    "m_out_aux_mode": [(0, "Off"), (1, "On after 2 seconds"), (2, "On after 5 seconds"), (3, "On after 10 seconds"),
                       (5, "On when running"), (6, "On when not running"), (7, "Temp motor > 50 C"),
                       (8, "Temp mosfet > 50 C"), (9, "Temp motor > 70 C"), (10, "Temp mosfet > 70 C"),
                       (11, "Temp motor or mosfet > 50 C"), (12, "Temp motor or mosfet > 70 C")],
}
APP_ENUM_KEEP = {
    "can_mode": [(0, "VESC"), (2, "COMM Bridge"), (3, "Unused")],
    "imu_conf.type": [(0, "Off"), (1, "Internal (BMI160)"), (4, "External BMI160 (UART port, I2C)")],
    "imu_conf.mode": [(0, "Madgwick"), (1, "Mahony"), (2, "Madgwick Fusion")],
    "imu_conf.filter": [(0, "Low"), (1, "Medium"), (2, "High")],
    "can_status_msgs_r1": [(k, "Status %d" % (k + 1)) for k in range(6)],
    "can_status_msgs_r2": [(k, "Status %d" % (k + 1)) for k in range(6)],
    "app_adc_conf.buttons": [(0, "Enable Cruise Control"), (1, "Invert CC Button"), (2, "Invert Reverse Button")],
}


def apply_keep(root, keep, emap):
    params = root.find("Params")
    for name, items in keep.items():
        e = params.find(name)
        bits = e.findtext("type") == "6"
        old_def = int(e.findtext("valInt") or "0")
        for x in e.findall("enumNames"):
            e.remove(x)
        for _, label in items:
            ET.SubElement(e, "enumNames").text = label
        vals = [v for v, _ in items]
        if bits:
            new_def = sum(1 << j for j, v in enumerate(vals) if old_def & (1 << v))
        else:
            new_def = vals.index(old_def) if old_def in vals else 0
        set_text(e, "valInt", str(new_def))
        if not bits:
            set_text(e, "maxInt", str(len(vals) - 1))
        emap[flat_name(name)] = {"bits": bits, "keep": vals}


def flat_name(n):
    import re as _re
    return _re.sub(r"__(\d+)", r"_\1", n).replace(".", "_")


# ------------------------------------------------------------ ajouts RESC
# (nom, modele a copier, champs, groupe, sous-groupe, separateur)
MC_ADD = [
    ("foc_motor_kind", "foc_sat_comp_mode", dict(
        longName="Motor Type", cDefine="MCCONF_FOC_MOTOR_KIND", valInt="0",
        enums=["FOC Permanent Magnets (PMSM / BLDC / IPM)", "FOC Induction (ACIM)"],
        description=html("<b>FOC Permanent Magnets</b>: surface (PMSM, BLDC) or interior (IPM) magnet motor.",
                         "<b>FOC Induction</b>: induction motor without magnet, rotor flux oriented control with "
                         "slip from the rotor time constant. Set the Induction Motor section (FOC, General): run "
                         "the R/L detection, then detect_acim in the terminal (DC current step, the motor does "
                         "not turn), then set the magnetizing current. Angle source: encoder (indirect FOC) or "
                         "sensorless (magnetization, frequency ramp, observer).",
                         "Only the parameters used by the selected motor type and options are shown. After "
                         "changing it, write the configuration and reconnect to update the list.")),
     "General", "General", None),
    ("foc_acim_lm", "foc_motor_l", dict(
        longName="Magnetizing Inductance", cDefine="MCCONF_FOC_ACIM_LM", valDouble="0", maxDouble="1",
        description=html("Induction motor: magnetizing inductance Lm (equivalent circuit, per phase).")),
     "FOC", "General", "Induction Motor"),
    ("foc_acim_lr", "foc_motor_l", dict(
        longName="Rotor Inductance", cDefine="MCCONF_FOC_ACIM_LR", valDouble="0", maxDouble="1",
        description=html("Induction motor: rotor inductance Lr = Lm + rotor leakage. For an induction motor, "
                         "Motor Inductance (L) is the transient inductance sigma Ls measured by the R/L detection "
                         "(used by the current loop).")),
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

# App ADC : marche arriere intelligente et commande hybride courant / rapport cyclique
_SR = ("Applies to the reverse current control types (Reverse Center, Reverse Button, Reverse ADC2 Brake "
       "Button, Reverse Button Brake Center).")
APP_ADD = [
    ("app_adc_conf.resc_smart_rev", "app_adc_conf.use_filter", dict(
        longName="Smart Reverse", cDefine="APPCONF_ADC_RESC_SMART_REV", valInt="0",
        description=html("Reverse request while rolling forward: brake (current proportional to the request). "
                         "Below Smart Reverse Brake Above ERPM: reverse at a limited duty cycle, reached with "
                         "a ramp, like the PPM Smart Reverse.", _SR)),
     "ADC", "General", "Smart Reverse"),
    ("app_adc_conf.resc_smart_rev_erpm", "app_ppm_conf.max_erpm_for_dir", dict(
        longName="Smart Reverse Brake Above ERPM", cDefine="APPCONF_ADC_RESC_SMART_REV_ERPM", valDouble="1000",
        description=html("Forward speed above which a reverse request brakes. Below it, the motor reverses.")),
     "ADC", "General", None),
    ("app_adc_conf.resc_smart_rev_max_duty", "app_ppm_conf.smart_rev_max_duty", dict(
        longName="Smart Reverse Max Duty Cycle", cDefine="APPCONF_ADC_RESC_SMART_REV_MAX_DUTY", valDouble="0.07",
        description=html("Duty cycle in reverse at full request.")),
     "ADC", "General", None),
    ("app_adc_conf.resc_smart_rev_ramp_time", "app_ppm_conf.smart_rev_ramp_time", dict(
        longName="Smart Reverse Ramp Time", cDefine="APPCONF_ADC_RESC_SMART_REV_RAMP_TIME", valDouble="3",
        description=html("Time to go from 0 to Smart Reverse Max Duty Cycle.")),
     "ADC", "General", None),
    ("app_adc_conf.resc_hybrid", "app_adc_conf.use_filter", dict(
        longName="Hybrid Current / Duty Control", cDefine="APPCONF_ADC_RESC_HYBRID", valInt="0",
        description=html("Current control with a duty cycle limit proportional to the throttle: half throttle "
                         "gives half torque and, without load, about half speed (in current control alone, a few "
                         "percent of throttle bring an unloaded motor to full speed).",
                         "Duty limit = throttle x Duty Limit at Full Throttle x Maximum Duty Cycle. The current "
                         "is reduced linearly over the Current Taper Band below the limit and reaches zero at the "
                         "limit, so the speed settles without a sudden cut. Braking is not affected.",
                         "Applies to the current control types.")),
     "ADC", "General", "Hybrid Current / Duty"),
    ("app_adc_conf.resc_hybrid_duty", "app_ppm_conf.smart_rev_max_duty", dict(
        longName="Duty Limit at Full Throttle", cDefine="APPCONF_ADC_RESC_HYBRID_DUTY", valDouble="1",
        minDouble="0.05",
        description=html("Duty cycle limit at full throttle, as a fraction of Maximum Duty Cycle (Motor, "
                         "General). Lower it to cap the top speed.")),
     "ADC", "General", None),
    ("app_adc_conf.resc_hybrid_band", "app_ppm_conf.smart_rev_max_duty", dict(
        longName="Current Taper Band", cDefine="APPCONF_ADC_RESC_HYBRID_BAND", valDouble="0.15",
        minDouble="0.02", maxDouble="0.6",
        description=html("Part of the duty cycle limit over which the current decreases to zero. Wider: "
                         "softer approach of the limit, the speed settles a little lower under load. "
                         "Narrower: firmer, the current is held closer to the limit.")),
     "ADC", "General", None),
]


# Parametres places en tete de leur onglet
MC_FIRST = {("General", "General"): ["foc_motor_kind"]}


def put_first(root, first):
    for (group, sub), names in first.items():
        sp = find_subgroup(root, group, sub)
        for n in reversed(names):
            e = [p for p in sp if p.text == n][0]
            sp.remove(e)
            sp.insert(0, e)


def ser_names(root):
    return [s.text for s in root.find("SerOrder")]


def remove(root, names, hide_only=False):
    """Retire les parametres ; hide_only : seulement des onglets (serialisation inchangee)."""
    params, ser = root.find("Params"), root.find("SerOrder")
    for n in names:
        e = params.find(n)
        if e is None:
            raise SystemExit("parametre absent : " + n)
        if hide_only:
            continue
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


# Profil VESC700, vue RESC Tool : aides seulement (les choix et les defauts restent ceux de 7.00)
VIEW_EDIT_MC = {
    "foc_observer_type": dict(description=html(
        "Sensorless position observer: the VESC observers, re-implemented with the same equations. The RESC "
        "observers (types 7 to 13) are selected from the terminal, e.g. set foc_observer_type 7. Former text: "
        "<b>ORTEGA_ORIGINAL</b> = active flux with non-linear correction (recommended); "
        "<b>MXLEMMING</b> = active flux with radial correction (gain 2|w|); "
        "<b>..._LAMBDA_COMP</b> = same with flux linkage re-estimated while running; "
        "<b>MXV</b> = back-EMF observer (Luenberger, no current derivative); "
        "<b>MXV_LAMBDA_COMP_LIN</b> = hybrid active flux / back-EMF.")),
    "foc_sensor_mode": dict(description=html(
        "Angle source. Encoder and AB Encoder: ABI encoder (index not used). HFI and the four HFI variants: "
        "the RESC injection method (needs Lq &gt; Ld). VSS: position found by injection at standstill, "
        "then I/f without alignment and observer.")),
    "foc_observer_offset": dict(description=html(
        "Extra angle advance of the observer output in PWM periods, minus one: -1 (default) = none.")),
    "foc_cc_decoupling": dict(description=html(
        "Current loop feed-forward. With this configuration format, RESC always keeps the back-EMF "
        "feed-forward; Cross adds the w L i terms.")),
}


# Toujours transmis mais jamais affiches : RESC Tool lit leur valeur pour choisir ses pages
# (motor_type = FOC -> page FOC), l'utilisateur n'a rien a y choisir.
MC_HIDE = ["motor_type"]


def drop_unserialized(root):
    """Retire les fiches que le firmware ne recoit pas (marque, modele, poids, qualite du moteur...)."""
    params, ser = root.find("Params"), {s.text for s in root.find("SerOrder")}
    names = [p.tag for p in list(params) if p.tag not in ser]
    for n in names:
        params.remove(params.find(n))
    for sub in root.iter("subgroupParams"):
        for p in list(sub):
            if p.text in names:
                sub.remove(p)


def drop_empty_tabs(root):
    """Retire de <Grouping> les onglets sans parametre (separateurs seuls) et les groupes vides."""
    grouping = root.find("Grouping")
    for g in list(grouping):
        for sg in list(g.findall("subgroup")):
            sp = sg.find("subgroupParams")
            if sp is None or not [p for p in sp if not p.text.startswith("::sep::")]:
                g.remove(sg)
        if not g.findall("subgroup"):
            grouping.remove(g)


def main():
    mc = ET.parse(os.path.join(SRC, "parameters_mcconf.xml")).getroot()
    app = ET.parse(os.path.join(SRC, "parameters_appconf.xml")).getroot()
    remove(mc, MC_REMOVE)
    edit(mc, MC_EDIT)
    add(mc, MC_ADD)
    put_first(mc, MC_FIRST)
    remove(app, APP_REMOVE)
    edit(app, APP_EDIT)
    add(app, APP_ADD)
    remove(mc, MC_HIDE, hide_only=True)
    drop_unserialized(mc)
    drop_unserialized(app)
    drop_empty_tabs(mc)
    drop_empty_tabs(app)
    emap = {}
    apply_keep(mc, ENUM_KEEP, emap)
    apply_keep(app, APP_ENUM_KEEP, emap)
    with open(os.path.join(DST, "resc_enum_map.json"), "w") as fh:
        json.dump(emap, fh, indent=1)
    write(mc, os.path.join(DST, "resc_mcconf.xml"))
    write(app, os.path.join(DST, "resc_appconf.xml"))
    print("RESC   : mcconf %d parametres, appconf %d" % (len(ser_names(mc)), len(ser_names(app))))

    # vue RESC Tool du profil VESC700 : inutilises invisibles, aides RESC (sans choix ni defauts)
    def text_only(edits):
        keep = ("description", "longName")
        return {k: {kk: vv for kk, vv in v.items() if kk in keep} for k, v in edits.items()}

    vmc = ET.parse(os.path.join(SRC, "parameters_mcconf.xml")).getroot()
    vapp = ET.parse(os.path.join(SRC, "parameters_appconf.xml")).getroot()
    remove(vmc, MC_REMOVE, hide_only=True)
    edit(vmc, text_only(MC_EDIT))
    edit(vmc, VIEW_EDIT_MC)
    remove(vapp, APP_REMOVE, hide_only=True)
    edit(vapp, text_only(APP_EDIT))
    vdir = os.path.join(DST, "vesc700_view")
    os.makedirs(vdir, exist_ok=True)
    write(vmc, os.path.join(vdir, "parameters_mcconf.xml"))
    write(vapp, os.path.join(vdir, "parameters_appconf.xml"))
    vis = lambda r: len({p.text for p in r.iter("param") if not p.text.startswith("::sep::")})
    print("VESC700 (vue RESC Tool) : mcconf %d visibles / %d, appconf %d visibles / %d" % (
        vis(vmc), len(ser_names(vmc)), vis(vapp), len(ser_names(vapp))))


if __name__ == "__main__":
    sys.exit(main())
