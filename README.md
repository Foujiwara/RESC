# RESC — High-performance FOC firmware for STM32F405RGT6

**English** · [Français](README.fr.md)

A **brand new** motor control firmware for permanent magnet (PMSM / BLDC / IPM) and induction (ACIM) motors. It is not derived from VESC.
For interoperability with VESC Tool, displays and existing controllers, RESC reuses from VESC (`release_7_00`) the **CAN/UART protocols**, the **configuration format and default values**, the **board data** (pinouts, values) and, as selectable options, the **VESC observer equations** (re-implemented under their original names). Details in [NOTICE.md](NOTICE.md). The control core (FOC, RESC observers, control, protections, detections) is an independent development.

> **Status: firmware 1.0, validated in simulation and by PC tests, not yet tried on the board.** v0.1 was tried on the board (USB, VESC Tool, real-time data, rotation after flux detection).
> Test board: **Ubox SINGLE 85_200**. Procedure: [docs/12_essai_banc.md](docs/12_essai_banc.md). Functions and settings: [docs/14_fonctions_v1.md](docs/14_fonctions_v1.md) (French).

## Firmware

| Item | Where | Status |
|---|---|---|
| RESC FOC core (d/q PI, configurable feed-forward, anti-windup, SVPWM, delay compensation, 3-shunt reconstruction) | `firmware/src/core/foc.c` | simulation OK |
| 14 observers: the 7 VESC 7.00 observers (ORTEGA_ORIGINAL, MXLEMMING, MXV and their LAMBDA_COMP variants) and 7 RESC observers (active flux, radial pull, Luenberger back-EMF, hybrid, adaptive λ) | `firmware/src/core/observer.c` | simulation OK |
| Sensors: Hall (blend to observer, majority vote), ABI encoder, HFI, VSS | `core/foc.c`, `core/angle_sensors.c` | simulation OK |
| Damped I/f start, VESC sensorless options, saturation and temperature compensation, MTPA, field weakening, phase filters | `foc.c` | simulation OK |
| Induction motor: rotor flux oriented control, encoder or sensorless, Lm/Lr/Rr identification | `foc.c`, `app/detect_app.c` | simulation OK |
| Manual detections: current and voltage offsets, R/L (3 steps), flux, Hall, encoder, induction | `foc.c`, `app/detect_app.c` | simulation OK |
| Control: current, brake, duty (voltage mode, down-ramp brake), speed, position, open loop, limits | `firmware/src/core/ctrl.c` | simulation OK |
| Apps: PPM, ADC (+ buttons, RESC smart reverse and hybrid current / duty control), VESC Remote, PAS, multi-ESC over CAN with traction control, kill switch | `firmware/src/app/apps.c`, `bsp/inputs.c` | PC tests OK |
| Configuration: generated from the XMLs, storage by name with migration, description sent to RESC Tool (command 250) | `firmware/conf/`, `tools/resc_conf/`, `src/vesc/`, `src/comm/conf_desc.c` | PC tests OK |
| VESC 7.00 features: V0/V7 sampling, overmodulation, brake held to standstill with low-side short, handbrake, sampled data, statistics, odometer, absolute encoders (all VESC sensor port modes except Custom: AS5047, MT6816, AS5x47U, TLE5012, AMT22, MA782, BiSS-C, TS5700N8501, AD2S1205, sin/cos, PWM, PWM + ABI), IMU (BMI160), VESC BMS over CAN, setup wizard (auto detection, CAN included), firmware update over the CAN bus | `core/`, `bsp/abs_enc.c`, `app/imu.c`, `app/bms.c`, `app/detect_all.c`, `comm/sampler.c` | simulation and PC tests OK |
| Protections: software overcurrent + ADC watchdog, overvoltage, undervoltage, temperatures, driver fault, break, overspeed; since 1.7 (doc 15): sticky shutdown, flying restart validated against the measured back-EMF, loss-of-sync detection from the current prediction residual, FOC ISR deadline (fault 34), lock after repeated overcurrents, never a live bridge without calibration | `foc.c`, `ctrl.c`, `bsp/motor_hw.c` | partly simulated |
| Pre-fault recorder: 256 periods around the first fault, kept across a software reset, terminal `fault_dump` (CSV) | `core/blackbox.c` | simulation OK |
| **Known limitation (1.7)**: flying restart at very high speed (40,000 ERPM, 14 µH / 8 mΩ motor) with real phase dividers: ~47 A peak for 2 A requested. Already present in 1.6 (without shutdown); 1.7 shuts down on the first peak. See doc 15 §15.7 | `foc.c` | **to fix** |
| STM32F405: TIM1, triple ADC, TIM4 (servo), UART, CAN, USB CDC, flash, IWDG | `firmware/src/bsp/` | **to validate on the board** |

## Configuration

RESC has its own parameter set: 179 motor and 111 application parameters, derived from VESC 7.00 (no BLDC, DC, DRV8301, UAVCAN, NRF24, slow absolute current; added: induction motor, current loop bandwidth, hybrid current / duty control, Basic Duty Mode, smart reverse on the ADC...). Only the options RESC and the board support remain in the lists. The firmware sends its own description of the parameters to **RESC Tool** (command 250), so the tool needs no built-in list. The official VESC Tool, which has VESC 7.00's list built in, cannot read this configuration.

**Motor Type** (Motor > General > General): **FOC Permanent Magnets (PMSM / BLDC / IPM)** or **FOC Induction (ACIM)**.

The firmware builds the tabs it sends to RESC Tool from the current configuration: motor type, angle source (sensorless, encoder, Hall, HFI, VSS), options (temperature sensor, field weakening, phase filter...) and app control types. A parameter is listed only when it is used; empty tabs and groups disappear. Hidden values are still transmitted and kept. After changing the motor type or the sensor, write the configuration and reconnect to refresh the list. The display rules are in [tools/resc_conf/desc_vis.py](tools/resc_conf/desc_vis.py).

**Dynamic display (command 250, bit 7):** a RESC Tool that sends `0x80 | conf` receives every tab, each with a `visibleIf` condition, plus a `<Pages>` section. It then shows and hides the parameters while you edit, with no write or reconnect. Without the bit the reply is unchanged. Sample streams: [tools/resc_conf/samples/](tools/resc_conf/samples/).

## Build and flash

```bash
python firmware/build.py
```

```bash
python firmware/build.py --sim
```

```bash
python firmware/build.py --tests
```

Ready-to-flash images in `firmware/release/`:
- `resc_ubox_single_85_200.bin` / `.hex`: the firmware (still 1.6: 1.7 is not published until the known limitation above is dealt with; `python firmware/build.py` builds it). It works with **RESC Tool** (the version that reads the description sent by the firmware), not with the official VESC Tool, whose built-in parameter list differs.

Console: `python tools/resc_cli.py`. The terminal (`help`) gives every command, including the detections (`detect_rl`, `detect_acim`...), `get` / `set` / `list` for all parameters, `faults` (faults, refused restarts, loss-of-sync margin) and `fault_dump` (pre-fault recorder).

## Reliability convention (used in all documents)

| Tag | Meaning |
|---|---|
| **[C]** | **Confirmed**: checked in the datasheet, the RM0090 reference manual or the VESC 7.00 source |
| **[E]** | **Estimated**: calculation, order of magnitude or experience. To be measured |
| **[V]** | **To validate**: depends on the real hardware or on a test. Do not use as a safety assumption |

## Documents (French)

| # | Document | Content |
|---|---|---|
| 1 | [STM32F405RGT6 audit](docs/01_audit_mcu.md) | Features, peripherals, constraints, pin multiplexing |
| 2 | [VESC 7.00 protocols](docs/02_protocole_vesc.md) | CAN, UART, messages, encodings, VESC Tool compatibility |
| 3 | [Real-time architecture](docs/03_architecture_temps_reel.md) | Bare-metal, RTOS, hybrid; priorities; instrumentation |
| 4 | [PWM, ADC, currents](docs/04_pwm_adc_courants.md) | Modulation, sampling, 3 shunts, frequencies |
| 5 | [FOC control](docs/05_commande_foc.md) | Regulators, decoupling, saturation, MTPA, FW, dead time |
| 6 | [Sensorless observers](docs/06_observateurs.md) | Comparison of 10 methods, interface, comparison mode |
| 7 | [Protection and loss of sync](docs/07_protection.md) | Absolute hardware overcurrent, safe state, stall detection |
| 8 | [Missing hardware information](docs/08_infos_manquantes.md) | What must be provided before coding |
| 9 | [Recommended architecture](docs/09_architecture_recommandee.md) | Summary, tree, interfaces, configuration |
| 10 | [Roadmap](docs/10_feuille_de_route.md) | Steps, pass criteria, validation |
| 11 | [Supported boards](docs/11_cartes_vesc.md) | VESC-compatible hardware configuration, survey, Ubox reference board |
| 12 | [First bench test](docs/12_essai_banc.md) | Backup, flashing, checks, detections, first turns |
| 13 | [Code audit and FOC independence](docs/13_audit_code_et_independance_foc.md) | What comes from VESC, what is RESC's own |
| 14 | [RESC 1.0 functions](docs/14_fonctions_v1.md) | Observers, sensors, sensorless, phase filters, apps, induction, configuration |
| 15 | [Firmware 1.6 audit and 1.7 fixes](docs/15_audit_1.6.md) | Safety defects, race conditions, pre-fault recorder, high-speed restart |
| — | [RESC 0.1 vs VESC 7.00 (PDF, English)](docs/RESC_vs_VESC_FOC_observer.pdf) | FOC core and observer comparison, **version 0.1** (outdated: no HFI, single observer at the time) |

## Hardware configuration

A board is selected at compile time, like VESC's `HW_HEADER`:

```
-DRESC_BOARD_HEADER='"boards/ubox/board_ubox_single_85_200.h"'
```

- [hw/hw_config.h](hw/hw_config.h): entry point and contract (`BOARD_*` = hardware, `HWLIM_*` = absolute limits, `DEF_*` = defaults).
- [hw/boards/common/board_common_f405.h](hw/boards/common/board_common_f405.h): common pinout, taken from the 172 VESC 7.00 configurations.
- [hw/boards/ubox/](hw/boards/ubox/): reference board, reviewed by hand.
- `hw/boards/imported/`: 150 automatically generated boards (status IMPORTED, bridge forbidden until reviewed).
- Check: `python -I tools/hw_import/check_boards.py . --quiet`

Analysis tools: [tools/analysis/svpwm_window.py](tools/analysis/svpwm_window.py) (3-shunt measurement windows, delays, ripple) and [tools/analysis/vesc_hw_survey.py](tools/analysis/vesc_hw_survey.py) (VESC board survey).

## Design choices

| Area | Choice | Planned alternative |
|---|---|---|
| Scheduling | Hybrid: FOC ISR outside any kernel + run-to-completion scheduler on the NVIC (no third-party RTOS) | FreeRTOS for non-critical tasks |
| PWM | Centre-aligned, min-max SVPWM, 1 sample + 1 update, delay compensated by angle prediction | Double sampling / double update for low-L motors |
| Current sensing | 3 shunts, dynamic choice of the 2 best phases, adaptive ADC trigger, model prediction fallback | Phase shunts (hardware variant) |
| Default f_PWM | Per board: 15 kHz on the Ubox (= VESC `f_zv/2`), range 8–40 kHz, checked against the ADC window | 20–25 kHz after measuring T_win |
| Current | d/q PI with decoupling + back-EMF feed-forward, back-calculation anti-windup, vector limiting | Predictive deadbeat (option, low-L motors) |
| Sensorless | Selectable: VESC observers or RESC active flux / back-EMF observers + type-2 PLL | HFI / VSS at low speed (IPM) |
| Absolute protection | Per board (`BOARD_HW_OCP`): BKIN if wired, else nFAULT → software break, else ADC watchdog + software break. **On the Ubox: software only** (doc 07 §7.8) | Hardware mod: comparator → PB12/BKIN |
| Watchdog | IWDG + **dynamic enable** of the driver (the signal dies if the FOC ISR stops) | Timer deadline monitor → BKIN |

## License

GNU GPL v3 (see [LICENSE](LICENSE)). Third-party code and elements taken from the VESC project for compatibility: see [NOTICE.md](NOTICE.md).

> Experimental power-electronics firmware, still in bench testing:
> use with a current-limited supply, at your own risk.
