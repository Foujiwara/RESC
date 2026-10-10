#!/usr/bin/env python3
"""
Analyse des fenetres de mesure low-side (3 shunts) en SVPWM centree (min-max injection).

Pour chaque indice de modulation et chaque angle, calcule :
  - la fenetre low-side de la 2e phase la plus "basse" (2 shunts suffisent, Ic = -Ia-Ib)
  - la fenetre low-side de la phase la plus haute (3 shunts necessaires)
Puis en deduit la frequence PWM max compatible avec une fenetre minimale T_win.

Convention : m = |Vref| / (Vdc/sqrt(3))  -> m = 1 = limite lineaire SVPWM.
La fenetre low-side de la phase x vaut (1 - d_x) * Ts (PWM centree, tous les
low-side ON autour du sommet du compteur).

Usage : python svpwm_window.py
"""
import math

def duties(m, theta):
    a = m / math.sqrt(3.0)  # amplitude phase en p.u. de Vdc
    v = [a * math.cos(theta - k * 2.0 * math.pi / 3.0) for k in range(3)]
    off = -(max(v) + min(v)) / 2.0
    return [0.5 + x + off for x in v]

def worst_windows(m, n=3600):
    worst2 = 1.0  # pire fenetre de la 2e phase (mesure 2 shunts)
    worst3 = 1.0  # pire fenetre de la phase la plus haute (mesure 3 shunts)
    for i in range(n):
        d = sorted(duties(m, 2.0 * math.pi * i / n))
        worst3 = min(worst3, 1.0 - d[2])
        worst2 = min(worst2, 1.0 - d[1])
    return worst2, worst3

def main():
    print("== Fenetre low-side minimale (fraction de Ts) en fonction de m ==")
    print(f"{'m':>5} {'2-shunts':>10} {'3-shunts':>10}")
    for m in [0.5, 0.7, 0.8, 0.9, 0.95, 1.0]:
        w2, w3 = worst_windows(m)
        print(f"{m:5.2f} {w2:10.4f} {w3:10.4f}")

    print("\n== f_PWM max pour garder 2 shunts valides a m = 1.0 ==")
    w2, _ = worst_windows(1.0)
    for t_win_us in [1.0, 1.5, 2.0, 2.5, 3.0]:
        fmax = w2 / (t_win_us * 1e-6)
        print(f"T_win = {t_win_us:3.1f} us -> f_PWM <= {fmax/1e3:6.1f} kHz")

    print("\n== Erreur d'angle due au retard non compense (delta = we * Td) ==")
    print("Td = 1.5 Ts (echantillonnage k, application k+1, ZOH)")
    print(f"{'f_PWM':>7} " + " ".join(f"{fe:>8}" for fe in ['0.5kHz', '1kHz', '2kHz', '3kHz']))
    for fpwm in [20e3, 30e3, 40e3, 60e3]:
        td = 1.5 / fpwm
        row = [math.degrees(2 * math.pi * fe * td) for fe in [500, 1000, 2000, 3000]]
        print(f"{fpwm/1e3:5.0f}k  " + " ".join(f"{x:8.1f}" for x in row))

    print("\n== Ondulation de courant crete-crete (borne haute Vdc/(4 L f)) ==")
    for vdc in [48, 72, 100]:
        for l_uh in [5, 10, 30]:
            row = [vdc / (4 * l_uh * 1e-6 * f) for f in [20e3, 30e3, 40e3]]
            print(f"Vdc={vdc:3d} V L={l_uh:3d} uH : " +
                  "  ".join(f"{f/1e3:.0f}k:{r:6.1f} A" for f, r in zip([20e3, 30e3, 40e3], row)))

if __name__ == "__main__":
    main()
