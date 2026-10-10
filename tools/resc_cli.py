#!/usr/bin/env python3
"""
resc_cli.py — Console RESC sur port serie (USB ou UART), protocole paquet.

  python tools/resc_cli.py                  detecte le port USB (0483:5740)
  python tools/resc_cli.py COM5
  python tools/resc_cli.py COM5 --values    affiche les valeurs temps reel en boucle

Dans la console : toute ligne est envoyee au terminal du firmware (help, status,
rt, offsets_cal, detect_rl, ...). Commandes locales : 'values' (une mesure),
'watch' (mesures en continu, Ctrl+C pour arreter), 'quit'.
La console envoie un maintien toutes les 250 ms : une consigne reste active tant
qu'elle est ouverte ; la fermer (ou perdre la liaison) relache le moteur en 1 s.
Necessite pyserial.
"""
import struct
import sys
import threading
import time

import serial
import serial.tools.list_ports

COMM_FW_VERSION, COMM_GET_VALUES, COMM_TERMINAL_CMD, COMM_PRINT, COMM_ALIVE = 0, 4, 20, 21, 30


def crc16(data):
    c = 0
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def frame(payload):
    n = len(payload)
    head = bytes([2, n]) if n <= 255 else bytes([3, n >> 8, n & 0xFF])
    c = crc16(payload)
    return head + payload + bytes([c >> 8, c & 0xFF, 3])


class Link:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.05)
        self.buf = bytearray()
        self.on_packet = None
        self.alive = True
        self.lock = threading.Lock()
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._keepalive, daemon=True).start()

    def _keepalive(self):
        # maintien de la consigne tant que la console est ouverte (timeout firmware : 1 s)
        while self.alive:
            try:
                self.send([COMM_ALIVE])
            except serial.SerialException:
                return
            time.sleep(0.25)

    def send(self, payload):
        with self.lock:
            self.s.write(frame(bytes(payload)))

    def _rx(self):
        while self.alive:
            try:
                d = self.s.read(256)
            except serial.SerialException:
                print("\n[liaison perdue]")
                self.alive = False
                return
            if d:
                self.buf += d
                self._parse()

    def _parse(self):
        b = self.buf
        while b:
            if b[0] == 2 and len(b) >= 2:
                hl, n = 2, b[1]
            elif b[0] == 3 and len(b) >= 3:
                hl, n = 3, (b[1] << 8) | b[2]
            elif b[0] in (2, 3):
                return
            else:
                del b[0]
                continue
            tot = hl + n + 3
            if len(b) < tot:
                return
            pl = bytes(b[hl:hl + n])
            if b[tot - 1] == 3 and crc16(pl) == ((b[hl + n] << 8) | b[hl + n + 1]):
                if self.on_packet:
                    self.on_packet(pl)
                del b[:tot]
            else:
                del b[0]


def decode_values(pl):
    i = 1
    def f16(s):
        nonlocal i
        v = struct.unpack(">h", pl[i:i + 2])[0] / s; i += 2; return v
    def f32(s):
        nonlocal i
        v = struct.unpack(">i", pl[i:i + 4])[0] / s; i += 4; return v
    t_fet, t_mot = f16(10), f16(10)
    i_mot, i_in, i_d, i_q = f32(100), f32(100), f32(100), f32(100)
    duty, erpm, vin = f16(1000), f32(1), f16(10)
    ah, ahc, wh, whc = f32(1e4), f32(1e4), f32(1e4), f32(1e4)
    tacho, tacho_abs = f32(1), f32(1)
    fault = pl[i]
    return (f"Vbus {vin:5.1f} V | ERPM {erpm:8.0f} | duty {duty:6.3f} | Imot {i_mot:6.1f} A | Ibat {i_in:6.1f} A | "
            f"Id {i_d:6.1f} Iq {i_q:6.1f} | T_FET {t_fet:5.1f} | T_mot {t_mot:5.1f} | Wh {wh:6.2f} | defaut {fault}")


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483 and p.pid == 0x5740:
            return p.device
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else find_port()
    if not port:
        print("Aucun port RESC detecte. Ports disponibles :")
        for p in serial.tools.list_ports.comports():
            print("  ", p.device, p.description)
        sys.exit(1)
    link = Link(port)
    last_values = [None]

    def on_packet(pl):
        if pl[0] == COMM_PRINT:
            print(pl[1:].decode("latin-1"))
        elif pl[0] == COMM_GET_VALUES:
            last_values[0] = decode_values(pl)
            print(last_values[0])
        elif pl[0] == COMM_FW_VERSION:
            name_end = pl.index(0, 3)
            hw = pl[3:name_end].decode("latin-1")
            fw = pl[name_end + 1 + 12 + 8:].split(b"\0")[0].decode("latin-1", "replace")
            print(f"Connecte : firmware {fw} {pl[1]}.{pl[2]:02d} sur {hw} ({port})")

    link.on_packet = on_packet
    link.send([COMM_FW_VERSION])
    time.sleep(0.3)

    if "--values" in sys.argv:
        try:
            while True:
                link.send([COMM_GET_VALUES])
                time.sleep(0.2)
        except KeyboardInterrupt:
            return

    print("Console RESC : 'help' pour les commandes du firmware, 'values', 'watch', 'quit'.")
    while link.alive:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        if line in ("quit", "exit"):
            break
        if line == "values":
            link.send([COMM_GET_VALUES])
        elif line == "watch":
            try:
                while True:
                    link.send([COMM_GET_VALUES])
                    time.sleep(0.2)
            except KeyboardInterrupt:
                print()
        else:
            link.send([COMM_TERMINAL_CMD] + list(line.encode("latin-1")))
        time.sleep(0.25)
    link.alive = False


if __name__ == "__main__":
    main()
