"""Stand-in for the AR4 Teensy (sketch v6.0) on a pseudo-terminal, for testing the bridge without hardware.

Implements RP and RJ the way the sketch does (limit check, blocking move, position reply). Other
commands are ignored, as the firmware ignores unknown ones. Not a physics model.

usage: ros2 run ar4tc_toolchanger annin_emulator [--link /tmp/ttyANNIN] [--speedup 1]
                                                  [--estop-at-move N] [--collision-at-move N]
"""
import argparse
import os
import time
import tty

import numpy as np

from . import annin_protocol as ap

REST = [0.0, -41.832, -4.284, 0.0, 89.981, 0.0]   # last pose saved by the HMI (ARbot.cal)


def position_line(q, flag=""):
    j = "".join(f"{k}{v:.3f}" for k, v in zip("ABCDEF", q))
    return f"{j}G0.000H0.000I0.000J0.000K0.000L0.000M0N{''}O{flag}P0.00Q0.00R0.00\r\n"


def parse_rj(data):
    """Same field lookup as the sketch: indexOf of each marker."""
    idx = {k: data.index(k) for k in ("A", "B", "C", "D", "E", "F", "J7", "S", "Ac")}
    order = ["A", "B", "C", "D", "E", "F", "J7"]
    q = [float(data[idx[a] + 1:idx[b]]) for a, b in zip(order, order[1:])]
    s = data[idx["S"] + 1:]
    return np.array(q), s[0], float(s[1:s.index("Ac")])


def main():
    ap_ = argparse.ArgumentParser()
    ap_.add_argument("--link", default="/tmp/ttyANNIN")
    ap_.add_argument("--speedup", type=float, default=1.0)
    ap_.add_argument("--estop-at-move", type=int, default=0)
    ap_.add_argument("--collision-at-move", type=int, default=0)
    ap_.add_argument("--start-zero", action="store_true", help="start like a freshly reset (uncalibrated) Teensy")
    args, _ = ap_.parse_known_args()

    master, slave = os.openpty()
    tty.setraw(slave)
    if os.path.lexists(args.link):
        os.remove(args.link)
    os.symlink(os.ttyname(slave), args.link)
    print(f"annin_emulator: {args.link} -> {os.ttyname(slave)}", flush=True)

    q, moves, buf = np.zeros(6) if args.start_zero else np.array(REST), 0, b""
    while True:
        buf += os.read(master, 1024)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", "replace").strip()
            fn, data = line[:2], line[2:]
            if fn == "RP":
                time.sleep(0.005)
                os.write(master, position_line(q).encode())
            elif fn == "RJ":
                moves += 1
                target, stype, sval = parse_rj(data)
                fault = [int(not (-ap.LIM_NEG[i] <= target[i] <= ap.LIM_POS[i])) for i in range(6)]
                if any(fault):
                    os.write(master, ("EL" + "".join(map(str, fault)) + "000\r\n").encode())
                    continue
                dur = (sval * 1.2 if stype == "s" else 1.0) / args.speedup
                if moves == args.estop_at_move:
                    time.sleep(dur / 2)
                    q = q + 0.5 * (target - q)
                    os.write(master, position_line(q, "EB").encode())
                    continue
                time.sleep(dur)
                if moves == args.collision_at_move:
                    q = q + 0.9 * (target - q)
                    os.write(master, position_line(q, "EC010000").encode())
                    continue
                q = target + np.random.default_rng(moves).uniform(-0.01, 0.01, 6)  # encoder-level noise
                os.write(master, position_line(q).encode())


if __name__ == "__main__":
    main()
