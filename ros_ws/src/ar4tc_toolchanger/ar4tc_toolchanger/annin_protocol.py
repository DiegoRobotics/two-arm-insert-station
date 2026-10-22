"""Annin AR4 Teensy sketch v6.0 serial protocol (subset) + trajectory helpers. No ROS imports.

Commands used (from AR4_teensy41_sketch_v6.0.ino):
  RP\n   request position  -> "A<j1>B<j2>C<j3>D<j4>E<j5>F<j6>G<x>H<y>I<z>J<rz>K<ry>L<rx>M<speedViol>N<debug>O<flag>P<j7>Q<j8>R<j9>"
  RJ...\n move in joints   -> same position line when done, or an alarm line ("EL<faults>", "ER")
Angles are degrees in Annin's convention. Flags in field O: "EC<6 bits>" encoder/collision, "EB" E-stop.
"""
import math
from dataclasses import dataclass, field

import numpy as np

# ROS joint (rad) = SIGN * Annin joint (deg -> rad). Measured by FK cross-check (KINEMATICS.md).
SIGN = np.array([-1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
# Annin v6.0 firmware axis limits (deg), sketch lines 152-163: angle must be within [-NEG, +POS].
LIM_POS = np.array([170.0, 90.0, 52.0, 180.0, 105.0, 180.0])
LIM_NEG = np.array([170.0, 42.0, 89.0, 180.0, 105.0, 180.0])

_RP_KEYS = "ABCDEFGHIJKLMNOPQR"


def ros_to_annin(q_rad):
    return SIGN * np.degrees(np.asarray(q_rad, dtype=float))


def annin_to_ros(q_deg):
    return np.radians(SIGN * np.asarray(q_deg, dtype=float))


@dataclass
class Reply:
    ok: bool
    joints_deg: np.ndarray = field(default_factory=lambda: np.zeros(6))
    ext_deg: tuple = (0.0, 0.0, 0.0)
    xyzuvw: tuple = ()
    flag: str = ""
    speed_violation: bool = False
    alarm: str = ""
    raw: str = ""

    @property
    def estop(self):
        return self.flag.startswith("EB")

    @property
    def collision(self):
        return self.flag.startswith("EC") and "1" in self.flag[2:]


def parse_reply(line: str) -> Reply:
    """Parse a position line or an alarm line from the Teensy."""
    s = line.strip()
    if not s.startswith("A"):
        return Reply(ok=False, alarm=s or "<empty>", raw=s)
    vals, pos = {}, 0
    for i, key in enumerate(_RP_KEYS):
        start = s.find(key, pos)
        if start < 0:
            return Reply(ok=False, alarm=f"malformed position line (missing {key})", raw=s)
        nxt = s.find(_RP_KEYS[i + 1], start + 1) if i + 1 < len(_RP_KEYS) else len(s)
        if nxt < 0:
            return Reply(ok=False, alarm=f"malformed position line (missing {_RP_KEYS[i + 1]})", raw=s)
        vals[key], pos = s[start + 1:nxt], nxt
    try:
        joints = np.array([float(vals[k]) for k in "ABCDEF"])
        xyz = tuple(float(vals[k]) for k in "GHIJKL")
        ext = tuple(float(vals[k] or 0) for k in "PQR")
    except ValueError as e:
        return Reply(ok=False, alarm=f"malformed number: {e}", raw=s)
    r = Reply(ok=True, joints_deg=joints, ext_deg=ext, xyzuvw=xyz, flag=vals["O"].strip(),
              speed_violation=vals["M"].strip() == "1", raw=s)
    if r.estop:
        r.ok, r.alarm = False, "E-STOP (flag EB)"
    elif r.collision:
        r.ok, r.alarm = False, f"encoder/collision fault (flag {r.flag})"
    return r


def within_limits(q_deg, margin_deg=0.0):
    q = np.asarray(q_deg)
    return bool(np.all(q >= -LIM_NEG + margin_deg) and np.all(q <= LIM_POS - margin_deg))


def format_rj(q_deg, seconds, ext_deg=(0.0, 0.0, 0.0), acc=20, dcc=20, ramp=20, wrist="F", loop="000000"):
    """Move-in-joints command. Speed type 's' = complete the move in <seconds>.
    loop '0' per joint = closed loop (encoder check after the move)."""
    j = "".join(f"{k}{v:.3f}" for k, v in zip("ABCDEF", q_deg))
    e = "".join(f"J{n}{v:.3f}" for n, v in zip((7, 8, 9), ext_deg))
    return f"RJ{j}{e}Ss{seconds:.2f}Ac{acc:g}Dc{dcc:g}Rm{ramp:g}W{wrist}Lm{loop}\n"


def simplify(times, points_deg, tol_deg):
    """Douglas-Peucker on a joint-space path. The firmware moves all joints linearly in step
    space between RJ targets, so a kept segment deviates from the planned path by <= tol_deg."""
    pts = np.asarray(points_deg, dtype=float)
    keep = {0, len(pts) - 1}

    def rec(i, j):
        if j <= i + 1:
            return
        seg = pts[j] - pts[i]
        L = np.dot(seg, seg)
        worst, k_worst = -1.0, None
        for k in range(i + 1, j):
            t = 0.0 if L == 0 else np.clip(np.dot(pts[k] - pts[i], seg) / L, 0, 1)
            d = np.max(np.abs(pts[k] - (pts[i] + t * seg)))
            if d > worst:
                worst, k_worst = d, k
        if worst > tol_deg:
            keep.add(k_worst)
            rec(i, k_worst)
            rec(k_worst, j)

    rec(0, len(pts) - 1)
    idx = sorted(keep)
    return [times[i] for i in idx], pts[idx]


def segment_durations(times, points_deg, max_speed_deg_s, min_seconds):
    """Planned time per segment, slowed down if any joint would exceed max_speed_deg_s."""
    out = []
    for k in range(1, len(points_deg)):
        planned = times[k] - times[k - 1]
        speed_floor = float(np.max(np.abs(points_deg[k] - points_deg[k - 1]))) / max_speed_deg_s
        out.append(max(planned, speed_floor, min_seconds))
    return out


def max_step_deg(points_deg):
    p = np.asarray(points_deg)
    return float(np.max(np.abs(np.diff(p, axis=0)))) if len(p) > 1 else 0.0


def duration_from_msg(d):
    return d.sec + d.nanosec * 1e-9


if __name__ == "__main__":  # quick manual check
    print(format_rj([0, -41.832, -4.284, 0, 89.981, 0], 2.0), end="")
    print(parse_reply("A0.000B-41.832C-4.284D0.000E89.981F0.000G44.668H0.000I529.075J0.000K133.865L-0.000M0NOP0Q0R0"))
    print(math.degrees(annin_to_ros([10, 0, 0, 0, 0, 0])[0]))
