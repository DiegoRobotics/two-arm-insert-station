"""goto_pose for the real robot through the Annin HMI 6.0, with chosen joints held still (default J4, J5, J6).

Target = flange position X Y Z in HMI coordinates (mm, tool frame 0: what the HMI position display shows).
IK runs on Annin's own kinematics (annin_kinematics.py), so there is no URDF/HMI 7 mm difference and no
J1 sign conversion. Only position is solved: the tool orientation follows from J2/J3 and is reported.
The held joints are written with exactly the start values in every line, so the firmware gets a step
difference of 0 for them (checked below with the sketch v6.0 angle->step formula).

Output: a Move R program  start -> target -> pause -> back to start, checked for firmware limits, a 1 deg
margin on moving joints, per-joint travel, and that no DH frame (elbow, wrist, flange) goes below --min-z
along the joint-linear path the firmware drives.

No ROS needed:  python3 -m ar4tc_toolchanger.goto_pose_hmi X Y Z --start J1 J2 J3 J4 J5 J6 -o "<file>.ar4"
"""
import argparse
import time

import numpy as np

from . import annin_kinematics as ak
from . import annin_protocol as ap
from . import hmi_program as hp

# Sketch v6.0 step conversion (JxStepDeg as set by the HMI from ARbot.cal; JxaxisLimNeg)
STEP_DEG = np.array([44.4444, 55.5555, 55.5555, 49.7777, 21.8602, 22.2222])


def steps(q_deg):
    """Firmware target step for each joint: int((angle + lim_neg) * step_deg), as in the RJ handler."""
    return np.array([int((float(a) + n) * s) for a, n, s in zip(q_deg, ap.LIM_NEG, STEP_DEG)])


def solve(target_mm, q_start, free, seeds_deg=(0.0, 20.0, -20.0, 40.0, -40.0)):
    """Position IK over the free joints (indices), others fixed at q_start. Returns the in-limit solution
    closest to q_start, or None."""
    best = None
    for s in seeds_deg:
        q = np.array(q_start, float)
        q[free] += s
        for _ in range(100):
            e = target_mm - ak.xyz(q)
            if np.linalg.norm(e) < 1e-4:
                break
            J = np.zeros((3, len(free)))
            for k, i in enumerate(free):
                dq = np.zeros(6)
                dq[i] = 1e-4
                J[:, k] = (ak.xyz(q + dq) - ak.xyz(q)) / 1e-4
            q[free] += np.linalg.lstsq(J, e, rcond=None)[0].clip(-10, 10)
        q[free] = (q[free] + 180) % 360 - 180
        if np.linalg.norm(target_mm - ak.xyz(q)) < 0.01 and ap.within_limits(q):
            cost = np.max(np.abs(q - q_start))
            if best is None or cost < best[0]:
                best = (cost, q)
    return None if best is None else best[1]


def check_path(q0, q1, min_z, step_deg=0.5):
    """Joint-linear path q0 -> q1 (how the firmware moves); returns a list of problems."""
    n = max(2, int(np.ceil(np.max(np.abs(q1 - q0)) / step_deg)) + 1)
    for s in np.linspace(0, 1, n):
        q = q0 + (q1 - q0) * s
        for k, T in enumerate(ak.frames(q)[2:], start=3):      # frames 1-2 sit on the base
            if T[2, 3] < min_z:
                return [f"frame {k} at Z {T[2, 3]:.0f} mm (< {min_z:g}) at {np.round(q, 2).tolist()}"]
    return []


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("xyz", nargs=3, type=float, metavar=("X", "Y", "Z"), help="flange target, HMI coords, mm")
    a.add_argument("--start", nargs=6, type=float, required=True, metavar="Jn",
                   help="current joint angles exactly as the HMI shows them (deg)")
    a.add_argument("--hold", nargs="+", type=int, default=[4, 5, 6], help="joints that must not move (default 4 5 6)")
    a.add_argument("--speed", type=float, default=10, help="HMI Sp %% (default 10)")
    a.add_argument("--pause", type=float, default=2.0, help="seconds at the target before returning")
    a.add_argument("--max-travel", type=float, default=30.0, help="max degrees any joint may move (default 30)")
    a.add_argument("--min-z", type=float, default=150.0, help="lowest Z for elbow/wrist/flange, mm (default 150)")
    a.add_argument("--no-return", action="store_true", help="stay at the target (no move back to start)")
    a.add_argument("-o", "--out", required=True)
    args = a.parse_args()

    q0 = np.array(args.start)
    hold = sorted(set(args.hold))
    free = [i for i in range(6) if i + 1 not in hold]
    target = np.array(args.xyz)
    print(f"start {q0.tolist()}  (flange X {ak.xyz(q0)[0]:.1f} Y {ak.xyz(q0)[1]:.1f} Z {ak.xyz(q0)[2]:.1f} mm)")
    print(f"held joints J{' J'.join(map(str, hold))}; solving with J{' J'.join(str(i + 1) for i in free)}")

    q1 = solve(target, q0, free)
    if q1 is None:
        raise SystemExit(f"REFUSED: X {target[0]:g} Y {target[1]:g} Z {target[2]:g} is not reachable within the "
                         "firmware limits with the held joints fixed")
    q1 = np.array([float(f"{v:.3f}") for v in q1])            # exactly what the .ar4 line will contain
    q1[[h - 1 for h in hold]] = q0[[h - 1 for h in hold]]
    problems = []
    moving = np.abs(q1 - q0) > 1e-9
    if not ap.within_limits(np.where(moving, q1, 0.0), margin_deg=1.0) or not ap.within_limits(q1):
        problems.append(f"target {q1.tolist()} within 1 deg of a firmware limit")
    if (travel := np.max(np.abs(q1 - q0))) > args.max_travel:
        problems.append(f"a joint travels {travel:.1f} deg (> --max-travel {args.max_travel:g})")
    problems += check_path(q0, q1, args.min_z)
    held_steps = [h for h in hold if steps(q1)[h - 1] != steps(q0)[h - 1]]
    if held_steps:
        problems.append(f"held joints J{held_steps} would get a step change; use the exact HMI reading")
    T = ak.fk(q1)
    err = np.linalg.norm(T[:3, 3] - target)
    tool_tilt = np.degrees(np.arccos(np.clip(-T[2, 2], -1, 1)))   # angle between tool axis and straight down
    print(f"target q {q1.tolist()}  travel {np.round(q1 - q0, 3).tolist()}")
    print(f"flange at X {T[0, 3]:.2f} Y {T[1, 3]:.2f} Z {T[2, 3]:.2f} mm (error {err:.3f} mm); "
          f"tool axis {tool_tilt:.1f} deg from straight down")
    if problems:
        for p in problems:
            print("  PROBLEM:", p)
        raise SystemExit("REFUSED: no program written")

    lim = "J" + " J".join(map(str, hold))
    sp = lambda q: hp.move_r(q, args.speed, 20, 20, 100)
    lines = [
        "##goto_pose (HMI) - generated by ar4tc goto_pose_hmi (do not edit by hand)",
        f"##generated {time.strftime('%Y-%m-%d %H:%M')}; target flange X {target[0]:g} Y {target[1]:g} Z {target[2]:g} mm "
        "(HMI coords, tool frame 0)",
        f"##HELD STILL: {lim}. ONLY RUN IF THE HMI SHOWS EXACTLY: "
        + " ".join(f"J{i + 1} {q0[i]:.3f}" for i in range(6)),
        f"##(if {lim} read anything else, those joints WOULD move to the values below - regenerate with --start)",
        "##J1-J3 must be calibrated. Area clear, hand on the E-stop.",
        "##start pose (no motion if the robot is already there)",
        sp(q0),
        f"##target: flange X {T[0, 3]:.1f} Y {T[1, 3]:.1f} Z {T[2, 3]:.1f} mm, tool axis {tool_tilt:.1f} deg from down",
        sp(q1),
        hp.wait(args.pause),
    ]
    if not args.no_return:
        lines += ["##back to start", sp(q0)]
    lines.append("##END")
    hp.write(args.out, lines)
    print(f"wrote {args.out}  RESULT: OK")


if __name__ == "__main__":
    main()
