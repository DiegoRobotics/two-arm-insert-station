"""IK validation: random reachable poses -> MoveIt IK -> independent Pinocchio FK check.

Poses come from FK of random joint vectors inside the limits, so every target is reachable.
usage: ros2 run ar4tc_toolchanger ik_benchmark [N]
"""
import sys
import time

import numpy as np
import rclpy

from .common import connect, load_config


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    rclpy.init()
    node = rclpy.create_node("ar4tc_ik_benchmark")
    cfg = load_config()
    cfg["tcp_offset"] = [0.0, 0.0, 0.0]  # benchmark the flange
    mv, kin = connect(node, cfg)
    tol = cfg["tolerances"]
    rng = np.random.default_rng(42)
    margin = np.radians(5)
    pos, rot, times, fails, rejected, singular = [], [], [], 0, 0, 0
    for _ in range(n):
        q_true = rng.uniform(kin.lower + margin, kin.upper - margin)
        target = kin.fk(q_true)
        t0 = time.time()
        q = mv.compute_ik(target, np.zeros(6), avoid_collisions=False, attempts=3)
        times.append(time.time() - t0)
        if q is None:
            fails += 1
            continue
        c = kin.validate(q, target, tol["pos_mm"], tol["rot_deg"], 0.0)
        pos.append(c.pos_mm)
        rot.append(c.rot_deg)
        rejected += not c.ok
        singular += kin.manipulability(q_true) < tol["min_manipulability"]
    ok = n - fails - rejected
    print(f"IK benchmark ({kin.model.name}, solver from kinematics.yaml), N={n}, seed=home")
    print(f"  solved+validated: {ok}/{n} ({100 * ok / n:.1f}%)  no solution: {fails}  rejected by FK check: {rejected}")
    if pos:
        print(f"  position error mm: mean {np.mean(pos):.5f}  max {np.max(pos):.5f}")
        print(f"  orientation error deg: mean {np.mean(rot):.5f}  max {np.max(rot):.5f}")
    print(f"  time per pose s: mean {np.mean(times):.3f}  max {np.max(times):.3f}")
    print(f"  sampled configs below manipulability threshold: {singular}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
