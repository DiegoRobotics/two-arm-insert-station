"""Compare URDF FK (Pinocchio) with Annin HMI 6.0 FK on the same joint angles.

Mapping found 2026-10-22:
  joints:  annin_deg = SIGN * urdf_deg            (J1 inverted, no offsets)
  base:    Annin X = -URDF Y, Annin Y = URDF X, Z equal; Annin uses mm
  flange:  Annin flange frame = URDF ee_link rotated +90 deg about z
  Annin orientation output is a rotation vector (axis * angle, rad)
"""
import math
import sys

import numpy as np
import pinocchio as pin
import robot_kinematics as rk

from ar4tc_toolchanger.kinematics import KinematicsValidator

SIGN = np.array([-1, 1, 1, 1, 1, 1])
A2U = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])
# DH table as stored in the HMI's ARbot.cal (theta, alpha in deg; a, d in mm)
THETA, ALPHA = [0, -90, 0, 0, 0, 180], [0, -90, 0, -90, 90, -90]
A, D = [0, 64.2, 305, 0, 0, 0], [169.77, 0, 0, 222.63, 0, 41]
HMI_SAMPLE = ([0, -41.832, -4.284, 0, 89.981, 0], [44.668, 0.0, 529.075])  # saved in ARbot.cal


def annin_fk(q_annin_deg):
    o = rk.forward_kinematics([float(v) for v in q_annin_deg])
    return A2U @ np.array(o[:3]) / 1000.0, A2U @ pin.exp3(np.array(o[3:6])), np.linalg.norm(o[3:6])


def main(urdf_path, n=500):
    rk.robot_set()
    rk.set_dh_parameters_explicit(*map(math.radians, THETA), *map(math.radians, ALPHA), *A, *D)
    o = rk.forward_kinematics(HMI_SAMPLE[0])
    print(f"Annin FK reproduces HMI saved pose: {np.round(o[:3], 3).tolist()} vs {HMI_SAMPLE[1]}")

    kin = KinematicsValidator(open(urdf_path).read(), "ee_link")
    rng = np.random.default_rng(3)
    lo, hi = np.degrees(kin.lower) + 5, np.degrees(kin.upper) - 5
    pos, rot, R0 = [], [], None
    for _ in range(n):
        q = rng.uniform(lo, hi)
        p_a, R_a, ang = annin_fk(SIGN * q)
        M = kin.flange(np.radians(q))
        pos.append(np.linalg.norm(M.translation - p_a) * 1000)
        off = M.rotation.T @ R_a
        R0 = off if R0 is None else R0
        if ang < 3.1:  # Annin's float32 rotation-vector output is ill-conditioned near 180 deg
            rot.append(math.degrees(np.linalg.norm(pin.log3(R0.T @ off))))
    print(f"{n} random configs: position diff mean {np.mean(pos):.4f} mm, min {np.min(pos):.4f}, max {np.max(pos):.4f}")
    print(f"flange orientation offset constant within {max(rot):.4f} deg ({len(rot)} samples with |rotvec| < 3.1 rad)")
    print("URDF ee_link -> Annin flange rotation:\n", np.round(R0, 4))


if __name__ == "__main__":
    main(sys.argv[1])
