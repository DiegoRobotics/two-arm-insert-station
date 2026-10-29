"""Milestone 1: move the simulated MK3 TCP to a Cartesian pose.

usage: ros2 run ar4tc_toolchanger goto_pose X Y Z ROLL PITCH YAW   (m, deg, base_link)
"""
import argparse
import math

import numpy as np
import rclpy

from .common import connect, load_config
from .kinematics import pose_error, se3_from_xyz_rpy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pose", nargs=6, type=float, metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"))
    ap.add_argument("--flange", action="store_true", help="target the flange instead of the TCP")
    args = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])

    rclpy.init()
    node = rclpy.create_node("ar4tc_goto_pose")
    cfg = load_config()
    if args.flange:
        cfg["tcp_offset"] = [0.0, 0.0, 0.0]
    mv, kin = connect(node, cfg)
    tol = cfg["tolerances"]
    x, y, z, r, p, yw = args.pose
    target = se3_from_xyz_rpy([x, y, z], np.radians([r, p, yw]))
    log = node.get_logger()

    q = mv.compute_ik(kin.tcp_to_flange(target), mv.joint_positions())
    if q is None:
        log.error("no collision-free IK solution (unreachable or blocked)")
        raise SystemExit(1)
    chk = kin.validate(q, target, tol["pos_mm"], tol["rot_deg"], tol["min_manipulability"])
    log.info(f"IK q(deg)={np.round(np.degrees(q), 2).tolist()} limits_ok={kin.within_limits(q)} "
             f"w={chk.manipulability:.4f} FK check {chk.pos_mm:.4f} mm / {chk.rot_deg:.4f} deg")
    if not chk.ok:
        log.error("IK rejected: " + "; ".join(chk.reasons))
        raise SystemExit(1)
    ok, info = mv.move_to_joints(q)
    pos, rot = pose_error(kin.fk(mv.joint_positions()), target)
    reached = ok and pos <= tol["pos_mm"] and rot <= tol["rot_deg"]
    (log.info if reached else log.error)(f"{info}; final TCP error {pos:.3f} mm / {rot:.3f} deg -> "
                                         f"{'REACHED' if reached else 'FAILED'}")
    rclpy.shutdown()
    raise SystemExit(0 if reached else 1)


if __name__ == "__main__":
    main()
