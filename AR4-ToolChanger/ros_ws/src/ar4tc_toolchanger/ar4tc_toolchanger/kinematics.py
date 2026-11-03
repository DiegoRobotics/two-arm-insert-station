"""Independent kinematics check (Pinocchio). No ROS imports, so it can be unit-tested on its own.

MoveIt solves IK; this module re-computes FK from the same URDF and decides if a
solution is acceptable: joint limits, pose error and distance from singularity.
"""
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin


def se3_from_xyz_rpy(xyz, rpy):
    return pin.SE3(pin.rpy.rpyToMatrix(*rpy), np.asarray(xyz, dtype=float))


def pose_error(a: pin.SE3, b: pin.SE3):
    """Return (position error in mm, orientation error in deg) between two poses."""
    pos_mm = float(np.linalg.norm(a.translation - b.translation) * 1000.0)
    rot_deg = float(np.degrees(np.linalg.norm(pin.log3(a.rotation.T @ b.rotation))))
    return pos_mm, rot_deg


@dataclass
class Check:
    ok: bool
    pos_mm: float = 0.0
    rot_deg: float = 0.0
    manipulability: float = 0.0
    reasons: list = field(default_factory=list)


class KinematicsValidator:
    def __init__(self, urdf_xml, tip_frame="ee_link", tcp_offset=(0.0, 0.0, 0.0), n_arm=6):
        # Upstream ar4_ros_driver writes effort="-1"; Pinocchio rejects negative effort.
        self.model = pin.buildModelFromXML(urdf_xml.replace('effort="-1"', 'effort="1000"'))
        self.data = self.model.createData()
        self.frame = self.model.getFrameId(tip_frame)
        self.tcp = pin.SE3(np.eye(3), np.asarray(tcp_offset, dtype=float))
        self.n = n_arm
        self.lower = self.model.lowerPositionLimit[:n_arm].copy()
        self.upper = self.model.upperPositionLimit[:n_arm].copy()

    def _q(self, q6):
        q = pin.neutral(self.model)
        q[: self.n] = q6
        return q

    def flange(self, q6) -> pin.SE3:
        pin.framesForwardKinematics(self.model, self.data, self._q(q6))
        return pin.SE3(self.data.oMf[self.frame])

    def fk(self, q6) -> pin.SE3:
        """TCP pose in base_link."""
        return self.flange(q6) * self.tcp

    def tcp_to_flange(self, tcp_pose: pin.SE3) -> pin.SE3:
        return tcp_pose * self.tcp.inverse()

    def manipulability(self, q6) -> float:
        """Yoshikawa measure sqrt(det(J J^T)); approaches 0 at a singularity."""
        q = self._q(q6)
        J = pin.computeFrameJacobian(self.model, self.data, q, self.frame, pin.LOCAL_WORLD_ALIGNED)[:, : self.n]
        return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

    def within_limits(self, q6, margin=1e-6) -> bool:
        q6 = np.asarray(q6)
        return bool(np.all(q6 >= self.lower - margin) and np.all(q6 <= self.upper + margin))

    def validate(self, q6, target: pin.SE3, tol_mm=1.0, tol_deg=0.5, min_manip=1e-4) -> Check:
        c = Check(ok=True)
        if not self.within_limits(q6):
            c.reasons.append("joint limit")
        c.pos_mm, c.rot_deg = pose_error(self.fk(q6), target)
        if c.pos_mm > tol_mm:
            c.reasons.append(f"position error {c.pos_mm:.3f} mm > {tol_mm}")
        if c.rot_deg > tol_deg:
            c.reasons.append(f"orientation error {c.rot_deg:.3f} deg > {tol_deg}")
        c.manipulability = self.manipulability(q6)
        if c.manipulability < min_manip:
            c.reasons.append(f"near singularity (w={c.manipulability:.2e})")
        c.ok = not c.reasons
        return c
