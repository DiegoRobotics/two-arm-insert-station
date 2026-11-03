"""Offline kinematics tests (no MoveIt running). Needs the built upstream workspace for the URDF."""
import os
import subprocess

import numpy as np
import pinocchio as pin
import pytest

from ar4tc_toolchanger.kinematics import KinematicsValidator, pose_error, se3_from_xyz_rpy

ANNIN = {"d1": 0.16977, "a1": 0.0642, "a2": 0.305, "d4": 0.22263, "d6": 0.041}  # Annin HMI 6.0 ARbot.cal


@pytest.fixture(scope="module")
def kin():
    try:
        from ament_index_python.packages import get_package_share_directory
        xacro_file = os.path.join(get_package_share_directory("annin_ar4_moveit_config"), "urdf", "fake_ar.urdf.xacro")
        urdf = subprocess.run(["xacro", xacro_file, "ar_model:=mk3"], check=True, capture_output=True, text=True).stdout
    except Exception as e:
        pytest.skip(f"MK3 URDF unavailable: {e}")
    return KinematicsValidator(urdf, "ee_link")


def test_mk3_limits_match_annin(kin):
    deg = np.degrees
    assert np.allclose(deg(kin.lower), [-170, -42, -89, -180, -105, -180], atol=0.01)
    assert np.allclose(deg(kin.upper), [170, 90, 52, 180, 105, 180], atol=0.01)


def test_home_flange_matches_annin_link_lengths(kin):
    # At home the arm is L-shaped: J2 at height d1, upper arm vertical (a2), forearm + flange horizontal along -Y.
    p = kin.flange(np.zeros(6)).translation
    assert p[2] == pytest.approx(ANNIN["d1"] + ANNIN["a2"], abs=1e-4)
    assert -p[1] == pytest.approx(ANNIN["a1"] + ANNIN["d4"] + ANNIN["d6"], abs=1e-4)


def test_pose_error_units():
    a = se3_from_xyz_rpy([0, 0, 0], [0, 0, 0])
    b = se3_from_xyz_rpy([0.001, 0, 0], [0, 0, np.radians(1.0)])
    pos, rot = pose_error(a, b)
    assert pos == pytest.approx(1.0) and rot == pytest.approx(1.0)


def test_wrist_singularity_detected(kin):
    assert kin.manipulability([0, 0, 0, 0, 0, 0]) < 1e-6          # J5 = 0 aligns J4 and J6
    assert kin.manipulability([0, 0.3, -0.3, 0, 0.8, 0]) > 1e-4


def test_validator_rejects_limit_and_error(kin):
    q = np.array([0.2, 0.3, -0.2, 0.1, 0.7, 0.1])
    target = kin.fk(q)
    assert kin.validate(q, target).ok
    assert not kin.validate(q, target * pin.SE3(np.eye(3), np.array([0.002, 0, 0]))).ok
    assert not kin.validate(np.array([3.1, 0.3, -0.2, 0.1, 0.7, 0.1]), target).ok   # J1 beyond 170 deg


def test_tcp_flange_roundtrip():
    k = KinematicsValidator.__new__(KinematicsValidator)
    k.tcp = pin.SE3(np.eye(3), np.array([0, 0, 0.036]))
    M = se3_from_xyz_rpy([0.1, -0.2, 0.3], [0.1, 0.2, 0.3])
    assert pose_error(k.tcp_to_flange(M) * k.tcp, M)[0] < 1e-9
