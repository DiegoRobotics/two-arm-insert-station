"""Protocol tests against the formats in AR4_teensy41_sketch_v6.0.ino (no hardware, no ROS)."""
import numpy as np
import pytest

from ar4tc_toolchanger import annin_protocol as ap
from ar4tc_toolchanger.annin_emulator import parse_rj

HMI_LINE = "A0.000B-41.832C-4.284D0.000E89.981F0.000G44.668H0.000I529.075J0.000K133.865L-0.000M0NOP0.00Q0.00R0.00"


def test_parse_position_line():
    r = ap.parse_reply(HMI_LINE + "\r\n")
    assert r.ok and not r.speed_violation and r.flag == ""
    assert np.allclose(r.joints_deg, [0, -41.832, -4.284, 0, 89.981, 0])
    assert r.xyzuvw[:3] == (44.668, 0.0, 529.075)


@pytest.mark.parametrize("line,expect", [
    ("EL010000000", "EL010000000"),          # axis limit fault, J2
    ("ER", "ER"),                            # kinematic error
    ("", "<empty>"),
])
def test_alarm_lines(line, expect):
    r = ap.parse_reply(line)
    assert not r.ok and r.alarm == expect


def test_estop_and_collision_flags():
    assert ap.parse_reply(HMI_LINE.replace("NOP", "NOEBP")).estop
    r = ap.parse_reply(HMI_LINE.replace("NOP", "NOEC001000P"))
    assert r.collision and not r.ok
    assert ap.parse_reply(HMI_LINE.replace("NOP", "NOEC000000P")).ok   # EC with no joint set = no fault


def test_rj_format_matches_firmware_parser():
    q = [12.5, -30.25, 40.0, -170.0, 95.5, 179.0]
    cmd = ap.format_rj(q, 1.5)
    assert cmd.startswith("RJA12.500B-30.250C40.000D-170.000E95.500F179.000J70.000J80.000J90.000Ss1.50Ac20")
    assert cmd.endswith("WFLm000000\n")
    target, stype, sval = parse_rj(cmd[2:].strip())   # emulator uses the sketch's indexOf logic
    assert np.allclose(target, q) and stype == "s" and sval == 1.5


def test_joint_mapping_j1_inverted():
    q = np.radians([10, 20, -30, 40, 50, 60])
    deg = ap.ros_to_annin(q)
    assert np.allclose(deg, [-10, 20, -30, 40, 50, 60])
    assert np.allclose(ap.annin_to_ros(deg), q)


def test_limits():
    assert ap.within_limits([0, -42, 52, 0, 0, 0])
    assert not ap.within_limits([0, -42.1, 0, 0, 0, 0])
    assert not ap.within_limits([0, -41.5, 0, 0, 0, 0], margin_deg=1.0)


def test_simplify_keeps_shape_within_tolerance():
    t = np.linspace(0, 2, 41)
    pts = np.stack([np.linspace(0, 40, 41), 5 * np.sin(np.linspace(0, np.pi, 41))] + [np.zeros(41)] * 4, axis=1)
    ts, kept = ap.simplify(list(t), pts, 0.2)
    assert len(kept) < len(pts) and np.allclose(kept[0], pts[0]) and np.allclose(kept[-1], pts[-1])
    for k in range(len(pts)):           # every original point is within tol of the kept polyline (per joint)
        j = np.searchsorted(ts, t[k], side="right") - 1
        j = min(j, len(ts) - 2)
        a = (t[k] - ts[j]) / (ts[j + 1] - ts[j])
        assert np.max(np.abs(pts[k] - (kept[j] + a * (kept[j + 1] - kept[j])))) < 0.6  # time-param check is looser
    ts2, kept2 = ap.simplify([0, 1, 2], np.array([[0] * 6, [1] * 6, [2] * 6], float), 0.01)
    assert len(kept2) == 2                # collinear -> endpoints only


def test_segment_speed_cap():
    secs = ap.segment_durations([0.0, 0.5], np.array([[0] * 6, [30, 0, 0, 0, 0, 0]], float), 20.0, 0.4)
    assert secs == [pytest.approx(1.5)]   # 30 deg at 20 deg/s, slower than the planned 0.5 s
