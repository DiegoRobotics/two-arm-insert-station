"""Annin HMI 6.0 forward kinematics, re-implemented from kinematics.cpp with the DH table the HMI
stores in ARbot.cal. Gives positions in HMI coordinates (mm, robot base, tool frame = flange).
Checked against poses the HMI itself saved (test_hmi.py)."""
import numpy as np

# Modified DH (Craig): T_i = Rx(alpha) * Tx(a) * Rz(theta + q) * Tz(d); angles deg, lengths mm
THETA = [0, -90, 0, 0, 0, 180]
ALPHA = [0, -90, 0, -90, 90, -90]
A = [0, 64.2, 305, 0, 0, 0]
D = [169.77, 0, 0, 222.63, 0, 41]


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def _t(x=0.0, z=0.0):
    T = np.eye(4)
    T[0, 3], T[2, 3] = x, z
    return T


def frames(q_annin_deg):
    """Poses (4x4, mm) of DH frames 1..6 for Annin joint angles in degrees; the last one is the flange."""
    T, out = np.eye(4), []
    for i, q in enumerate(q_annin_deg):
        T = T @ _rx(np.radians(ALPHA[i])) @ _t(x=A[i]) @ _rz(np.radians(THETA[i] + q)) @ _t(z=D[i])
        out.append(T)
    return out


def fk(q_annin_deg):
    """Flange pose (4x4, mm) for Annin joint angles in degrees."""
    return frames(q_annin_deg)[-1]


def xyz(q_annin_deg):
    return fk(q_annin_deg)[:3, 3]
