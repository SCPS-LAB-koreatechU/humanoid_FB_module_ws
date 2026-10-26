"""SE(3) helpers. Lengths are whatever the caller uses (mm in the assembly, m in URDF)."""
import math

import numpy as np


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rpy_to_R(r, p, y):
    """URDF/ROS convention: R = Rz(yaw) Ry(pitch) Rx(roll), angles in rad."""
    return rot_z(y) @ rot_y(p) @ rot_x(r)


def R_to_rpy(R):
    """Inverse of rpy_to_R (rad). Handles the pitch = +-90 deg case."""
    sp = max(-1.0, min(1.0, -R[2, 0]))
    p = math.asin(sp)
    if abs(math.cos(p)) < 1e-9:
        r = 0.0
        y = math.atan2(-R[0, 1], R[1, 1])
    else:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    return r, p, y


def H(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def from_xyz_rpy_deg(xyz=(0, 0, 0), rpy_deg=(0, 0, 0)):
    r, p, y = (math.radians(v) for v in rpy_deg)
    return H(rpy_to_R(r, p, y), np.asarray(xyz, dtype=float))


def dh_standard(a, alpha, d, theta):
    """A = Rz(theta) Tz(d) Tx(a) Rx(alpha); alpha, theta in rad."""
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def axis_angle(axis, q):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
