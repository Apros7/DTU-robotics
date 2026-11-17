"""Robot model: Problems 1, 2 and 6 from solutions_pratyush.ipynb as plain numpy functions."""
import numpy as np

d1, a2, a3, a4 = 50.0, 93.0, 93.0, 50.0        # [mm] measure the real robot (Hint 1)


def dh_transformation(theta, d, a, alpha):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1]])


def fk(q):
    """T_4^0 for joint angles q [rad] (Problem 1)."""
    return (dh_transformation(q[0], d1, 0, np.pi / 2) @ dh_transformation(q[1] + np.pi / 2, 0, a2, 0)
            @ dh_transformation(q[2], 0, a3, 0) @ dh_transformation(q[3], 0, a4, 0))


def inverse_kinematics(o, r31, elbow_up=True):
    """Problem 2. o = stylus tip o_4^0 [mm], r31 = z-component of x_4^0 (0 = horizontal, -1 = straight down)."""
    o_x, o_y, o_z = o
    th1 = np.arctan2(o_y, o_x)
    w = np.sqrt(1 - r31**2)
    x_c = o_x - a4 * np.cos(th1) * w             # wrist centre
    y_c = o_y - a4 * np.sin(th1) * w
    z_c = o_z - a4 * r31
    r, s = np.hypot(x_c, y_c), z_c - d1
    c3 = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
    if abs(c3) > 1:
        raise ValueError(f"Point {np.round(o, 1)} is out of reach")
    s3 = -np.sqrt(1 - c3**2) if elbow_up else np.sqrt(1 - c3**2)
    th3 = np.arctan2(s3, c3)
    th2 = np.arctan2(-r, s) - np.arctan2(a3 * s3, a2 + a3 * c3)
    th4 = np.arctan2(-w, r31) - th2 - th3
    return (np.array([th1, th2, th3, th4]) + np.pi) % (2 * np.pi) - np.pi


def quintic_coeffs(q0, qf, v0, vf, a0, af, T):
    """Problem 6: [c0..c5] with q(t) = c0 + c1 t + ... + c5 t^5."""
    M = np.array([[1, 0, 0,    0,      0,       0],
                  [0, 1, 0,    0,      0,       0],
                  [0, 0, 2,    0,      0,       0],
                  [1, T, T**2, T**3,   T**4,    T**5],
                  [0, 1, 2*T,  3*T**2, 4*T**3,  5*T**4],
                  [0, 0, 2,    6*T,    12*T**2, 20*T**3]])
    return np.linalg.solve(M, [q0, v0, a0, qf, vf, af])