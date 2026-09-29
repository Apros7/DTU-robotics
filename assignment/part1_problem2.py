"""Inverse kinematics for Part 1, Problem 2.

Edit q_example below, then run:

    .venv/bin/python assignment/part1_problem2.py

The script runs the Problem 1 forward map on that joint pose, reads off the
stylus position o and the tilt x_z, and solves every inverse branch.

Lengths and twists come from part1_problem1.py. Change them there.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import part1_problem1 as fk

# Joint pose used to build a target (o, x_z). Radians.
q_example = (0.4, 0.6, -0.5, 0.3)


def forward_stylus(q):
    """T_4^0 at a numeric joint vector, using the DH parameters of Problem 1."""
    rows = (
        (q[0], fk.d1, fk.a1, fk.alpha1),
        (q[1], fk.d2, fk.a2, fk.alpha2),
        (q[2], fk.d3, fk.a3, fk.alpha3),
        (q[3], fk.d4, fk.a4, fk.alpha4),
    )
    T = fk.sp.eye(4)
    for row in rows:
        T *= fk.dh_matrix(*row)
    return T


def inverse_branches(ox, oy, oz, xz):
    """All real solutions of Problem 2 for one stylus specification.

    ox, oy, oz: stylus tip in the base, mm.
    xz: last component of x_4^0, which is sin(q2+q3+q4).

    Each solution is (delta, sigma, epsilon, (q1, q2, q3, q4)).
    delta is the sign of the horizontal reach R.
    sigma is the sign of cos(q2+q3+q4).
    epsilon is the sign of sin(q3).
    """
    d1 = float(fk.d1)
    a2 = float(fk.a2)
    a3 = float(fk.a3)
    a4 = float(fk.a4)

    if xz * xz > 1.0 + 1e-9:
        return []

    rho = math.hypot(ox, oy)
    c_phi_mag = math.sqrt(max(0.0, 1.0 - xz * xz))
    solutions = []

    for delta in (1, -1):
        q1 = math.atan2(oy, ox) + (0.0 if delta > 0 else math.pi)
        reach = delta * rho
        for sigma in (1, -1):
            c_phi = sigma * c_phi_mag
            u = reach - a4 * c_phi
            v = (oz - d1) - a4 * xz
            c3 = (u * u + v * v - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
            if abs(c3) > 1.0 + 1e-8:
                continue
            c3 = max(-1.0, min(1.0, c3))
            phi = math.atan2(xz, c_phi)
            for epsilon in (1, -1):
                s3 = epsilon * math.sqrt(max(0.0, 1.0 - c3 * c3))
                q3 = math.atan2(s3, c3)
                q2 = math.atan2(v, u) - math.atan2(a3 * s3, a2 + a3 * c3)
                q4 = phi - q2 - q3
                solutions.append((delta, sigma, epsilon, (q1, q2, q3, q4)))
    return solutions


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def deg(angle):
    return math.degrees(wrap(angle))


def main():
    T = forward_stylus(q_example)
    ox, oy, oz = (float(T[i, 3]) for i in range(3))
    xz = float(T[2, 0])

    print("DH lengths from part1_problem1.py (mm)")
    print(f"  d1={fk.d1}, a2={fk.a2}, a3={fk.a3}, a4={fk.a4}")
    print(f"\nExample joints q (rad) = {tuple(round(v, 4) for v in q_example)}")
    print(f"Forward kinematics: o = ({ox:.2f}, {oy:.2f}, {oz:.2f}) mm, x_z = {xz:.4f}")

    print("\nInverse solutions (branches that reach this o and x_z)")
    print(f"{'delta':>6} {'sigma':>6} {'eps':>6} {'q1 deg':>10} {'q2 deg':>10} {'q3 deg':>10} {'q4 deg':>10} {'tip err mm':>12}")
    for delta, sigma, epsilon, q in inverse_branches(ox, oy, oz, xz):
        T_sol = forward_stylus(q)
        tip_err = max(abs(float(T_sol[i, 3]) - p) for i, p in enumerate((ox, oy, oz)))
        tilt_err = abs(float(T_sol[2, 0]) - xz)
        err = max(tip_err, tilt_err)
        angles = " ".join(f"{deg(a):10.2f}" for a in q)
        print(f"{delta:6d} {sigma:6d} {epsilon:6d} {angles} {err:12.2e}")


if __name__ == "__main__":
    main()
