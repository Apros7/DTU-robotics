"""Direct kinematics for Part 1, Problem 1.

Edit the parameters in the block below, then run:

    .venv/bin/python assignment/part1_problem1.py

Joint angles stay symbolic. Lengths, twists, and the camera offset are the
values you set here, so changing them rebuilds T_4^0 and T_5^0.

A length can also be a symbol, for example d1 = sp.symbols("d1").
"""

import sympy as sp

# ---------------------------------------------------------------------------
# Parameters. Lengths in mm, angles in radians.
# ---------------------------------------------------------------------------

# Classical DH rows: (d, a, alpha) for joints 1..4. q1..q4 are the variables.
d1, a1, alpha1 = 50, 0, sp.pi / 2
d2, a2, alpha2 = 0, 93, 0
d3, a3, alpha3 = 0, 93, 0
d4, a4, alpha4 = 0, 50, 0

# Camera origin in the stylus frame {4}: 15 mm down x4, 45 mm along y4.
cam_x, cam_y, cam_z = -15, 45, 0

# Pose to evaluate numerically after the symbolic result. None skips it.
# Figure 1 is the arm straight up: q = (0, pi/2, 0, 0), not the DH zero.
q_check = (0, sp.pi / 2, 0, 0)

# ---------------------------------------------------------------------------

q1, q2, q3, q4 = sp.symbols("q1 q2 q3 q4", real=True)
Q = (q1, q2, q3, q4)


def dh_matrix(q, d, a, alpha):
    """Classical DH transform from frame i-1 to frame i."""
    cq, sq = sp.cos(q), sp.sin(q)
    ca, sa = sp.cos(alpha), sp.sin(alpha)
    return sp.Matrix(
        [
            [cq, -sq * ca, sq * sa, a * cq],
            [sq, cq * ca, -cq * sa, a * sq],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ]
    )


def tidy(matrix):
    return sp.simplify(sp.trigsimp(matrix))


def build_transforms():
    """Return each link transform, T_4^0, T_5^4, and T_5^0."""
    rows = (
        (q1, d1, a1, alpha1),
        (q2, d2, a2, alpha2),
        (q3, d3, a3, alpha3),
        (q4, d4, a4, alpha4),
    )
    links = [tidy(dh_matrix(*row)) for row in rows]
    t40 = tidy(links[0] * links[1] * links[2] * links[3])

    t54 = sp.eye(4)
    t54[0, 3] = cam_x
    t54[1, 3] = cam_y
    t54[2, 3] = cam_z

    t50 = tidy(t40 * t54)
    return links, t40, t54, t50


def at_pose(matrix, angles):
    subs = dict(zip(Q, angles))
    return tidy(matrix.subs(subs))


def show(title, matrix):
    print(f"\n{title}")
    print(sp.pretty(matrix, wrap_line=False))


def main():
    links, t40, t54, t50 = build_transforms()

    print("DH parameters (d, a, alpha)")
    for i, (d, a, alpha) in enumerate(
        ((d1, a1, alpha1), (d2, a2, alpha2), (d3, a3, alpha3), (d4, a4, alpha4)),
        start=1,
    ):
        print(f"  joint {i}: d={d}, a={a}, alpha={alpha}")
    print(f"Camera offset in frame 4: ({cam_x}, {cam_y}, {cam_z}) mm")

    for i, link in enumerate(links, start=1):
        show(f"T_{i}^{i - 1}", link)
    show("T_4^0 = T_1^0 T_2^1 T_3^2 T_4^3", t40)
    show("T_5^4", t54)
    show("T_5^0 = T_4^0 T_5^4", t50)

    if q_check is None:
        return

    print(f"\nEvaluated at q = {q_check}")
    show("T_4^0(q)", at_pose(t40, q_check))
    show("T_5^0(q)", at_pose(t50, q_check))


if __name__ == "__main__":
    main()
