# Part 1, Problem 2 — Inverse kinematics

Find

\[
q = \begin{bmatrix} q_1 & q_2 & q_3 & q_4 \end{bmatrix}^\top
= f(x_4^0, o_4^0)
\]

such that the stylus reaches position \(o_4^0\) and the last entry of \(x_4^0\) matches. The first two entries of \(x_4^0\) are left free.

\(x_4^0\) is the first column of \(T_4^0\) (the stylus \(x\)-axis in the base). \(o_4^0\) is the last column (the stylus tip). From Problem 1, with \(c_{23} = \cos(q_2+q_3)\) and \(c_{234} = \cos(q_2+q_3+q_4)\),

\[
x_4^0
=
\begin{bmatrix}
c_1 c_{234} \\
s_1 c_{234} \\
s_{234}
\end{bmatrix},
\qquad
o_4^0
=
\begin{bmatrix}
c_1 R \\
s_1 R \\
50 + 93 s_2 + 93 s_{23} + 50 s_{234}
\end{bmatrix},
\]

\[
R = 93 c_2 + 93 c_{23} + 50 c_{234}.
\]

The four numbers we are allowed to use are \(o_x, o_y, o_z\) and

\[
x_z = s_{234}.
\]

\(c_{234}\) is not given, only its sine, so each pose has several joint solutions. The script `part1_problem2.py` evaluates them. Lengths below are the Figure 1 values; the same steps work with the parameters in `part1_problem1.py`.

---

## 1. Base joint, \(q_1\)

The horizontal distance from the base axis to the stylus tip is

\[
\rho = \sqrt{o_x^2 + o_y^2}.
\]

In the forward map, \(o_x = c_1 R\) and \(o_y = s_1 R\). The sign of \(R\) is not fixed by \(\rho\), so there are two base angles:

\[
q_1 = \operatorname{atan2}(o_y, o_x) +
\begin{cases}
0 & \text{if } \delta = +1, \\
\pi & \text{if } \delta = -1,
\end{cases}
\qquad
R = \delta \, \rho,
\quad \delta = \pm 1.
\]

\(\delta = +1\) is the arm reaching out toward \((o_x, o_y)\). \(\delta = -1\) is the same tip reached with the arm pointing the opposite way and \(R\) negative. When \(\rho = 0\) the tip is on the base axis and \(q_1\) does not change the position.

---

## 2. Stylus tilt, \(q_2+q_3+q_4\)

The constraint on \(x_4^0\) is only

\[
s_{234} = x_z, \qquad |x_z| \le 1.
\]

The cosine is then a choice of sign:

\[
c_{234} = \sigma \sqrt{1 - x_z^2}, \quad \sigma = \pm 1,
\]

\[
\phi = q_2 + q_3 + q_4 = \operatorname{atan2}(x_z,\ c_{234}).
\]

\(\sigma = +1\) and \(\sigma = -1\) tilt the stylus by supplementary angles. Both keep the same \(x_z\). They differ in the first two entries of \(x_4^0\), which this problem does not constrain. For a horizontal stylus, \(x_z = 0\), so \(\phi = 0\) or \(\phi = \pi\).

---

## 3. Take off the last link

Frame {3} is the wrist joint, one 50 mm link inward from the tip, at angle \(\phi\) in the arm plane. Its in-plane coordinates \((u, v)\) are

\[
\begin{aligned}
u &= R - 50\, c_{234}, \\
v &= o_z - 50 - 50\, x_z.
\end{aligned}
\]

\(u\) is horizontal, away from the base axis. \(v\) is the height above joint 2. The 50 mm subtracted from \(o_z\) is the base offset \(d_1\). What remains is a planar arm with two links of 93 mm.

---

## 4. Elbow, \(q_2\) and \(q_3\)

For links \(a_2 = a_3 = 93\),

\[
u = 93 c_2 + 93 c_{23}, \qquad v = 93 s_2 + 93 s_{23}.
\]

Square and add:

\[
u^2 + v^2 = 93^2 + 93^2 + 2 \cdot 93 \cdot 93 \, c_3,
\]

\[
c_3 = \frac{u^2 + v^2 - 2 \cdot 93^2}{2 \cdot 93^2}.
\]

The point is reachable when \(|c_3| \le 1\). The sine is the elbow branch:

\[
s_3 = \varepsilon \sqrt{1 - c_3^2}, \quad \varepsilon = \pm 1,
\qquad
q_3 = \operatorname{atan2}(s_3,\ c_3).
\]

\(\varepsilon = +1\) and \(\varepsilon = -1\) are the two ways to bend the elbow to the same wrist point. Then

\[
q_2 = \operatorname{atan2}(v,\ u) - \operatorname{atan2}(s_3,\ 1 + c_3).
\]

The second term is \(q_3/2\), because the two links have the same length. With unequal lengths \(a_2, a_3\) it would be \(\operatorname{atan2}(a_3 s_3,\ a_2 + a_3 c_3)\).

---

## 5. Wrist, \(q_4\)

\(\phi\) is already known, and \(q_2\), \(q_3\) are known, so

\[
q_4 = \phi - q_2 - q_3.
\]

---

## Solutions

Each combination of

\[
\delta = \pm 1, \qquad \sigma = \pm 1, \qquad \varepsilon = \pm 1
\]

is a candidate. That is at most 8 joint vectors. Drop any candidate with \(|x_z| > 1\) or \(|c_3| > 1\).

Running `part1_problem2.py` picks a joint pose, computes \(o_4^0\) and \(x_z\) from \(T_4^0\), and lists the inverse solutions. One of them is the pose that was put in. The others reach the same tip with the same stylus tilt \(x_z\), on a different branch.
