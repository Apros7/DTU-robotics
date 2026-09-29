# Part 1, Problem 1 — Direct kinematics

Find \(T_4^0(q)\) for the stylus and \(T_5^0(q)\) for the camera.

Notation: \(T_4^0\) means “pose of frame 4, written in frame 0”. Same object as \({}^{0}T_4\). A point written in frame 4 is mapped to frame 0 by

\[
\begin{bmatrix} p^0 \\ 1 \end{bmatrix}
=
T_4^0
\begin{bmatrix} p^4 \\ 1 \end{bmatrix}.
\]

The first three columns of the rotation block are the \(x\), \(y\), and \(z\) axes of the child frame, expressed in the parent. The last column is the origin of the child frame in the parent.

Neither \(T_4^0\) nor \(T_5^0\) is written down in one step. Each joint gets its own 4×4 matrix from the Denavit–Hartenberg (DH) table. The full transform is the product of those matrices.

---

## 1. Frames in Figure 1

Colours: red = \(x\), green = \(y\), blue = \(z\).

In the drawn pose the arm is straight up:

| Frame | \(x\) | \(y\) | \(z\) |
|---|---|---|---|
| {0} | right | into the page | up |
| {1} | right | up | toward you |
| {2}, {3}, {4}, {5} | up | left | toward you |

Lengths read off the figure, in that same pose:

| From → to | Distance |
|---|---|
| {0} → {1} | 50 mm, straight up |
| {1} → {2} | 93 mm, straight up |
| {2} → {3} | 93 mm, straight up |
| {3} → {4} | 50 mm, straight up |
| {4} → {5} | 45 mm to the left, and 15 mm down |

The 15 mm is not labelled on its own. The figure labels 50 mm from {3} to {4} and 35 mm from {3} up to {5}, so the camera sits \(50 - 35 = 15\) mm below the stylus tip. Frame {5} is the camera. It does **not** follow DH. It is bolted to the same link as the stylus, so \(T_5^4\) is a constant.

Joint axes (the \(z\) axis of the previous frame):

- Joint 1 rotates about \(z_0\) (vertical).
- Joints 2, 3, and 4 rotate about parallel axes pointing toward you.

---

## 2. DH table

Classical DH. For joint \(i\):

- \(z_{i-1}\) lies on the joint axis.
- \(x_i\) is the common normal from \(z_{i-1}\) to \(z_i\).
- \(q_i = 0\) when \(x_{i-1}\) is parallel to \(x_i\) and points the same way.

| \(i\) | \(q_i\) | \(d_i\) (mm) | \(a_i\) (mm) | \(\alpha_i\) |
|---|---|---|---|---|
| 1 | \(q_1\) | 50 | 0 | \(\pi/2\) |
| 2 | \(q_2\) | 0 | 93 | 0 |
| 3 | \(q_3\) | 0 | 93 | 0 |
| 4 | \(q_4\) | 0 | 50 | 0 |

Where the numbers come from:

- \(d_1 = 50\): slide 50 mm up \(z_0\) from {0} to {1}. The two axes meet there, so the link length \(a_1 = 0\).
- \(\alpha_1 = \pi/2\): at {1}, \(z_0\) points up and \(z_1\) points toward you. That is a \(+90^\circ\) rotation about \(x_1\).
- \(a_2 = a_3 = 93\), \(a_4 = 50\): those are the common normals, the link lengths along \(x_2\), \(x_3\), \(x_4\).
- \(\alpha_2 = \alpha_3 = \alpha_4 = 0\): joints 2, 3, and 4 are parallel and point the same way.
- \(d_2 = d_3 = d_4 = 0\): no shift along those joint axes between the frames.

Figure 1 is **not** the all-zero angle pose. In the drawing, \(x_0\) and \(x_1\) both point right, so \(q_1 = 0\). But \(x_1\) points right and \(x_2\) points up, which is \(+90^\circ\) about \(z_1\). Then \(x_2\), \(x_3\), and \(x_4\) all point up, so \(q_3 = q_4 = 0\). The drawn pose is

\[
q_{\text{figure}} = \begin{bmatrix} 0 & \pi/2 & 0 & 0 \end{bmatrix}^\top.
\]

At \(q = 0\) the arm sticks out horizontally to the right.

---

## 3. One matrix per joint

The standard DH matrix from frame \(i-1\) to frame \(i\) is

\[
{}^{i-1}T_i(q_i) =
\begin{bmatrix}
\cos q_i & -\sin q_i \cos\alpha_i & \sin q_i \sin\alpha_i & a_i \cos q_i \\
\sin q_i & \cos q_i \cos\alpha_i & -\cos q_i \sin\alpha_i & a_i \sin q_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

It does four things, in order: rotate by \(q_i\) about \(z_{i-1}\), slide \(d_i\) along that axis, slide \(a_i\) along the new \(x\), rotate by \(\alpha_i\) about that \(x\).

Plug in row 1 of the table (\(\alpha_1 = \pi/2\), so \(\cos\alpha_1 = 0\), \(\sin\alpha_1 = 1\)):

\[
T_1^0 =
\begin{bmatrix}
c_1 & 0 & s_1 & 0 \\
s_1 & 0 & -c_1 & 0 \\
0 & 1 & 0 & 50 \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

Rows 2–4 all have \(\alpha = 0\) and \(d = 0\), so they share one shape. For a link of length \(a\):

\[
{}^{i-1}T_i =
\begin{bmatrix}
c_i & -s_i & 0 & a\, c_i \\
s_i & c_i & 0 & a\, s_i \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

So \(T_2^1\) uses \(a = 93\) and angle \(q_2\), \(T_3^2\) uses \(a = 93\) and angle \(q_3\), and \(T_4^3\) uses \(a = 50\) and angle \(q_4\).

---

## 4. How \(T_4^0\) is built

\[
T_4^0 = T_1^0 \, T_2^1 \, T_3^2 \, T_4^3.
\]

Read the product from the base outward: start in frame 0, apply joint 1, then joint 2, then joint 3, then joint 4.

Joints 2, 3, and 4 are parallel, so this product collapses. In frame {1} the arm is planar. Each link adds a step whose direction is the sum of the joint angles so far:

- 93 mm at angle \(q_2\)
- 93 mm at angle \(q_2 + q_3\)
- 50 mm at angle \(q_2 + q_3 + q_4\)

Write \(c_{23} = \cos(q_2+q_3)\), \(s_{234} = \sin(q_2+q_3+q_4)\), and the same pattern for the other sums. In frame {1} the stylus tip is

\[
\begin{aligned}
x_1 &= 93 c_2 + 93 c_{23} + 50 c_{234}, \\
y_1 &= 93 s_2 + 93 s_{23} + 50 s_{234}, \\
z_1 &= 0.
\end{aligned}
\]

\(x_1\) is “right” and \(y_1\) is “up” when \(q_1 = 0\). Nothing moves along \(z_1\), because that axis points out of the bending plane.

Joint 1 then spins that planar result about the vertical axis and the base height adds 50 mm:

\[
\begin{aligned}
o_4^0
&=
\begin{bmatrix}
c_1 \, x_1 \\
s_1 \, x_1 \\
50 + y_1
\end{bmatrix}
=
\begin{bmatrix}
c_1 (93 c_2 + 93 c_{23} + 50 c_{234}) \\
s_1 (93 c_2 + 93 c_{23} + 50 c_{234}) \\
50 + 93 s_2 + 93 s_{23} + 50 s_{234}
\end{bmatrix}.
\end{aligned}
\]

The stylus axes come from the same product. After joint 1, the bending joints only rotate inside the arm plane, so the stylus \(x\)-axis is “horizontal in the arm direction, then tilted by \(q_2+q_3+q_4\)”:

\[
T_4^0 =
\begin{bmatrix}
c_1 c_{234} & -c_1 s_{234} & s_1 & c_1 (93 c_2 + 93 c_{23} + 50 c_{234}) \\
s_1 c_{234} & -s_1 s_{234} & -c_1 & s_1 (93 c_2 + 93 c_{23} + 50 c_{234}) \\
s_{234} & c_{234} & 0 & 50 + 93 s_2 + 93 s_{23} + 50 s_{234} \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

Column by column, in frame 0:

- column 1: stylus \(x\)-axis
- column 2: stylus \(y\)-axis
- column 3: stylus \(z\)-axis
- column 4: stylus tip position

The third entry of column 1 is \(s_{234}\). That is the vertical part of the stylus \(x\)-axis. Problem 3 keeps the stylus horizontal by making this entry 0.

---

## 5. How \(T_5^0\) is built

The camera is fixed on the stylus link, so its pose is the stylus pose times one constant matrix:

\[
T_5^0 = T_4^0 \, T_5^4.
\]

\(T_5^4\) is read from Figure 1, in stylus coordinates. In the drawing, frame {4} and frame {5} have the same orientation, and the camera origin is

- 15 mm along \(-x_4\) (15 mm down, and \(x_4\) points up),
- 45 mm along \(+y_4\) (45 mm to the left, and \(y_4\) points left),
- 0 mm along \(z_4\).

\[
T_5^4 =
\begin{bmatrix}
1 & 0 & 0 & -15 \\
0 & 1 & 0 & 45 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\quad \text{(mm)}.
\]

Multiplying by \(T_4^0\) leaves the rotation unchanged (the camera axes equal the stylus axes) and moves the origin:

\[
o_5^0 = o_4^0 - 15\, x_4 + 45\, y_4.
\]

Substitute the columns of \(T_4^0\):

\[
o_5^0 =
\begin{bmatrix}
c_1 (93 c_2 + 93 c_{23} + 35 c_{234} - 45 s_{234}) \\
s_1 (93 c_2 + 93 c_{23} + 35 c_{234} - 45 s_{234}) \\
50 + 93 s_2 + 93 s_{23} + 35 s_{234} + 45 c_{234}
\end{bmatrix}.
\]

The \(35\) appears because the 50 mm stylus link and the \(-15\) mm camera offset along \(x_4\) combine: \(50 c_{234} - 15 c_{234} = 35 c_{234}\). Same for the sine terms. The \(-45 s_{234}\) and \(+45 c_{234}\) terms are the 45 mm offset along \(y_4\).

So

\[
T_5^0 =
\begin{bmatrix}
c_1 c_{234} & -c_1 s_{234} & s_1 & c_1 (93 c_2 + 93 c_{23} + 35 c_{234} - 45 s_{234}) \\
s_1 c_{234} & -s_1 s_{234} & -c_1 & s_1 (93 c_2 + 93 c_{23} + 35 c_{234} - 45 s_{234}) \\
s_{234} & c_{234} & 0 & 50 + 93 s_2 + 93 s_{23} + 35 s_{234} + 45 c_{234} \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

---

## 6. Check on Figure 1

Set \(q_1 = 0\), \(q_2 = \pi/2\), \(q_3 = 0\), \(q_4 = 0\). Then \(c_2 = 0\), \(s_2 = 1\), \(c_{234} = 0\), \(s_{234} = 1\).

Stylus tip:

\[
o_4^0 = \begin{bmatrix} 0 \\ 0 \\ 50 + 93 + 93 + 50 \end{bmatrix}
= \begin{bmatrix} 0 \\ 0 \\ 286 \end{bmatrix}\ \text{mm}.
\]

That is the four vertical lengths stacked. The stylus \(x\)-axis is \((0,0,1)\), straight up, which is what the red arrow on {4} does in the figure.

Camera:

\[
o_5^0 = \begin{bmatrix} -45 \\ 0 \\ 271 \end{bmatrix}\ \text{mm},
\]

which is 45 mm to the left of the stylus and \(286 - 15 = 271\) mm up. That matches the 45 mm and 35 mm dimensions.
