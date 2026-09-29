# Walkthrough: Problems 1 and 2

Read this from the top. The other two notes are the compact version of the same work. This one is the thought process.

Two questions:

1. **Problem 1.** The motors are at known angles. Where is the stylus tip, and which way does the stylus point?
2. **Problem 2.** The tip must go to a known point, and the stylus must have a known steepness. Which motor angles do that?

Problem 1 builds a formula from angles to pose. Problem 2 runs that formula backwards.

---

## 1. What the drawing is

Figure 1 is the robot standing straight up. Four motors, one above the other. A stylus (a pen tip) sits at the top. A camera sits on a bracket next to it.

A **frame** is a point with three arrows glued to it. The arrows are always at right angles and follow the right-hand rule: point your right-hand fingers along red, curl them toward green, and your thumb points along blue.

| Colour | Arrow | Name |
|---|---|---|
| red | \(x\) | "forward" of that frame |
| green | \(y\) | "left" of that frame, once \(x\) and \(z\) are fixed |
| blue | \(z\) | the axis you spin around |

Frame {0} is glued to the table. It never moves. In the drawing:

- red \(x_0\) points to the right
- blue \(z_0\) points up
- green \(y_0\) points into the page, away from you

Every later position is measured in this frame. "The tip is at \((0, 0, 286)\) mm" means: not to the right, not into the page, 286 mm up from frame {0}.

The other frames ride on the moving parts:

| Frame | Glued to | In the drawing, red \(x\) points | Blue \(z\) points |
|---|---|---|---|
| {1} | just above the base motor | right | toward you |
| {2} | just above the shoulder motor | up | toward you |
| {3} | just above the elbow motor | up | toward you |
| {4} | the stylus tip | up | toward you |
| {5} | the camera | up | toward you |

Frames {2}, {3}, {4}, and {5} look identical in the drawing because the arm is straight. They are still different points, stacked along the arm.

Lengths along that stack:

| From | To | Length |
|---|---|---|
| {0} | {1} | 50 mm |
| {1} | {2} | 93 mm |
| {2} | {3} | 93 mm |
| {3} | {4} | 50 mm |

The camera {5} is 45 mm to the left of the stylus tip, and 35 mm above frame {3}. Frame {4} is 50 mm above frame {3}, so the camera is \(50 - 35 = 15\) mm below the stylus tip.

Frame {5} is bolted to the same piece as the stylus. It has no motor of its own.

---

## 2. What a pose matrix is storing

\(T_4^0\) is one 4×4 table. The name means "frame 4, described by someone sitting in frame 0".

It answers: I measured a point in stylus coordinates. Where is that point in base coordinates?

You do not have to derive the shape. You only have to be able to read one. The bottom row is always \(0\ 0\ 0\ 1\). The useful part is the top three rows:

| Column 1 | Column 2 | Column 3 | Column 4 |
|---|---|---|---|
| stylus red arrow, written in base coordinates | stylus green arrow | stylus blue arrow | stylus origin, in base coordinates |

Column 4 is the position of the tip. Column 1 is the direction the stylus points, because the stylus red arrow is \(x_4\).

A short way to write cos and sin, used everywhere below:

\[
c_1 = \cos q_1, \quad s_1 = \sin q_1, \quad c_{23} = \cos(q_2+q_3), \quad s_{234} = \sin(q_2+q_3+q_4).
\]

---

## 3. Why there are four small matrices

The stylus is four motors away from the table. Writing \(T_4^0\) in one guess is miserable. The manageable version is:

- describe how frame {1} sits relative to frame {0}
- describe how frame {2} sits relative to frame {1}
- same for {3} relative to {2}, and {4} relative to {3}

Each of those is a small matrix. Stacking them is matrix multiplication, from the base outward:

\[
T_4^0 = T_1^0 \, T_2^1 \, T_3^2 \, T_4^3.
\]

Read the product as a trip: start at the table, pass through motor 1, then motor 2, then motor 3, then motor 4. You arrive at the stylus.

The camera is the same trip plus one fixed shift, because it is bolted on:

\[
T_5^0 = T_4^0 \, T_5^4.
\]

---

## 4. The four numbers that describe one motor

Denavit–Hartenberg is a checklist. For each motor you fill in four numbers, then a fixed recipe turns those numbers into the small matrix.

| Symbol | Plain meaning | Changes when the robot moves? |
|---|---|---|
| \(q\) | how far this motor has turned | yes, this is the variable |
| \(d\) | slide along the previous blue axis before the next frame | no, it is a length from the drawing |
| \(a\) | length of the bar to the next frame, along the next red axis | no |
| \(\alpha\) | twist of the next blue axis relative to the previous one | no, it is how the part was machined |

\(q\) is the joint angle. \(\alpha\) is a different angle: it is frozen in the metal. In the script, \(\alpha_1 = \pi/2\) because motor 2's axis is perpendicular to motor 1's axis. That number stays \(\pi/2\) at every pose.

The recipe, the same for every motor, is

\[
{}^{i-1}T_i =
\begin{bmatrix}
c_i & -s_i \cos\alpha_i & s_i \sin\alpha_i & a_i c_i \\
s_i & c_i \cos\alpha_i & -c_i \sin\alpha_i & a_i s_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

You will use this once per motor and then stop looking at it. The script `part1_problem1.py` is this recipe plus the multiplication.

Filled in for our arm:

| Motor | \(q\) | \(d\) (mm) | \(a\) (mm) | \(\alpha\) | Why these constants |
|---|---|---|---|---|---|
| 1 | \(q_1\) | 50 | 0 | \(90^\circ\) | frame {1} is 50 mm straight up the base axis; the next axis points toward you, a right-angle twist |
| 2 | \(q_2\) | 0 | 93 | \(0^\circ\) | a 93 mm bar; the next axis is parallel to this one |
| 3 | \(q_3\) | 0 | 93 | \(0^\circ\) | another 93 mm bar, same kind of joint |
| 4 | \(q_4\) | 0 | 50 | \(0^\circ\) | the 50 mm stylus bar, same kind of joint |

\(a_1 = 0\) because the base axis and the shoulder axis meet. There is no sideways bar between them, only the 50 mm rise, which is \(d_1\).

---

## 5. What "zero angle" means

This is the step that makes the drawing confusing.

\(q_i = 0\) means "the red arrow of this frame is parallel to the red arrow of the previous frame". It does **not** mean "the arm is in the pose of Figure 1".

Check the drawing arrow by arrow:

- Frame {0} red points right. Frame {1} red points right. They match, so \(q_1 = 0\) in the drawing.
- Frame {1} red points right. Frame {2} red points up. That is a quarter turn, so \(q_2 = 90^\circ = \pi/2\) in the drawing.
- Frame {2}, {3}, and {4} all have red pointing up. They match each other, so \(q_3 = 0\) and \(q_4 = 0\) in the drawing.

Figure 1 is therefore

\[
q = (0,\ \pi/2,\ 0,\ 0).
\]

If you set every \(q\) to 0, frame {2}'s red arrow lines up with frame {1}'s red arrow, which points right. The arm sticks out horizontally to the right. The script checks the drawn pose separately, at the bottom, under `Evaluated at q = ...`. That block plugs in numbers. The big matrices above it still contain \(q_1, q_2, q_3, q_4\) as unknowns.

---

## 6. Motor 1, read out loud

Put motor 1's row into the recipe. \(\alpha = 90^\circ\), so \(\cos\alpha = 0\) and \(\sin\alpha = 1\). The \(a\) terms vanish because \(a_1 = 0\). You get

\[
T_1^0 =
\begin{bmatrix}
c_1 & 0 & s_1 & 0 \\
s_1 & 0 & -c_1 & 0 \\
0 & 1 & 0 & 50 \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

Now set \(q_1 = 0\), so \(c_1 = 1\) and \(s_1 = 0\):

\[
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & -1 & 0 \\
0 & 1 & 0 & 50 \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

Read the columns:

- Column 4 says frame {1} sits at \((0, 0, 50)\) mm. That is the 50 mm rise.
- Column 1 says red \(x_1\) is \((1, 0, 0)\), to the right. Same as the base. Good, because \(q_1 = 0\).
- Column 2 says green \(y_1\) is \((0, 0, 1)\), straight up.
- Column 3 says blue \(z_1\) is \((0, -1, 0)\). Base green points into the page, so negative green points toward you. The shoulder axis points toward you. Good.

When \(q_1\) is some other angle, column 4 is still \((0, 0, 50)\). Motor 1 spins the arm, and frame {1} itself stays on the axis, so its origin does not travel. The arrows in columns 1–3 turn.

---

## 7. Motors 2, 3, and 4

These three are the same kind of hinge: \(\alpha = 0\), \(d = 0\), and a bar of length \(a\). The recipe collapses to

\[
\begin{bmatrix}
c_i & -s_i & 0 & a\, c_i \\
s_i & c_i & 0 & a\, s_i \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

Motor 2 uses \(a = 93\) and angle \(q_2\). Motor 3 uses \(a = 93\) and angle \(q_3\). Motor 4 uses \(a = 50\) and angle \(q_4\).

Read one of them at angle 0, say motor 4 with \(q_4 = 0\):

\[
\begin{bmatrix}
1 & 0 & 0 & 50 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

Column 4 is \((50, 0, 0)\) in the previous frame. The bar lies along the previous red arrow, 50 mm long. Columns 1–3 are the identity, so the new arrows are parallel to the old ones. That is exactly what \(q = 0\) was defined to mean.

At a nonzero angle the bar swings in a circle in that frame's \(x\)-\(y\) plane. The blue axis is unchanged (column 3 stays \((0,0,1)\)), which is the statement "this motor spins about the same direction as the previous one".

---

## 8. Stacking them into the stylus pose

Multiply in order:

\[
T_4^0 = T_1^0 \, T_2^1 \, T_3^2 \, T_4^3.
\]

You can let SymPy do the algebra (`part1_problem1.py`). Here is the reason the result looks the way it does, so the result is not a surprise.

After motor 1, motors 2, 3, and 4 all spin about axes that point toward you. The whole rest of the arm bends in one plane, like three sticks on a sheet of paper:

1. Stick of 93 mm. Its direction on the paper is the angle \(q_2\).
2. Stick of 93 mm. The elbow adds its own turn, so this stick's direction is \(q_2 + q_3\).
3. Stick of 50 mm. The wrist adds one more turn, so this stick's direction is \(q_2 + q_3 + q_4\).

On that paper, call "right" the direction of frame {1}'s red arrow and "up" the direction of frame {1}'s green arrow. The tip, measured on the paper, is the sum of the three sticks:

\[
\begin{aligned}
\text{right} &= 93 c_2 + 93 c_{23} + 50 c_{234}, \\
\text{up} &= 93 s_2 + 93 s_{23} + 50 s_{234}.
\end{aligned}
\]

Nothing moves out of the paper. The shoulder, elbow, and wrist axes are perpendicular to the paper.

Motor 1 then spins that sheet around the vertical axis, and the sheet starts 50 mm off the table:

\[
\begin{aligned}
o_x &= c_1 \cdot (\text{right}), \\
o_y &= s_1 \cdot (\text{right}), \\
o_z &= 50 + (\text{up}).
\end{aligned}
\]

The stylus red arrow is the direction of the last stick. On the paper that direction is angle \(q_2+q_3+q_4\). After motor 1 spins the paper, the arrow in base coordinates is

\[
x_4^0 =
\begin{bmatrix}
c_1 \, c_{234} \\
s_1 \, c_{234} \\
s_{234}
\end{bmatrix}.
\]

The third entry, \(s_{234}\), is how much the stylus red arrow points upward. Horizontal stylus means this entry is 0. Straight up means it is 1. Problem 2 will use this entry and leave the other two alone.

Put the arrows and the tip into one matrix and you have \(T_4^0\):

\[
T_4^0 =
\begin{bmatrix}
c_1 c_{234} & -c_1 s_{234} & s_1 & c_1(93c_2 + 93c_{23} + 50c_{234}) \\
s_1 c_{234} & -s_1 s_{234} & -c_1 & s_1(93c_2 + 93c_{23} + 50c_{234}) \\
s_{234} & c_{234} & 0 & 50 + 93s_2 + 93s_{23} + 50s_{234} \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

Column 1 is the stylus red arrow. Column 4 is the tip. Columns 2 and 3 are the green and blue arrows. You can ignore them until a later problem asks for orientation.

---

## 9. The camera

The camera has the same arrows as the stylus. In the drawing both red arrows point up and both green arrows point left. So \(T_5^4\) has the identity in the arrow part: no extra rotation.

The camera origin, measured in stylus coordinates:

- 15 mm against the stylus red arrow. In the drawing that red arrow points up, and the camera is below the tip.
- 45 mm along the stylus green arrow. In the drawing that green arrow points left, and the camera is to the left.
- 0 mm along the blue arrow.

\[
T_5^4 =
\begin{bmatrix}
1 & 0 & 0 & -15 \\
0 & 1 & 0 & 45 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}.
\]

\(T_5^0 = T_4^0 \, T_5^4\) keeps the arrows and moves the origin by \(-15\) times column 1 plus \(45\) times column 2. The 50 mm stylus bar and the 15 mm backstep combine, which is why the camera position contains a \(35\) where the stylus position contains a \(50\):

\[
50\, c_{234} - 15\, c_{234} = 35\, c_{234}.
\]

---

## 10. Check it on the drawing

Plug in \(q_1 = 0\), \(q_2 = \pi/2\), \(q_3 = 0\), \(q_4 = 0\).

Then \(c_2 = 0\), \(s_2 = 1\), \(c_{23} = 0\), \(s_{23} = 1\), \(c_{234} = 0\), \(s_{234} = 1\).

The "right" distance on the paper is 0, because every stick is vertical. The "up" distance is \(93 + 93 + 50\). Add the 50 mm base:

\[
o_4^0 = \begin{bmatrix} 0 \\ 0 \\ 286 \end{bmatrix} \text{ mm}.
\]

\(286 = 50 + 93 + 93 + 50\). That is the four lengths in the drawing, stacked. The stylus red arrow is \((0, 0, 1)\), straight up, which is what the drawing shows.

The camera is 45 mm to the left and 15 mm below that tip:

\[
o_5^0 = \begin{bmatrix} -45 \\ 0 \\ 271 \end{bmatrix} \text{ mm}.
\]

Run the script and compare the last two printed matrices with this:

```bash
.venv/bin/python assignment/part1_problem1.py
```

The matrices named \(T_4^0\) and \(T_5^0\) are the formulas of sections 8 and 9. The two matrices under `Evaluated at q = (0, pi/2, 0, 0)` are this check. Stylus height 286, camera at \((-45, 0, 271)\).

If you change `d1`, `a2`, `a3`, or `a4` at the top of the script and run it again, those formulas are rebuilt from the recipe. The joint angles in the formulas stay symbols.

---

## 11. Problem 2 is the return trip

Problem 1 says: angles in, tip and stylus direction out.

Problem 2 says: I will give you the tip \((o_x, o_y, o_z)\), and I will give you only the upward part of the stylus red arrow,

\[
x_z = s_{234}.
\]

Give me the four angles.

Four numbers in, four angles out. The catch is that \(x_z\) is a sine. A sine does not tell you the angle uniquely, and the elbow can bend two ways. Several angle sets can put the tip in the same place with the same steepness.

Do the forward trip backwards.

### Step A. Motor 1, from the shadow of the tip

Look down from above. The tip's shadow is at \((o_x, o_y)\). Its distance from the base axis is

\[
\rho = \sqrt{o_x^2 + o_y^2}.
\]

The base motor yaw that points the arm toward that shadow is

\[
q_1 = \operatorname{atan2}(o_y, o_x).
\]

`atan2(y, x)` is the compass angle of the point \((x, y)\), with the quadrants handled. Plain `atan(y/x)` gets the quadrants wrong.

There is a second possibility. The arm can approach the same shadow from the opposite direction, with the horizontal reach counted as negative. Then

\[
q_1 = \operatorname{atan2}(o_y, o_x) + \pi, \qquad R = -\rho.
\]

Call the sign of that reach \(\delta\). \(\delta = +1\) is "reach toward the shadow". \(\delta = -1\) is "reach the shadow from the opposite side". Both are legal. The script tries both.

### Step B. How steep the stylus is

We were given \(s_{234} = x_z\). Then

\[
c_{234} = \sigma \sqrt{1 - x_z^2}, \quad \sigma = \pm 1,
\]

and the total bend of the three sticks is

\[
\phi = q_2 + q_3 + q_4 = \operatorname{atan2}(x_z,\ c_{234}).
\]

\(\sigma = +1\) and \(\sigma = -1\) are two different stylus directions that share the same upward component. The assignment allows this, because it told us to match only the last entry of the red arrow. A horizontal stylus has \(x_z = 0\), so \(\phi\) is either \(0\) or \(\pi\): pointing outward, or pointing back the way it came.

### Step C. Walk back along the last stick

The last stick is 50 mm long at angle \(\phi\). The wrist (frame {3}) is that stick subtracted from the tip.

On the paper from section 8:

\[
\begin{aligned}
u &= R - 50\, c_{234}, \\
v &= o_z - 50 - 50\, x_z.
\end{aligned}
\]

\(u\) is the horizontal distance from the shoulder axis to the wrist. \(v\) is the wrist height above the shoulder. The \(-50\) in \(v\) removes the base pedestal, so the two 93 mm sticks are measured from motor 2, where they actually start.

### Step D. Two sticks of 93 mm

You now have a point \((u, v)\) and two bars of length 93 mm. This is the standard elbow problem.

The angle inside the elbow comes from the law of cosines. Distance from shoulder to wrist, squared, equals \(93^2 + 93^2 + 2\cdot 93\cdot 93\cdot\cos q_3\), rearranged:

\[
c_3 = \frac{u^2 + v^2 - 2\cdot 93^2}{2\cdot 93^2}.
\]

If this comes out bigger than 1 or smaller than \(-1\), the wrist is farther away than the two bars can reach, or too close. That branch is impossible. Skip it.

The sine still has a sign choice, the elbow bend:

\[
s_3 = \varepsilon \sqrt{1 - c_3^2}, \quad \varepsilon = \pm 1,
\qquad
q_3 = \operatorname{atan2}(s_3,\ c_3).
\]

One sign bends the elbow one way, the other sign bends it the other way. Both put the wrist on \((u, v)\) when they are reachable.

The shoulder angle points the first bar so that the second bar lands on the wrist:

\[
q_2 = \operatorname{atan2}(v,\ u) - \operatorname{atan2}(s_3,\ 1 + c_3).
\]

The second `atan2` is half of \(q_3\). That shortcut exists because the two bars have the same length. The script keeps the version that also works if you later change `a2` and `a3` to different numbers:

\[
q_2 = \operatorname{atan2}(v,\ u) - \operatorname{atan2}(a_3 s_3,\ a_2 + a_3 c_3).
\]

### Step E. Whatever is left is the wrist motor

The three bends must add up to \(\phi\):

\[
q_4 = \phi - q_2 - q_3.
\]

Done. One combination of \((\delta, \sigma, \varepsilon)\) has produced one joint vector \((q_1, q_2, q_3, q_4)\).

---

## 12. Why the script prints several rows

There are three independent sign choices, each \(\pm 1\):

| Choice | What it decides |
|---|---|
| \(\delta\) | base motor toward the tip, or a half turn the other way |
| \(\sigma\) | which way the stylus cosine goes |
| \(\varepsilon\) | which way the elbow bends |

\(2 \times 2 \times 2 = 8\) candidates. Any candidate whose \(c_3\) is outside \([-1, 1]\) is dropped.

Run:

```bash
.venv/bin/python assignment/part1_problem2.py
```

The script takes `q_example` at the top, runs Problem 1 forward, and prints the tip and \(x_z\) that those angles produce. Then it prints every inverse branch that gets back to that same tip and the same \(x_z\).

For the default `q_example = (0.4, 0.6, -0.5, 0.3)` radians, which is about \((23^\circ,\ 34^\circ,\ -29^\circ,\ 17^\circ)\), four branches are reachable. One of the four is that same angle set. The other three are different motor angles that put the pen on the same point with the same steepness.

Change `q_example`, run again, and watch which row matches your input. That is the whole check: forward, then backward, and one of the backward answers must be the angles you started with.

---

## 13. How the files fit together

| File | Role |
|---|---|
| `kinematics_explained.md` | this walkthrough |
| `part1_problem1.md` | the same forward kinematics, written as a solution |
| `part1_problem1.py` | builds \(T_4^0\) and \(T_5^0\) from the lengths you type |
| `part1_problem2.md` | the same inverse kinematics, written as a solution |
| `part1_problem2.py` | turns a tip and a tilt back into joint angles |

Change a length in `part1_problem1.py` only. Problem 2 imports those lengths, so the inverse stays consistent with the forward map.
