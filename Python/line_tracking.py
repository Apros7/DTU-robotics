"""Problem 11c: track a black line on an A4 page with the stylus tip (look, then move).

  python line_tracking.py check    kinematics check: does the model match the motors?
  python line_tracking.py calib    extrinsics: touch a dot, then look at it -> camera_extrinsics.npz
  python line_tracking.py scan     look at the page and plot the path (no drawing)
  python line_tracking.py run      scan + trace the line
"""
import sys
import time
from pathlib import Path
import numpy as np
import cv2
import matplotlib.pyplot as plt
import vision
from kinematics import fk, inverse_kinematics, quintic_coeffs

sys.path.insert(0, str(Path(__file__).parents[1] / "DynamixelSDK/python/src"))   # MyRobot.py looks in the wrong folder
from MyRobot import MyRobot

JOINT_SIGN = np.array([1, 1, 1, 1])         # model angle -> motor angle, flip a joint with -1
SCAN_X, SCAN_Y = (100, 200), (-90, 90)      # [mm] area of the page that holds the line (inside the paper!)
LOOK_XY = [(x, y) for x in (125, 175) for y in (-60, 0, 60)]   # camera positions for the pictures (Hint 2)
LOOK_Z = 90.0                               # tip height [mm, base frame] while the camera looks straight down
PITCH = 60.0                                # stylus angle below horizontal while drawing [deg]
PRESS, LIFT = 1.0, 20.0                     # [mm] tip below the paper while drawing / above it before and after
SPEED, RATE = 15.0, 25.0                    # peak tip speed [mm/s], set-points per second
EXTRINSICS = Path(__file__).with_name("camera_extrinsics.npz")
R31 = -np.sin(np.radians(PITCH))            # x_4z for the drawing pose


def move(robot, q, wait=True):
    robot.move_j(*(JOINT_SIGN * np.degrees(q)), wait=wait)


def read(robot):
    return JOINT_SIGN * np.radians(robot.read_joint_angles())


def look(robot, cap, xy):
    """Camera straight down above xy (it sits 45 mm further out than the tip). Returns picture + measured q."""
    r = np.hypot(*xy)
    move(robot, inverse_kinematics([*(np.array(xy) * (r - 45) / r), LOOK_Z], -1.0))
    time.sleep(0.5)
    return vision.grab(cap), read(robot)


def check(robot):
    move(robot, inverse_kinematics([150, 32, 120], 0.0))        # Problem 3, phi = 0
    print("tip should be at (150, 32, 120) mm, stylus horizontal. FK of measured angles:", fk(read(robot))[:3, 3].round(1))
    input("Measure it, then Enter ")


def calibrate(robot, cap):
    """Table height, camera roll and camera offset from one black dot on white paper."""
    input("HOLD THE ARM, then Enter (motors go limp) ")
    for i in robot.motor_ids:
        robot._write1(i, robot.ADDR_MX_TORQUE_ENABLE, 0)
    input("Put the stylus tip on the centre of the dot, then Enter ")
    dot = fk(read(robot))[:3, 3]                                # FK of the measured angles
    input("Lift the tip off the paper, then Enter (motors come back on) ")
    cal = dict(np.load(vision.INTRINSICS), offset=np.array([-15.0, 45.0, 0.0]), z_table=dot[2])
    img, q = look(robot, cap, dot[:2] + 20)                     # dot off-centre in the picture
    uv = vision.dot_centre(vision.segment(img))
    err = {}
    for roll in (0, 90, 180, 270):                              # how is the camera turned about its optical axis?
        err[roll] = dot[:2] - vision.pixel_to_table(uv, q, dict(cal, roll=roll))[0, :2]
    cal["roll"] = min(err, key=lambda k: np.linalg.norm(err[k]))
    cal["offset"][1:] += (fk(q)[:3, :3].T @ [*err[cal["roll"]], 0])[1:]   # the rest is sideways camera offset
    print("dot error per roll [mm]:", {k: round(float(np.linalg.norm(e)), 1) for k, e in err.items()})
    print(f"dot at {dot.round(1)}, roll = {cal['roll']}, camera offset in frame 4 = {cal['offset'].round(1)}")
    np.savez(EXTRINSICS, roll=cal["roll"], offset=cal["offset"], z_table=cal["z_table"])


def scan(robot, cap, cal):
    """Back-project the line pixels of every picture into one top-down map (1 cell = 1 mm) -> path [mm]."""
    ink = np.zeros((SCAN_Y[1] - SCAN_Y[0], SCAN_X[1] - SCAN_X[0]), bool)
    for k, xy in enumerate(LOOK_XY):
        img, q = look(robot, cap, xy)
        cv2.imwrite(f"view{k}.png", img)
        uv = np.argwhere(vision.segment(img))[:, ::-1]              # (u, v) of the dark pixels
        if len(uv):
            P = vision.pixel_to_table(uv, q, cal)
            ink |= np.histogram2d(P[:, 1], P[:, 0], bins=ink.shape, range=[SCAN_Y, SCAN_X])[0] > 0
    return vision.line_path(ink) + [SCAN_X[0] + 0.5, SCAN_Y[0] + 0.5], ink


def plan(path, cal):
    """Task-space trajectory: quintic time scaling of the arc length (Problem 6), IK at every sample."""
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]     # arc length at each path point
    T = 1.875 * s[-1] / SPEED                                                   # quintic peak speed = 1.875 L / T
    t = np.arange(0, T + 1 / RATE, 1 / RATE)
    st = np.polyval(quintic_coeffs(0, s[-1], 0, 0, 0, 0, T)[::-1], t)           # s(t): rest to rest
    X = np.column_stack([np.interp(st, s, path[:, 0]), np.interp(st, s, path[:, 1]),
                         np.full(len(t), cal["z_table"] - PRESS)])
    return X, np.array([inverse_kinematics(x, R31) for x in X])


def trace(robot, X, Q):
    """Stream the joint set-points at RATE Hz. Returns the actual tip path (FK of the measured angles)."""
    lo, hi = robot.joint_limits.T
    if ((JOINT_SIGN * np.degrees(Q) < lo) | (JOINT_SIGN * np.degrees(Q) > hi)).any():
        raise ValueError("the path needs joint angles outside the limits")
    move(robot, inverse_kinematics(X[0] + [0, 0, LIFT], R31))
    move(robot, Q[0])
    actual, t0 = [], time.perf_counter()
    for k, q in enumerate(Q):
        move(robot, q, wait=False)
        actual.append(fk(read(robot))[:3, 3])
        time.sleep(max(0, t0 + (k + 1) / RATE - time.perf_counter()))
    move(robot, inverse_kinematics(X[-1] + [0, 0, LIFT], R31))
    return np.array(actual)


def plot(path, ink, actual=None):
    plt.imshow(ink, origin="lower", extent=[*SCAN_X, *SCAN_Y], cmap="Greys", alpha=0.4)
    plt.plot(*path.T, "b", label="planned path")
    if actual is not None:
        d = np.linalg.norm(actual[:, None, :2] - path[None], axis=2).min(1)
        plt.plot(*actual.T[:2], "r", label=f"actual tip, error mean {d.mean():.1f} / max {d.max():.1f} mm")
    plt.xlabel("x0 [mm]"), plt.ylabel("y0 [mm]"), plt.axis("equal"), plt.legend()
    plt.savefig("path.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    mode = sys.argv[1]
    robot, cap = MyRobot(), cv2.VideoCapture(vision.CAM_INDEX)
    try:
        if mode == "check":
            check(robot)
        elif mode == "calib":
            calibrate(robot, cap)
        else:
            cal = {**np.load(vision.INTRINSICS), **np.load(EXTRINSICS)}
            path, ink = scan(robot, cap, cal)
            X, Q = plan(path, cal)
            plot(path, ink, trace(robot, X, Q) if mode == "run" else None)
    finally:
        robot.move_j(0, 0, 0, 0)
        robot.disable_motors()