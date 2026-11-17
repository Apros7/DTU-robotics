"""Camera: pinhole model (intrinsics + extrinsics), segmentation, backward projection, line -> path.

  python vision.py          live preview: camera | segmentation   (q = quit)
  python vision.py calib    intrinsic calibration with a chessboard -> camera_intrinsics.npz
"""
import sys
from itertools import combinations
from pathlib import Path
import numpy as np
import cv2
from scipy.ndimage import uniform_filter1d
from skimage.morphology import skeletonize
from kinematics import fk

CAM_INDEX = 0                              # try 1, 2 ... if 0 is the laptop webcam
PATTERN, SQUARE_MM = (9, 6), 25.0           # chessboard: inner corners, square size (measure your print)
INTRINSICS = Path(__file__).with_name("camera_intrinsics.npz")


def read(cap):
    for _ in range(30):                     # the first frames can be empty while the camera starts up
        ok, img = cap.read()
        if ok:
            return img
    sys.exit(f"No picture from camera {CAM_INDEX}: try another CAM_INDEX, or allow camera access for your terminal")


def grab(cap):
    for _ in range(5):                      # drop the frames buffered while the robot was moving
        cap.read()
    return read(cap)


def segment(img):
    """Black ink -> 1. Adaptive threshold, then opening (erosion + dilation) to remove specks."""
    gray = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    bw = cv2.adaptiveThreshold(gray, 1, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 51, 20)
    return cv2.morphologyEx(bw, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def pixel_to_table(uv, q, cal):
    """Backward projection of pixels (u, v) onto the table plane z_0 = z_table, robot at q [rad].
    cal = K, dist (intrinsics) + roll [deg], offset (camera centre in frame 4), z_table (extrinsics)."""
    rays = cv2.undistortPoints(np.float64(uv).reshape(-1, 1, 2), cal["K"], cal["dist"]).reshape(-1, 2)
    rays = np.column_stack([rays, np.ones(len(rays))])            # (x/z, y/z, 1) in the camera frame
    # camera axes in frame 4: optical axis z_c = x_4, image x_c = z_4, y_c = -y_4, turned by roll about z_c
    R_c4 = np.array([[0, 0, 1], [0, -1, 0], [1., 0, 0]]) @ cv2.Rodrigues(np.radians([0, 0, float(cal["roll"])]))[0]
    T = fk(q)
    o = T[:3, 3] + T[:3, :3] @ cal["offset"]                      # camera centre o_c^0
    rays = rays @ (T[:3, :3] @ R_c4).T                            # ray directions in the base frame
    return o + rays * (cal["z_table"] - o[2]) / rays[:, 2:]       # scale each ray until it hits the table


def dot_centre(mask):
    """Centroid (u, v) of the biggest blob that does not touch the image border."""
    _, _, st, cent = cv2.connectedComponentsWithStats(mask)
    inside = (st[:, 0] > 0) & (st[:, 1] > 0) & (st[:, 0] + st[:, 2] < mask.shape[1]) & (st[:, 1] + st[:, 3] < mask.shape[0])
    return cent[np.argmax(st[:, 4] * inside)]


def line_path(ink):
    """Top-down ink map -> ordered, smoothed centre line as (col, row) points."""
    ink = cv2.morphologyEx(ink.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))   # close small gaps
    _, lab, st, _ = cv2.connectedComponentsWithStats(ink)
    skel = skeletonize(lab == 1 + np.argmax(st[1:, 4])).astype(np.uint8)       # biggest blob, 1 px wide
    c = max(cv2.findContours(skel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0], key=len)[:, 0]
    closed = len(c) < 1.5 * skel.sum()      # the contour of an open curve runs there and back (2x the pixels)
    if not closed:                          # keep one pass, between the two turning points that halve the contour
        turn = np.flatnonzero((np.roll(c, 1, 0) == np.roll(c, -1, 0)).all(1))
        i, j = min(combinations(turn, 2), key=lambda p: abs(p[1] - p[0] - len(c) / 2))
        c = c[i:j + 1]
    c = uniform_filter1d(c.astype(float), 5, axis=0, mode="wrap" if closed else "nearest")
    return np.vstack([c, c[:1]]) if closed else c


def preview(cap):
    while cv2.waitKey(1) & 0xFF != ord("q"):
        img = read(cap)
        cv2.imshow("camera | segmentation", np.hstack([cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 255 * segment(img)]))


def calibrate_intrinsics(cap):
    """SPACE = keep the picture (when the corners are drawn), q = calibrate. Take 15-20 pictures."""
    grid = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    grid[:, :2] = np.mgrid[:PATTERN[0], :PATTERN[1]].T.reshape(-1, 2) * SQUARE_MM
    obj, pts = [], []
    while (key := cv2.waitKey(1) & 0xFF) != ord("q"):
        img = read(cap)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, PATTERN)
        if found and key == ord(" "):
            obj.append(grid)
            pts.append(corners)
            print(len(obj), "pictures")
        cv2.imshow("chessboard", cv2.drawChessboardCorners(img, PATTERN, corners, found))
    rms, K, dist = cv2.calibrateCamera(obj, pts, gray.shape[::-1], None, None)[:3]
    np.savez(INTRINSICS, K=K, dist=dist)
    print(f"reprojection error {rms:.2f} px (should be < 1)\nK =\n{K.round(1)}")


if __name__ == "__main__":
    cap = cv2.VideoCapture(CAM_INDEX)
    calibrate_intrinsics(cap) if sys.argv[1:] == ["calib"] else preview(cap)