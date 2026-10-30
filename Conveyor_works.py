# ============================================================================
# Conveyor_works.py - the AI vision sorter: a camera watches a 12-cell tray
# of wooden blocks, figures out which shape (CUBE / SPHERE / PENTAGON /
# CLOVER) sits in each cell, then a Dobot robot arm picks them up one by
# one and sorts them onto transfer trays - one of each shape per tray.
# When a tray is complete, the arm carries it away and the conveyor belt
# brings the next one.
#
# HOW IT WORKS, IN ORDER:
#   Phase 1 (scanning): every camera frame, each of the 12 cells is checked.
#     A shape guess is made per frame; a cell "locks in" only after one
#     shape keeps winning (voting), so one bad frame can't fool it.
#     Then a sanity auditor makes sure the totals match reality (3 of each
#     shape) and flips the least-confident cells if they don't.
#   Phase 2 (sorting): the arm walks the frozen map, picking blocks and
#     dropping each at its shape's corner, tray after tray.
#
# THE AI PART: you can teach it by example - press a-l to target a cell,
# then 1-4 to say what's really there. Those examples train a small
# RandomForest model (see train_model) that then does the guessing.
# Without training data it falls back to geometric rules (classify_shape).
#
# RUN IT on the robot laptop: python Conveyor_works.py
#   Mode switches at the top: PREVIEW_ONLY = camera only, no robot;
#   DRY_RUN = print what the robot WOULD do without moving it.
# WHEN DEBUGGING (reasoned from the code):
#   - "Could not open camera": wrong webcam index or camera busy.
#   - Shapes misread: bad lighting - retrain with the 0/a-l/1-4 keys.
#   - Robot coordinates (TRAY_POINTS etc.) are hand-taught for one exact
#     physical setup; move the tray and they must be re-taught.
#   - Needs the vendor library DobotEDU (from Dobot) plus opencv, numpy,
#     scikit-learn (see requirements.txt).
# ============================================================================
from DobotEDU import *
import cv2
import numpy as np
from typing import Optional, Tuple
from collections import Counter, deque
import csv
import os
import time
from sklearn.ensemble import RandomForestClassifier

# Changed the training file name to avoid the corrupted one
TRAIN_FILE = "training_data_v2.csv"
model = None

# ============================================================
# GLOBAL AI CONSTRAINTS & TIMERS
# ============================================================
EXPECTED_COUNTS = {
    "CUBE": 3,
    "PENTAGON": 3,
    "SPHERE": 3,
    "CLOVER": 3
}

# Global tracker for the conveyor belt
conveyor_stop_time = 0.0  

# ============================================================
# MODE SWITCHES
# ============================================================
USE_WEBCAM = True
DRY_RUN = False            # True = print robot actions; False = Move Robot
LOCKOUT_AFTER_PICK = True  # Only consume a cell after a real robot pickup
SHOW_PREVIEW = True        # Must be True to see the window
PREVIEW_ONLY = True        # SET TO TRUE TO TEST CAMERA ONLY (No robot logic)

# ============================================================
# COORDINATES & STACKING
# ============================================================
STACK_PICK_POINT = (0, 0, 0)
CONVEYOR_PLACE_POINT = (0, 0, 0)
TRANSITION_POINT = (250, 0, 100)
TRAY_Z_OFFSET = 5.0
Z_DOWN_OFFSET = 5.0

# The 12 pickup cells of the master tray, in robot millimeters (X, Y, Z).
# These were measured by hand ('teaching') and are specific to one exact
# physical setup - move the tray and they must be re-taught.
# Master Tray Coords
NEUTRAL_VIEW = (263.8, 0, 88, 0)

# Recalibrated tray points from the four tray corner anchors:
TRAY_POINTS = {
    1:  (340.70,  68.90, -32.40),
    2:  (340.70,  34.00, -32.40),
    3:  (340.70,   0, -32.40),
    4:  (340.70, -34.50, -32.40),

    5:  (308.8, -34.50, -32.75),
    6:  (308.8,   0, -32.72),
    7:  (308.8,  34.00, -32.68),
    8:  (308.8,  68.9, -32.65),

    9:  (276.30,  68.9, -32.90),
    10: (276.30,  34.0, -32.97),
    11: (276.3,   0, -33.03),
    12: (276.3, -34.50, -33.10),
}

# Where each sorted shape gets dropped on the transfer tray (one corner per shape).
# Transfer Tray Drop Points
OUTPUT_POINTS = {
    "CUBE": (325.1, -89.2, -16.3),
    "SPHERE": (325.1, -124.7, -16.3),
    "CLOVER": (285.8, -124.7, -16.3),
    "PENTAGON": (285.8, -89.2, -16.3),
}

# Where to grab a full transfer tray, and where to carry it when done.
# Completed Transfer Tray Pick & Drop
COMPLETED_TRAY_PICK = (305.22, -103.6, -35.2, 0)
COMPLETED_TRAY_PLACE = (-7, -274.7, 32, 0)

# Apply Z-down offset to pick/place blocks coordinates.
TRAY_POINTS = {k: (x, y, z - Z_DOWN_OFFSET) for k, (x, y, z) in TRAY_POINTS.items()}
OUTPUT_POINTS = {k: (x, y, z - Z_DOWN_OFFSET) for k, (x, y, z) in OUTPUT_POINTS.items()}

# ============================================================
# VISION SETTINGS
# ============================================================
# Vision tuning knobs: how big a blob counts as a block, how dark the tray
# must be to be found, how the 12-cell grid is carved out of the image.
MIN_CONTOUR_AREA = 1800
MAX_CONTOUR_AREA = 90000
APPROX_EPSILON_PCT = 0.02

TRAY_MIN_AREA = 20000
TRAY_THRESHOLD = 60
TRAY_GRID_EXPAND_X = 0.03
TRAY_GRID_EXPAND_Y = 0.087

VISUAL_ROI_SHRINK_X = 10  
VISUAL_ROI_SHRINK_Y = 10  

SCAN_WAIT_FRAMES = 150 

# Display colors for each recognized shape (OpenCV stores colors as BGR).
SHAPE_COLORS = {
    "SPHERE": (0, 255, 255),
    "CUBE": (0, 255, 0),
    "PENTAGON": (255, 0, 255),
    "CLOVER": (0, 165, 255),
}

CELL_KEYS = {
    ord('a'): 1, ord('b'): 2, ord('c'): 3, ord('d'): 4,
    ord('e'): 5, ord('f'): 6, ord('g'): 7, ord('h'): 8,
    ord('i'): 9, ord('j'): 10, ord('k'): 11, ord('l'): 12
}

# Voting: the camera guesses a shape every frame, but a cell only 'locks in'
# once one shape wins enough recent frames by a clear margin. This makes the
# scan immune to single-frame misreads (shadows, a hand passing by).
VOTE_WINDOW = 150         
MIN_STABLE_VOTES = 12     
STABLE_RATIO = 0.60       
MIN_MARGIN = 3            

# ============================================================
# NON-BLOCKING CONVEYOR CHECKER
# ============================================================
# The conveyor belt runs on a 10-second timer; this is called constantly to
# stop it on time (skipped entirely in the no-robot test modes).
def check_conveyor():
    """Checks if the conveyor has been running for 10 seconds and safely shuts it down."""
    global conveyor_stop_time
    if conveyor_stop_time > 0 and time.time() >= conveyor_stop_time:
        print("\n[ CONVEYOR ] 10 seconds elapsed. Stopping.")
        if not DRY_RUN and not PREVIEW_ONLY:
            magicbox.set_converyor(index=magicbox.STP1, enable=False, speed=0.0)
        conveyor_stop_time = 0.0  # Reset timer

# ============================================================
# ROBOT CONTROL
# ============================================================
# Move the arm to its home 'ready' pose above the tray.
def robot_move_to_start():
    print("ACTION: Moving to start coordinate (241.5, 3, 60, 0)")
    m_lite.set_ptpcmd(ptp_mode=1, x=241.5, y=3, z=60, r=0)
    m_lite.wait(delay=0.5)

# Pick the block from one cell: hover above it, descend, suction ON, lift.
def robot_pick(cell_index):
    if PREVIEW_ONLY:
        return
    if cell_index not in TRAY_POINTS:
        return
    x, y, z = TRAY_POINTS[cell_index]
    print(f"  -> Picking from Cell S{cell_index}")
    if DRY_RUN:
        return
    m_lite.set_ptpcmd(ptp_mode=1, x=x, y=y, z=50, r=0)
    m_lite.set_ptpcmd(ptp_mode=1, x=x, y=y, z=z, r=0)
    m_lite.set_endeffector_suctioncup(enable=True, on=True)
    m_lite.wait(delay=0.5)
    m_lite.set_ptpcmd(ptp_mode=1, x=x, y=y, z=50, r=0)
    check_conveyor() # Safety check during movement

# Drop the held block at its shape's corner: hover, descend, suction OFF, lift.
def robot_place(shape_name):
    if PREVIEW_ONLY:
        return
    if shape_name not in OUTPUT_POINTS:
        return
    x, y, z = OUTPUT_POINTS[shape_name]
    print(f"  -> Placing {shape_name}")
    if DRY_RUN:
        return
    m_lite.set_ptpcmd(ptp_mode=1, x=x, y=y, z=50, r=0)
    m_lite.set_ptpcmd(ptp_mode=1, x=x, y=y, z=z, r=0)
    m_lite.set_endeffector_suctioncup(enable=True, on=False)
    m_lite.wait(delay=0.5)
    m_lite.set_ptpcmd(ptp_mode=1, x=x, y=y, z=50, r=0)
    check_conveyor() # Safety check during movement

# Carry the finished transfer tray to the drop-off point (grab, lift, carry, release).
def robot_move_completed_tray():
    if PREVIEW_ONLY:
        return
    print("ACTION: Moving completed Transfer Tray to drop-off point")
    px, py, pz, pr = COMPLETED_TRAY_PICK
    dx, dy, dz, dr = COMPLETED_TRAY_PLACE
    if DRY_RUN:
        return
    m_lite.set_ptpcmd(ptp_mode=1, x=px, y=py, z=50, r=pr)
    m_lite.set_ptpcmd(ptp_mode=1, x=px, y=py, z=pz, r=pr)
    m_lite.set_endeffector_suctioncup(enable=True, on=True)
    m_lite.wait(delay=0.5)
    m_lite.set_ptpcmd(ptp_mode=1, x=px, y=py, z=50, r=pr)
    
    m_lite.set_ptpcmd(ptp_mode=1, x=dx, y=dy, z=50, r=dr)
    m_lite.set_ptpcmd(ptp_mode=1, x=dx, y=dy, z=dz, r=dr)
    m_lite.set_endeffector_suctioncup(enable=True, on=False)
    m_lite.wait(delay=0.5)
    m_lite.set_ptpcmd(ptp_mode=1, x=dx, y=dy, z=50, r=dr)

# ============================================================
# VISION LOGIC
# ============================================================
# Turn a camera image into a black-and-white mask where the colored blocks
# are white. Works by filtering for the blocks' colors (pink/green/blue) in
# HSV color space, which handles changing light better than raw RGB.
def segment_shapes(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Pink wraps around both ends of the hue circle, so it needs two ranges.
    pink1 = cv2.inRange(hsv, (145, 55, 45), (180, 255, 255))
    pink2 = cv2.inRange(hsv, (0, 55, 45), (10, 255, 255))
    pink = cv2.bitwise_or(pink1, pink2)

    green = cv2.inRange(hsv, (40, 60, 60), (85, 255, 255))
    blue = cv2.inRange(hsv, (90, 60, 60), (130, 255, 255))

    mask = cv2.bitwise_or(pink, green)
    mask = cv2.bitwise_or(mask, blue)

    # Clean the mask: open removes speckles, close fills holes, dilate grows
    # the blobs slightly so edges are captured fully.
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    return mask

# Find the big dark tray in the image: threshold dark pixels, clean up the
# mask, and take the largest blob - if it is big enough to be the tray.
def detect_tray(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)

    # BINARY_INV: pixels DARKER than the threshold become white.
    _, thresh = cv2.threshold(blur, TRAY_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((7, 7), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    tray_cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(tray_cnt) < TRAY_MIN_AREA:
        return None

    x, y, w, h = cv2.boundingRect(tray_cnt)
    return x, y, w, h, tray_cnt, thresh

# Pixel box of one of the 12 cells, assuming the tray fills a fixed part of
# the image (fallback for when the tray itself was not detected).
def get_cell_roi_coords(frame, cell_index):
    h, w = frame.shape[:2]
    idx = cell_index - 1
    row = idx // 4
    col = idx % 4 if row % 2 == 0 else 3 - (idx % 4)

    x1 = int((w * 0.08) + col * (w * 0.84 / 4))
    y1 = int((h * 0.08) + row * (h * 0.84 / 3))
    
    roi_w = int(w * 0.21)
    roi_h = int(h * 0.28)
    
    return x1 + VISUAL_ROI_SHRINK_X, y1 + VISUAL_ROI_SHRINK_Y, max(1, roi_w - (2 * VISUAL_ROI_SHRINK_X)), max(1, roi_h - (2 * VISUAL_ROI_SHRINK_Y))

# Same, but carved out of the tray's detected position (more accurate).
# Cells snake back and forth - odd rows run right-to-left - matching how
# they were numbered on the physical tray.
def get_cell_roi_coords_from_tray(tray_rect, cell_index):
    tx, ty, tw, th = tray_rect

    idx = cell_index - 1
    row = idx // 4
    col = idx % 4 if row % 2 == 0 else 3 - (idx % 4)

    expand_x = int(tw * TRAY_GRID_EXPAND_X)
    expand_y = int(th * TRAY_GRID_EXPAND_Y)

    usable_x = max(0, tx - expand_x)
    usable_y = max(0, ty - expand_y)
    usable_w = max(1, tw + 2 * expand_x)
    usable_h = max(1, th + 2 * expand_y)

    cell_w = usable_w / 4.0
    cell_h = usable_h / 3.0

    x1 = int(usable_x + col * cell_w)
    y1 = int(usable_y + row * cell_h)
    
    return x1 + VISUAL_ROI_SHRINK_X, y1 + VISUAL_ROI_SHRINK_Y, max(1, int(cell_w) - (2 * VISUAL_ROI_SHRINK_X)), max(1, int(cell_h) - (2 * VISUAL_ROI_SHRINK_Y))

# The inside angle at each corner of a polygon (squares are ~90 degrees;
# pentagons ~108).
def _polygon_internal_angles(pts):
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
    n = len(pts)
    if n < 3:
        return []
    angles = []
    for i in range(n):
        p_prev = pts[(i - 1) % n]
        p = pts[i]
        p_next = pts[(i + 1) % n]

        v1 = p_prev - p
        v2 = p_next - p
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 == 0 or n2 == 0:
            continue
        cosang = np.dot(v1, v2) / (n1 * n2)
        cosang = float(np.clip(cosang, -1.0, 1.0))
        angles.append(np.degrees(np.arccos(cosang)))
    return angles

# Decide what shape a blob is from geometry alone: how round it is, how
# rectangular, how many corners, and how lumpy (deep notches = clover lobes).
# A hand-tuned decision tree; the last lines are best-guess fallbacks.
def classify_shape(contour):
    area = cv2.contourArea(contour)
    peri = cv2.arcLength(contour, True)
    if area <= 0 or peri <= 0:
        return None, None, {}

    # solidity: how 'filled in' the blob is vs its convex hull (a clover's
    # notches score lower than a solid square).
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = area / hull_area if hull_area > 0 else 0.0

    # approx: the blob simplified to a polygon - its corner count helps tell
    # squares (4) from pentagons (5-7).
    approx = cv2.approxPolyDP(contour, APPROX_EPSILON_PCT * peri, True)
    verts = len(approx)

    x, y, w, h = cv2.boundingRect(contour)
    aspect = w / float(h) if h else 0.0
    extent = area / float(w * h) if (w * h) else 0.0
    # circularity: 1.0 for a perfect circle, lower for edgy shapes.
    circularity = (4.0 * np.pi * area) / (peri * peri) if peri else 0.0

    rect = cv2.minAreaRect(contour)
    (_, _), (rw, rh), _ = rect
    rect_area = rw * rh if rw > 0 and rh > 0 else 0.0
    rectangularity = area / rect_area if rect_area > 0 else 0.0
    rect_ratio = (max(rw, rh) / min(rw, rh)) if min(rw, rh) > 0 else 0.0

    angle_dev = 180.0
    if verts >= 4:
        angles = _polygon_internal_angles(approx)
        if angles:
            angle_dev = float(np.mean([abs(a - 90.0) for a in angles]))

    # convexity defects: deep notches along the outline - the clover's lobes.
    defects_count = 0
    hull_idx = cv2.convexHull(contour, returnPoints=False)
    if hull_idx is not None and len(hull_idx) > 3:
        defects = cv2.convexityDefects(contour, hull_idx)
        if defects is not None:
            defects_count = sum(
                1 for i in range(defects.shape[0])
                if defects[i, 0, 3] / 256.0 > 8.0
            )

    metrics = {
        "area": area, "peri": peri, "verts": verts, "solidity": solidity,
        "circularity": circularity, "defects": defects_count, "aspect": aspect,
        "extent": extent, "rectangularity": rectangularity, "rect_ratio": rect_ratio,
        "angle_dev": angle_dev,
    }

    # The decision tree: most confident rules first, looser fallbacks after.
    if defects_count >= 2 and (solidity < 0.94 or circularity < 0.86):
        return "CLOVER", approx, metrics
    pentagon_like = (5 <= verts <= 7 and 0.74 <= circularity <= 0.90 and rectangularity < 0.82 and angle_dev >= 10.0 and defects_count <= 1)
    if pentagon_like:
        return "PENTAGON", approx, metrics
    if circularity >= 0.84 and solidity >= 0.92 and defects_count <= 1:
        return "SPHERE", approx, metrics
    cube_like = (rectangularity >= 0.68 and angle_dev <= 28.0 and circularity <= 0.86 and solidity >= 0.90 and defects_count <= 1 and 0.72 <= aspect <= 1.35)
    if cube_like:
        return "CUBE", approx, metrics
    if defects_count >= 2:
        return "CLOVER", approx, metrics
    if 5 <= verts <= 7 and rectangularity < 0.82:
        return "PENTAGON", approx, metrics
    if circularity >= 0.80 and solidity >= 0.90 and defects_count <= 1:
        return "SPHERE", approx, metrics
    if verts <= 4 or rectangularity >= 0.66:
        return "CUBE", approx, metrics
    if 5 <= verts <= 7:
        return "PENTAGON", approx, metrics

    return "CUBE", approx, metrics

# Look inside one cell: find the biggest block-sized blob, then classify it
# with the trained AI model if one exists, else with the geometric rules.
def detect_shape_in_cell(frame, cell_index, tray_rect=None):
    if tray_rect is not None:
        x, y, w_roi, h_roi = get_cell_roi_coords_from_tray(tray_rect, cell_index)
    else:
        x, y, w_roi, h_roi = get_cell_roi_coords(frame, cell_index)

    roi = frame[y:y + h_roi, x:x + w_roi]
    if roi.size == 0:
        return None, None, None, None, (x, y, w_roi, h_roi)

    # Cut the cell out of the frame, find colored blobs, keep block-sized ones.
    mask = segment_shapes(roi)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    valid = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if MIN_CONTOUR_AREA < area < MAX_CONTOUR_AREA:
            valid.append(cnt)

    if not valid:
        return None, None, None, None, (x, y, w_roi, h_roi)

    cnt = max(valid, key=cv2.contourArea)
    features = extract_features(cnt)

    # Prefer the trained model; fall back to the geometric rule tree.
    if model is not None and features is not None:
        shape = model.predict([features])[0]
        approx = cv2.approxPolyDP(cnt, APPROX_EPSILON_PCT * cv2.arcLength(cnt, True), True)
        metrics = {"circularity": 0, "defects": 0, "verts": len(approx)} 
    else:
        shape, approx, metrics = classify_shape(cnt)

    return shape, cnt, approx, metrics, (x, y, w_roi, h_roi)

# Draw the box, outline and label of a detected block onto the preview window.
def draw_detection_overlay(display_frame, x, y, contour, approx, shape, metrics):
    color = SHAPE_COLORS.get(shape, (255, 255, 255))
    contour_full = contour + np.array([[[x, y]]], dtype=np.int32)
    approx_full = approx + np.array([[[x, y]]], dtype=np.int32)

    cv2.drawContours(display_frame, [contour_full], -1, color, 2)
    cv2.drawContours(display_frame, [approx_full], -1, (255, 255, 255), 1)

    cv2.putText(display_frame, shape, (x + 5, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    if metrics:
        dbg = f"circ={metrics.get('circularity',0):.2f} def={metrics.get('defects',0)} v={metrics.get('verts',0)}"
        cv2.putText(display_frame, dbg, (x + 5, y + 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

# The 5 numbers that describe a blob to the AI model (size, perimeter,
# corners, roundness, solidity).
def extract_features(cnt):
    area = cv2.contourArea(cnt)
    peri = cv2.arcLength(cnt, True)
    if peri == 0:
        return None
    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
    verts = len(approx)
    circularity = 4 * np.pi * area / (peri * peri)
    hull = cv2.convexHull(cnt)
    solidity = area / cv2.contourArea(hull) if cv2.contourArea(hull) > 0 else 0
    return [area, peri, verts, circularity, solidity]

# Append one labeled example to the training spreadsheet (CSV).
def save_sample(label, features):
    file_needs_header = not os.path.isfile(TRAIN_FILE) or os.path.getsize(TRAIN_FILE) == 0
    
    with open(TRAIN_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        if file_needs_header:
            writer.writerow(["label", "area", "peri", "verts", "circularity", "solidity"])
        writer.writerow([label] + features)
    print(f"Saved sample: {label}")

# Re-train the shape recognizer from all saved examples. A RandomForest is
# just a few hundred small decision trees voting; it learns 'blobs shaped
# like THESE numbers are a CUBE'.
def train_model():
    global model
    if not os.path.exists(TRAIN_FILE) or os.path.getsize(TRAIN_FILE) == 0:
        print("No training data found. Please save some samples first!")
        return
        
    X, y = [], []
    try:
        with open(TRAIN_FILE, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    features = [float(row["area"]), float(row["peri"]), float(row["verts"]), float(row["circularity"]), float(row["solidity"])]
                    y.append(row["label"])
                    X.append(features)
                except (ValueError, TypeError):
                    continue
                    
        if not X:
            print("Training data file is empty. Please save new samples.")
            return
            
        model = RandomForestClassifier(n_estimators=200)
        model.fit(X, y)
        print(f"Model trained on {len(y)} valid samples")
    except KeyError:
        print("ERROR: Your training data file is missing its header row. Please delete the file and start over.")
    except Exception as e:
        print(f"ERROR reading training file: {e}")

# ============================================================
# AI SELF-CORRECTION AUDITOR
# ============================================================
# Sanity auditor: we KNOW a full tray holds exactly 3 of each shape. If the
# scan's totals disagree, flip the least-confident surplus cells toward the
# missing shapes (judged by their frame-by-frame vote history).
def force_global_distribution(final_map, all_votes):
    current_counts = Counter(final_map.values())
    
    surplus = []
    deficit = []
    
    for shape, exp_count in EXPECTED_COUNTS.items():
        diff = current_counts.get(shape, 0) - exp_count
        if diff > 0:
            surplus.extend([shape] * diff)
        elif diff < 0:
            deficit.extend([shape] * abs(diff))
            
    for shape, count in current_counts.items():
        if shape not in EXPECTED_COUNTS:
            surplus.extend([shape] * count)
            
    if not surplus or not deficit:
        return final_map
        
    print("\n[AI SELF-CORRECTION] Discrepancy detected in global shape counts.")
    print(f"Raw Scan Map Counts: {dict(current_counts)}")
    
    corrected_map = final_map.copy()
    
    for needed_shape in deficit:
        best_candidate = None
        best_score = -9999
        
        for cell_idx, current_shape in corrected_map.items():
            if current_shape in surplus:
                votes = Counter(all_votes[cell_idx])
                total_votes = sum(votes.values())
                
                needed_votes = votes.get(needed_shape, 0)
                primary_votes = votes.get(current_shape, 0) if current_shape in EXPECTED_COUNTS else 0
                
                if total_votes > 0:
                    score = (needed_votes / total_votes) - (primary_votes / total_votes)
                else:
                    score = 0
                    
                if score > best_score:
                    best_candidate = cell_idx
                    best_score = score
                    
        if best_candidate is not None:
            old_shape = corrected_map[best_candidate]
            print(f"  -> OVERRIDE: Flipping Cell S{best_candidate} from {old_shape} to {needed_shape}")
            corrected_map[best_candidate] = needed_shape
            surplus.remove(old_shape)
            
    return corrected_map

def main():
    global conveyor_stop_time
    train_model()
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Per-cell vote history: the last VOTE_WINDOW raw guesses for each cell.
    cell_votes = {i: deque(maxlen=VOTE_WINDOW) for i in range(1, 13)}
    cell_stable_labels = {i: None for i in range(1, 13)}
    active_training_cell = None 

    # One cell's verdict so far. Once a shape wins by enough margin, the cell
    # is 'locked' and stops changing its mind.
    def consensus_label(cell_index: int, raw_label: Optional[str]) -> Tuple[Optional[str], int, int, float]:
        votes = cell_votes[cell_index]
        
        if raw_label is not None:
            votes.append(raw_label)
        
        if cell_stable_labels[cell_index] is not None:
            return cell_stable_labels[cell_index], MIN_STABLE_VOTES, MIN_STABLE_VOTES, 1.0

        if not votes:
            return None, 0, 0, 0.0

        counts = Counter(votes)
        winner, winner_count = counts.most_common(1)[0]
        second_count = counts.most_common(2)[1][1] if len(counts) > 1 else 0
        total = len(votes)
        ratio = winner_count / float(total)
        
        stable = (winner_count >= MIN_STABLE_VOTES and ratio >= STABLE_RATIO and (winner_count - second_count) >= MIN_MARGIN)
        
        if stable:
            cell_stable_labels[cell_index] = winner
            
        return cell_stable_labels[cell_index], winner_count, total, ratio

    try:
        if not PREVIEW_ONLY and not DRY_RUN:
            m_lite.set_armspeed_ratio(set_type=1, set_value=50)
            m_lite.set_homecmd()
            
        robot_move_to_start()

        print(f"MODE: {'PREVIEW ONLY' if PREVIEW_ONLY else 'PRODUCTION'}")
        
        # ==========================================
        # PHASE 1: SCANNING THE BOARD
        # ==========================================
        frames_scanned = 0
        frozen_map = {}
        
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                break

            # Find the tray, then scan all 12 cells on every frame, drawing
            # boxes and labels on a copy we can show.
            display_frame = frame.copy()
            tray_data = detect_tray(frame)
            tray_rect = (tray_data[0], tray_data[1], tray_data[2], tray_data[3]) if tray_data else None

            for i in range(1, 13):
                tx, ty, tw, th = get_cell_roi_coords_from_tray(tray_rect, i) if tray_rect else get_cell_roi_coords(frame, i)
                
                cv2.rectangle(display_frame, (tx, ty), (tx + tw, ty + th), (255, 255, 255), 1)

                raw_shape, cnt, approx, metrics, roi_rect = detect_shape_in_cell(frame, i, tray_rect=tray_rect)
                stable_shape, winner_count, total_votes, ratio = consensus_label(i, raw_shape)

                if active_training_cell == i:
                    cv2.rectangle(display_frame, (tx-2, ty-2), (tx + tw + 2, ty + th + 2), (0, 255, 255), 3)
                    cv2.putText(display_frame, "PRESS 1,2,3, or 4", (tx, ty - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                if stable_shape is not None and cnt is not None:
                    draw_detection_overlay(display_frame, tx, ty, cnt, approx, stable_shape, metrics)
                    cv2.putText(display_frame, "LOCKED", (tx + 5, ty + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
                elif raw_shape and cnt is not None:
                    draw_detection_overlay(display_frame, tx, ty, cnt, approx, raw_shape, metrics)

            if not PREVIEW_ONLY:
                if active_training_cell is None:
                    frames_scanned += 1
                
                mapped_count = sum(1 for v in cell_stable_labels.values() if v is not None)
                
                status_color = (0, 255, 0) if active_training_cell is None else (0, 255, 255)
                status_text = f"SCANNING: {mapped_count}/12 BLOCKS LOCKED IN" if active_training_cell is None else f"TRAINING PAUSED. LABELING CELL {active_training_cell}"
                
                cv2.putText(display_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                
                # After ~150 frames of scanning, freeze the map (unscanned cells
                # become UNKNOWN), let the auditor fix the counts, and move to
                # Phase 2 (the physical sort).
                if frames_scanned >= SCAN_WAIT_FRAMES:
                    frozen_map = {k: v for k, v in cell_stable_labels.items() if v is not None}
                    
                    for i in range(1, 13):
                        if i not in frozen_map:
                            frozen_map[i] = "UNKNOWN"
                    
                    frozen_map = force_global_distribution(frozen_map, cell_votes)
                    
                    print(f"\n--- SCAN COMPLETE ---")
                    print(f"Final AI Map: {frozen_map}")
                    break

            if SHOW_PREVIEW:
                cv2.imshow("Dobot Camera View", display_frame)

            key = cv2.waitKey(1) & 0xFF
            
            # Keyboard controls while scanning: q quit | 0 batch-train all cells
            # a-l target a cell | 1-4 label the targeted cell | Esc cancel |
            # t retrain | p print sample count.
            if key == ord('q'):
                break

            elif key == ord('0'):
                print("BATCH TRAINING TRIGGERED: Locking all current predictions to memory...")
                trained_count = 0
                for j in range(1, 13):
                    raw_shape, cnt, approx, metrics, roi_rect = detect_shape_in_cell(frame, j, tray_rect=tray_rect)
                    
                    best_label = cell_stable_labels[j] if cell_stable_labels[j] else raw_shape
                    
                    if cnt is not None and best_label is not None:
                        feat = extract_features(cnt)
                        if feat:
                            save_sample(best_label, feat)
                            cell_stable_labels[j] = best_label 
                            trained_count += 1
                
                if trained_count > 0:
                    train_model() 
                    print(f"SUCCESS: Batch trained {trained_count} blocks! All locked in.")
                    
                    if not PREVIEW_ONLY:
                        frames_scanned = SCAN_WAIT_FRAMES 
                else:
                    print("TRAINING ERROR: No shapes detected to save.")
                
            elif key in CELL_KEYS:
                active_training_cell = CELL_KEYS[key]
                print(f"Targeting Cell S{active_training_cell}. Press 1(CUBE), 2(PENTAGON), 3(SPHERE), 4(CLOVER), or Esc to cancel.")
                
            elif key in [ord('1'), ord('2'), ord('3'), ord('4')] and active_training_cell is not None:
                label_map = {ord('1'): "CUBE", ord('2'): "PENTAGON", ord('3'): "SPHERE", ord('4'): "CLOVER"}
                label = label_map[key]
                
                _, cnt, _, _, _ = detect_shape_in_cell(frame, active_training_cell, tray_rect)
                if cnt is not None:
                    feat = extract_features(cnt)
                    if feat: 
                        save_sample(label, feat)
                        train_model() 
                        
                        for j in range(1, 13):
                            cell_votes[j].clear()
                            cell_stable_labels[j] = None
                            
                        print(f"SUCCESS: AI trained {label} at Cell S{active_training_cell}! Resuming scan...")
                else:
                    print(f"TRAINING ERROR: No block detected in Cell S{active_training_cell} to train.")
                
                active_training_cell = None 
                
            elif key == 27: 
                if active_training_cell is not None:
                    print("Targeting canceled. Resuming scan...")
                    active_training_cell = None
            
            elif key == ord('t'):
                train_model()
            elif key == ord('p'):
                if os.path.exists(TRAIN_FILE):
                    with open(TRAIN_FILE) as f: count = sum(1 for _ in f) - 1
                    print(f"Total samples: {count}")
                    
        # ==========================================
        # PHASE 2: PHYSICAL EXECUTION & SORTING
        # ==========================================
        if not PREVIEW_ONLY:
            cap.release()
            cv2.destroyAllWindows()
            
            print("\n--- INITIATING PHYSICAL SORT ---")
            # Each transfer tray needs exactly one of each shape; up to 3 trays.
            REQUIRED_SHAPES = {"CUBE", "PENTAGON", "SPHERE", "CLOVER"}
            MAX_TRAYS = 3
            trays_completed = 0
            
            remaining_cells = sorted(list(frozen_map.keys()))
            
            while trays_completed < MAX_TRAYS and remaining_cells:
                check_conveyor() # Initial check

                print(f"\n[ STARTING TRANSFER TRAY {trays_completed + 1} ]")
                current_tray_needs = set(REQUIRED_SHAPES)
                cells_to_remove = []
                
                    # Walk the cells; if this cell's shape is still needed on the
                    # current tray, pick it and place it in its corner.
                for cell_idx in remaining_cells:
                    check_conveyor() # Check before grabbing block
                    
                    shape = frozen_map[cell_idx]
                    
                    if shape in current_tray_needs:
                        robot_pick(cell_idx)
                        robot_place(shape)
                        current_tray_needs.remove(shape)
                        cells_to_remove.append(cell_idx)
                        
                        check_conveyor() # Check right after moving a block

                        if not current_tray_needs:
                            break
                    else:
                        print(f"  -> Skipping Cell S{cell_idx} ({shape}). Tray already contains one.")

                for c in cells_to_remove:
                    remaining_cells.remove(c)
                    
                if not current_tray_needs:
                    robot_move_completed_tray()
                    trays_completed += 1
                    
                    # Tray complete: carry it off, then run the conveyor ~10s to
                    # bring the next one in.
                    # TRIGGER THE CONVEYOR TIMER
                    print("\n[ CONVEYOR ] Starting for ~10 seconds...")
                    if not DRY_RUN and not PREVIEW_ONLY:
                        magicbox.set_converyor(index=magicbox.STP1, enable=True, speed=-100.0)
                    conveyor_stop_time = time.time() + 10.0
                    
                else:
                    print("WARNING: Could not find all required shapes to complete the transfer tray.")
                    print(f"Remaining shapes on board: {[frozen_map[c] for c in remaining_cells]}")
                    break
            
            # --- FINAL SAFETY LOOP --- 
            # If the last tray finishes, we must keep the script alive just long enough 
            # for the conveyor 5-second timer to run out before Python kills the connection.
            if conveyor_stop_time > 0:
                print("\nWaiting for final conveyor cycle to finish...")
                while conveyor_stop_time > 0:
                    check_conveyor()
                    time.sleep(0.1)

            print("\n--- ALL CYCLES COMPLETED ---")

    finally:
        if cap.isOpened():
            cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()























