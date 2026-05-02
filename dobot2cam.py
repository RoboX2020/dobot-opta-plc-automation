# dobot2.py — Pick-and-place + teaching + PLC + color vision
import DobotDllType as dType
import cv2
import numpy as np
from collections import Counter, deque
import json
import os
import sys
import time

# ============================================================
# CONNECTION
# ============================================================
COM_PORT = "COM"
BAUD = 115200
api = None

# ============================================================
# STORAGE
# ============================================================
COORDS_FILE = "dobot2_coords.json"
positions = {}
tasks = []

# ============================================================
# MOTION SETTINGS
# ============================================================
SAFE_HOVER_Z = 60.0
DRY_RUN = False

# ============================================================
# PLC HANDSHAKE
# ============================================================
USE_PLC = True
PLC_START_PIN = 16
PLC_DONE_PIN = 22

# ============================================================
# VISION
# ============================================================
CAMERA_INDEX = 0          # try 0 first, then 1 if no camera opens
FIXED_TRAY_RECT = (100, 100, 400, 300)   # (x, y, width, height) of the 2x2 tray in the frame
TRAY_GRID_EXPAND_X = 0.05
TRAY_GRID_EXPAND_Y = 0.09
ROI_INSET_PX = 8
MIN_CONTOUR_AREA = 1800
MIN_COLOR_PIXELS = 1000   # below this, cell is treated as empty

# Consensus voting (per cell, across frames) before locking a color
VOTE_WINDOW = 45
MIN_STABLE_VOTES = 12
STABLE_RATIO = 0.60
MIN_MARGIN = 3
SCAN_WAIT_FRAMES = 100    # how many frames to scan before freezing the map

COLOR_DISPLAY = {
    "BLUE":  (255, 0, 0),
    "GREEN": (0, 255, 0),
    "PINK":  (203, 192, 255),
}

# ============================================================
# DOBOT HELPERS
# ============================================================
def wait_for_cmd(idx):
    while True:
        cur = dType.GetQueuedCmdCurrentIndex(api)[0]
        if cur >= idx:
            break
        dType.dSleep(50)

def ptp(x, y, z, r=0.0):
    return dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, z, r, 1)[0]

def get_pose():
    p = dType.GetPose(api)
    return float(p[0]), float(p[1]), float(p[2]), float(p[3])

def suction(on):
    dType.SetEndEffectorSuctionCup(api, 1, 1 if on else 0, 1)

def queued_wait(ms):
    dType.SetWAITCmd(api, ms, 1)

def setup_plc_io():
    if not USE_PLC:
        return
    dType.SetIOMultiplexing(api, PLC_START_PIN, 0)
    dType.SetIOMultiplexing(api, PLC_DONE_PIN, 1)
    dType.SetIODO(api, PLC_DONE_PIN, 0)
    print(f"PLC IO configured: start=pin {PLC_START_PIN}, done=pin {PLC_DONE_PIN}")

def wait_for_plc_start():
    print(f"Waiting for PLC start signal on pin {PLC_START_PIN}... (Ctrl+C to cancel)")
    try:
        while True:
            val = dType.GetIODI(api, PLC_START_PIN)[0]
            if val == 0:
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Wait canceled.")
        raise
    print("PLC start received.")

def signal_plc_done():
    print("Asserting done signal to PLC.")
    dType.SetIODO(api, PLC_DONE_PIN, 1)
    time.sleep(1.0)
    dType.SetIODO(api, PLC_DONE_PIN, 0)

def connect():
    global api
    api = dType.load()
    state = dType.ConnectDobot(api, COM_PORT, BAUD)
    print(f"ConnectDobot -> {state}")
    if state[0] != 0:
        print(f"Connection failed (status {state[0]}). 0=ok, 1=not found, 2=occupied, 3=config error.")
        sys.exit(1)
    try:
        dType.ClearAllAlarmsState(api)
    except Exception:
        pass
    dType.SetQueuedCmdClear(api)
    dType.SetQueuedCmdStartExec(api)
    dType.SetPTPCommonParams(api, 50, 50, 0)
    setup_plc_io()
    print("Dobot ready.")

def disconnect():
    try:
        dType.SetQueuedCmdStopExec(api)
        dType.DisconnectDobot(api)
    except Exception:
        pass

def home_arm():
    print("Homing... (arm will move to home position)")
    home_idx = dType.SetHOMECmd(api, 0, 1)[0]
    wait_for_cmd(home_idx)
    print("Home complete.")

# ============================================================
# PERSISTENCE
# ============================================================
def load_data():
    global positions, tasks
    if os.path.exists(COORDS_FILE):
        try:
            with open(COORDS_FILE, "r") as f:
                content = f.read().strip()
            if not content:
                raise ValueError("file is empty")
            data = json.loads(content)
            positions = data.get("positions", {})
            tasks = data.get("tasks", [])
            print(f"Loaded {len(positions)} positions, {len(tasks)} tasks from {COORDS_FILE}")
            return
        except (json.JSONDecodeError, ValueError) as e:
            print(f"Warning: {COORDS_FILE} is corrupt or empty ({e}). Starting fresh.")
            backup = COORDS_FILE + ".bad"
            try:
                os.replace(COORDS_FILE, backup)
                print(f"Moved bad file to {backup} for inspection.")
            except Exception:
                pass
    positions = {}
    tasks = []

def save_data():
    with open(COORDS_FILE, "w") as f:
        json.dump({"positions": positions, "tasks": tasks}, f, indent=2)
    print(f"Saved {len(positions)} positions, {len(tasks)} tasks to {COORDS_FILE}")

# ============================================================
# TEACHING
# ============================================================
def teach_position(name):
    print("\n" + "=" * 50)
    print(f"  TEACHING: '{name}'")
    print("=" * 50)
    print("  1. Hold the button on top of the dobot arm (motors release).")
    print("  2. Drag the arm to the target position.")
    print("  3. Release the button (motors lock here).")
    print("  4. Press Enter to capture, or 'c' + Enter to cancel.")
    if input("> ").strip().lower() == "c":
        print("Canceled.")
        return False
    x, y, z, r = get_pose()
    positions[name] = [x, y, z, r]
    print(f"  Captured '{name}' = ({x:.2f}, {y:.2f}, {z:.2f}, {r:.2f})")
    save_data()
    return True

def list_positions():
    if not positions:
        print("No positions saved.")
        return
    print(f"\n--- Saved positions ({len(positions)}) ---")
    for name in sorted(positions.keys()):
        x, y, z, r = positions[name]
        print(f"  {name:20s} ({x:7.2f}, {y:7.2f}, {z:7.2f}, {r:6.2f})")

def list_tasks():
    if not tasks:
        print("No tasks defined.")
        return
    print(f"\n--- Tasks ({len(tasks)}) ---")
    for i, t in enumerate(tasks, 1):
        print(f"  [{i}] pick:'{t['pick']}'  ->  place:'{t['place']}'")

def delete_position(name):
    if name in positions:
        del positions[name]
        save_data()
        print(f"Deleted position '{name}'.")
    else:
        print(f"'{name}' not found.")

def add_task():
    list_positions()
    if len(positions) < 2:
        print("Need at least two saved positions before defining a task.")
        return
    pick = input("Pick position name: ").strip()
    if pick not in positions:
        print(f"'{pick}' not found.")
        return
    place = input("Place position name: ").strip()
    if place not in positions:
        print(f"'{place}' not found.")
        return
    tasks.append({"pick": pick, "place": place})
    save_data()
    print(f"Added task: {pick} -> {place}")

def delete_task(idx_str):
    try:
        idx = int(idx_str) - 1
        if 0 <= idx < len(tasks):
            removed = tasks.pop(idx)
            save_data()
            print(f"Removed task: {removed['pick']} -> {removed['place']}")
        else:
            print("Index out of range.")
    except ValueError:
        print("Invalid index.")

def test_move(name):
    if name not in positions:
        print(f"'{name}' not found.")
        return
    x, y, z, r = positions[name]
    print(f"Moving to '{name}' = ({x:.2f}, {y:.2f}, {z:.2f}, {r:.2f})")
    print(f"Will go to safe Z={SAFE_HOVER_Z} first, then descend.")
    if input("Proceed? (y/N) > ").strip().lower() != "y":
        return
    ptp(x, y, SAFE_HOVER_Z, r)
    idx = ptp(x, y, z, r)
    wait_for_cmd(idx)
    print("Arrived. Press Enter to retract.")
    input()
    idx = ptp(x, y, SAFE_HOVER_Z, r)
    wait_for_cmd(idx)

# ============================================================
# PICK AND PLACE PRIMITIVES
# ============================================================
def do_pick(name):
    x, y, z, r = positions[name]
    print(f"  PICK '{name}' ({x:.1f}, {y:.1f}, {z:.1f})")
    if DRY_RUN:
        return
    ptp(x, y, SAFE_HOVER_Z, r)
    ptp(x, y, z, r)
    suction(True)
    queued_wait(500)
    idx = ptp(x, y, SAFE_HOVER_Z, r)
    wait_for_cmd(idx)

def do_place(name):
    x, y, z, r = positions[name]
    print(f"  PLACE '{name}' ({x:.1f}, {y:.1f}, {z:.1f})")
    if DRY_RUN:
        return
    ptp(x, y, SAFE_HOVER_Z, r)
    ptp(x, y, z, r)
    suction(False)
    queued_wait(500)
    idx = ptp(x, y, SAFE_HOVER_Z, r)
    wait_for_cmd(idx)

# ============================================================
# VISION
# ============================================================
def get_cell_roi(cell_index):
    tx, ty, tw, th = FIXED_TRAY_RECT
    idx = cell_index - 1
    row = idx // 2
    col = idx % 2
    expand_x = int(tw * TRAY_GRID_EXPAND_X)
    expand_y = int(th * TRAY_GRID_EXPAND_Y)
    usable_x = max(0, tx - expand_x)
    usable_y = max(0, ty - expand_y)
    usable_w = max(1, tw + 2 * expand_x)
    usable_h = max(1, th + 2 * expand_y)
    cell_w = usable_w / 2.0
    cell_h = usable_h / 2.0
    x1 = int(usable_x + col * cell_w)
    y1 = int(usable_y + row * cell_h)
    return (x1 + ROI_INSET_PX, y1 + ROI_INSET_PX,
            max(1, int(cell_w) - 2 * ROI_INSET_PX),
            max(1, int(cell_h) - 2 * ROI_INSET_PX))

def detect_color_in_cell(frame, cell_index):
    x, y, w, h = get_cell_roi(cell_index)
    roi = frame[y:y + h, x:x + w]
    if roi.size == 0:
        return None, None, (x, y, w, h)
    blurred = cv2.GaussianBlur(roi, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    pink1 = cv2.inRange(hsv, (145, 55, 45), (180, 255, 255))
    pink2 = cv2.inRange(hsv, (0, 55, 45), (10, 255, 255))
    mask_pink = cv2.bitwise_or(pink1, pink2)
    mask_green = cv2.inRange(hsv, (40, 60, 60), (85, 255, 255))
    mask_blue = cv2.inRange(hsv, (90, 60, 60), (130, 255, 255))
    counts = {
        "PINK":  cv2.countNonZero(mask_pink),
        "GREEN": cv2.countNonZero(mask_green),
        "BLUE":  cv2.countNonZero(mask_blue),
    }
    dominant = max(counts, key=counts.get)
    if counts[dominant] < MIN_COLOR_PIXELS:
        return None, None, (x, y, w, h)
    active = mask_pink if dominant == "PINK" else (mask_green if dominant == "GREEN" else mask_blue)
    kernel = np.ones((5, 5), np.uint8)
    active = cv2.morphologyEx(active, cv2.MORPH_OPEN, kernel)
    active = cv2.morphologyEx(active, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(active, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_cnt = None
    if contours:
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) > MIN_CONTOUR_AREA:
            valid_cnt = cnt
    return dominant, valid_cnt, (x, y, w, h)

def draw_overlay(display, x, y, contour, color_name):
    bgr = COLOR_DISPLAY.get(color_name, (255, 255, 255))
    if contour is not None:
        contour_full = contour + np.array([[[x, y]]], dtype=np.int32)
        cv2.drawContours(display, [contour_full], -1, bgr, 2)
    cv2.putText(display, color_name, (x + 5, y + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)

def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Could not open camera {CAMERA_INDEX}. Try CAMERA_INDEX = 1 or 2.")
        return None
    return cap

def preview_camera():
    """Live camera view with tray rect + 4 cell color overlays. No motion. q to quit."""
    cap = open_camera()
    if cap is None:
        return
    print("Camera preview running. Press 'q' in the video window to quit.")
    print(f"FIXED_TRAY_RECT = {FIXED_TRAY_RECT}.  Edit this constant at top of file to align with your tray.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue
            display = frame.copy()
            tx, ty, tw, th = FIXED_TRAY_RECT
            cv2.rectangle(display, (tx, ty), (tx + tw, ty + th), (0, 0, 255), 2)
            cv2.putText(display, "TRAY (edit FIXED_TRAY_RECT to align)",
                        (tx, ty - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            for i in range(1, 5):
                cx, cy, cw, ch = get_cell_roi(i)
                cv2.rectangle(display, (cx, cy), (cx + cw, cy + ch), (255, 255, 255), 1)
                cv2.putText(display, f"S{i}", (cx + 4, cy + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                color, cnt, _ = detect_color_in_cell(frame, i)
                if color:
                    draw_overlay(display, cx, cy, cnt, color)
            cv2.imshow("Dobot 2 Camera Preview", display)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print("Preview closed.")

def scan_tray():
    """Open camera, build a stable {cell -> color} map via consensus voting. Returns the map."""
    cap = open_camera()
    if cap is None:
        return {}
    cell_votes = {i: deque(maxlen=VOTE_WINDOW) for i in range(1, 5)}
    cell_locked = {i: None for i in range(1, 5)}

    def consensus(i, raw):
        if cell_locked[i] is not None:
            return cell_locked[i]
        v = cell_votes[i]
        if raw is not None:
            v.append(raw)
        if not v:
            return None
        c = Counter(v)
        winner, wc = c.most_common(1)[0]
        second = c.most_common(2)[1][1] if len(c) > 1 else 0
        ratio = wc / float(len(v))
        if wc >= MIN_STABLE_VOTES and ratio >= STABLE_RATIO and (wc - second) >= MIN_MARGIN:
            cell_locked[i] = winner
        return cell_locked[i]

    frames_scanned = 0
    print("Scanning tray... (press 'q' in video window to abort)")
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue
            display = frame.copy()
            tx, ty, tw, th = FIXED_TRAY_RECT
            cv2.rectangle(display, (tx, ty), (tx + tw, ty + th), (0, 0, 255), 2)
            for i in range(1, 5):
                cx, cy, cw, ch = get_cell_roi(i)
                cv2.rectangle(display, (cx, cy), (cx + cw, cy + ch), (255, 255, 255), 1)
                raw, cnt, _ = detect_color_in_cell(frame, i)
                stable = consensus(i, raw)
                if stable:
                    draw_overlay(display, cx, cy, cnt, stable)
                    cv2.putText(display, "LOCKED", (cx + 5, cy + 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
                elif raw:
                    draw_overlay(display, cx, cy, cnt, raw)
            mapped = sum(1 for v in cell_locked.values() if v is not None)
            cv2.putText(display, f"SCAN: {mapped}/4 LOCKED  frame {frames_scanned}/{SCAN_WAIT_FRAMES}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Dobot 2 Vision Scan", display)
            frames_scanned += 1
            if frames_scanned >= SCAN_WAIT_FRAMES:
                break
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    final = {i: v for i, v in cell_locked.items() if v is not None}
    print(f"Scan result: {final}")
    return final

# ============================================================
# PLAY MODES
# ============================================================
def play():
    """Existing pick-place mode using the tasks list."""
    if not tasks:
        print("No tasks to run.")
        return
    missing = set()
    for t in tasks:
        if t["pick"] not in positions:
            missing.add(t["pick"])
        if t["place"] not in positions:
            missing.add(t["place"])
    if missing:
        print(f"Missing positions: {', '.join(sorted(missing))}")
        return
    print(f"\n=== PLAY: {len(tasks)} tasks ===")
    if USE_PLC:
        wait_for_plc_start()
    if "start" in positions:
        x, y, z, r = positions["start"]
        wait_for_cmd(ptp(x, y, z, r))
    for i, t in enumerate(tasks, 1):
        print(f"\n[Task {i}/{len(tasks)}] {t['pick']} -> {t['place']}")
        do_pick(t["pick"])
        do_place(t["place"])
    if "start" in positions:
        x, y, z, r = positions["start"]
        wait_for_cmd(ptp(x, y, z, r))
    print("\n=== ALL TASKS DONE ===")
    if USE_PLC:
        signal_plc_done()

def play_vision():
    """Wait for PLC, scan tray with camera, sort blocks by detected color."""
    # Validate positions
    required = ["tray_1", "tray_2", "tray_3", "tray_4"]
    missing = [p for p in required if p not in positions]
    if missing:
        print(f"Missing required tray positions: {', '.join(missing)}")
        return
    if "drop_blue" not in positions and "drop_green" not in positions:
        print("Need at least 'drop_blue' or 'drop_green' to be taught.")
        return

    if USE_PLC:
        wait_for_plc_start()

    if "start" in positions:
        x, y, z, r = positions["start"]
        wait_for_cmd(ptp(x, y, z, r))

    color_map = scan_tray()
    if not color_map:
        print("No blocks detected. Aborting.")
        if USE_PLC:
            signal_plc_done()
        return

    print("\n--- Sorting ---")
    for cell in sorted(color_map.keys()):
        color = color_map[cell]
        drop_name = f"drop_{color.lower()}"
        if drop_name not in positions:
            print(f"  -> Skipping cell {cell} ({color}): no '{drop_name}' position taught.")
            continue
        do_pick(f"tray_{cell}")
        do_place(drop_name)

    if "start" in positions:
        x, y, z, r = positions["start"]
        wait_for_cmd(ptp(x, y, z, r))

    print("\n=== VISION SORT DONE ===")
    if USE_PLC:
        signal_plc_done()

# ============================================================
# MENUS
# ============================================================
def teach_menu():
    while True:
        print("\n" + "-" * 50)
        print("  TEACH MENU")
        print("-" * 50)
        print("  t <name>   Teach (or re-teach) a position by name")
        print("  l          List saved positions")
        print("  d <name>   Delete a position")
        print("  m <name>   Move to a saved position (test)")
        print("  +          Add a pick-place task")
        print("  -          Remove a task by index")
        print("  ls         List tasks")
        print("  s          Save now")
        print("  p          Switch to play mode")
        print("  q          Back to main menu")
        cmd = input("> ").strip()
        if not cmd:
            continue
        parts = cmd.split(maxsplit=1)
        action = parts[0].lower()
        arg = parts[1].strip() if len(parts) > 1 else None
        if action == "t":
            if not arg:
                arg = input("Position name: ").strip()
            if arg:
                teach_position(arg)
        elif action == "l":
            list_positions()
        elif action == "d":
            if not arg:
                arg = input("Position to delete: ").strip()
            if arg:
                delete_position(arg)
        elif action == "m":
            if not arg:
                arg = input("Position to move to: ").strip()
            if arg:
                test_move(arg)
        elif action == "+":
            add_task()
        elif action == "-":
            if not arg:
                list_tasks()
                arg = input("Task index to remove: ").strip()
            if arg:
                delete_task(arg)
        elif action == "ls":
            list_tasks()
        elif action == "s":
            save_data()
        elif action == "p":
            return "play"
        elif action == "q":
            save_data()
            return "main"
        else:
            print("Unknown command.")

def main():
    print("=" * 50)
    print("  DOBOT 2 — Pick & place + teaching + PLC + vision")
    print("=" * 50)
    load_data()
    connect()
    try:
        while True:
            print("\nMain menu:")
            print("  [1] Teach mode")
            print("  [2] Play mode (named tasks)")
            print("  [3] List positions")
            print("  [4] List tasks")
            print("  [5] Home the arm")
            print("  [6] Quit")
            print("  [7] Camera preview (no motion — verify vision before sorting)")
            print("  [8] Vision sort (PLC -> scan tray -> sort by color)")
            choice = input("> ").strip()
            if choice == "1":
                if teach_menu() == "play":
                    if input("Home the arm before play? (y/N) > ").strip().lower() == "y":
                        home_arm()
                    play()
            elif choice == "2":
                if input("Home the arm before play? (y/N) > ").strip().lower() == "y":
                    home_arm()
                play()
            elif choice == "3":
                list_positions()
            elif choice == "4":
                list_tasks()
            elif choice == "5":
                home_arm()
            elif choice == "6":
                break
            elif choice == "7":
                preview_camera()
            elif choice == "8":
                if input("Home the arm before vision sort? (y/N) > ").strip().lower() == "y":
                    home_arm()
                play_vision()
            else:
                print("Invalid choice.")
    finally:
        save_data()
        disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()