# dobot4.py — Pick-and-place with coordinate teaching + PLC handshake
import DobotDllType as dType
import json
import os
import sys
import time

# ============================================================
# CONNECTION
# ============================================================
COM_PORT = "COM6"     # set to dobot 2's COM port
BAUD = 115200
api = None

# ============================================================
# STORAGE
# ============================================================
COORDS_FILE = "dobot4_coords.json"
positions = {}        # name -> [x, y, z, r]
tasks = []            # ordered list of {"pick": <name>, "place": <name>}

# ============================================================
# MOTION SETTINGS
# ============================================================
SAFE_HOVER_Z = 60.0   # arm goes here above pick/place before descending
DRY_RUN = False

# ============================================================
# PLC HANDSHAKE
# ============================================================
USE_PLC = True
PLC_START_PIN = 16    # dobot pin wired to PLC start line (PLC D0 -> here)
PLC_DONE_PIN = 22     # dobot pin wired to PLC input (here -> PLC A1)

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
    dType.SetIOMultiplexing(api, PLC_START_PIN, 0)   # 0 = input
    dType.SetIOMultiplexing(api, PLC_DONE_PIN, 1)    # 1 = output
    dType.SetIODO(api, PLC_DONE_PIN, 0)              # done LOW at boot
    print(f"PLC IO configured: start=pin {PLC_START_PIN}, done=pin {PLC_DONE_PIN}")

def wait_for_plc_start():
    print(f"Waiting for PLC start signal on pin {PLC_START_PIN}... (Ctrl+C to cancel)")
    try:
        while True:
            val = dType.GetIODI(api, PLC_START_PIN)[0]
            if val == 0:                # active-low, matches the chain
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
        print(f"Connection failed (status {state[0]}). "
              "0=ok, 1=not found, 2=occupied, 3=config error.")
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
# PICK AND PLACE
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
# PLAY MODE
# ============================================================
def play():
    if not tasks:
        print("No tasks to run. Add tasks in the teach menu first.")
        return

    missing = set()
    for t in tasks:
        if t["pick"] not in positions:
            missing.add(t["pick"])
        if t["place"] not in positions:
            missing.add(t["place"])
    if missing:
        print(f"Missing positions referenced by tasks: {', '.join(sorted(missing))}")
        return

    print(f"\n=== PLAY: {len(tasks)} tasks ===")

    if USE_PLC:
        wait_for_plc_start()

    if "start" in positions:
        x, y, z, r = positions["start"]
        idx = ptp(x, y, z, r)
        wait_for_cmd(idx)

    for i, t in enumerate(tasks, 1):
        print(f"\n[Task {i}/{len(tasks)}] {t['pick']} -> {t['place']}")
        do_pick(t["pick"])
        do_place(t["place"])

    if "start" in positions:
        x, y, z, r = positions["start"]
        idx = ptp(x, y, z, r)
        wait_for_cmd(idx)

    print("\n=== ALL TASKS DONE ===")

    if USE_PLC:
        signal_plc_done()
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
    print("  DOBOT 2 — Pick & place with teaching + PLC")
    print("=" * 50)
    load_data()
    connect()

    try:
        while True:
            print("\nMain menu:")
            print("  [1] Teach mode")
            print("  [2] Play mode (wait for PLC signal, then run)")
            print("  [3] List positions")
            print("  [4] List tasks")
            print("  [5] Home the arm")
            print("  [6] Quit")
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
            else:
                print("Invalid choice.")
    finally:
        save_data()
        disconnect()
        print("Disconnected.")
if __name__ == "__main__":
    main()