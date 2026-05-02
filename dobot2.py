# dobot3.py — Pick-and-place + teaching + PLC + waits + conveyor + multi-series
import DobotDllType as dType
import json
import os
import sys
import time

# ============================================================
# CONNECTION
# ============================================================
COM_PORT = "COM5"
BAUD = 115200
api = None

# ============================================================
# STORAGE
# ============================================================
COORDS_FILE = "dobot2_coords.json"
positions = {}
series_list = [[]]          # list of task lists; each inner list is one series
active_series_idx = 0       # which series is being edited (0-based)

# ============================================================
# MOTION
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
# CONVEYOR
# ============================================================
STP1_INDEX = 0
STP2_INDEX = 1
CONVEYOR_DEFAULT_SPEED = 10000

# ============================================================
# PLAY BEHAVIOR
# ============================================================
LOOP_FOREVER = True   # True = after the last series, loop back to series 1.
                      # False = stop after one full pass through all series.

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

def set_conveyor(index, speed):
    try:
        dType.SetEMotor(api, index, 1, int(speed), 0)
    except Exception as e:
        print(f"Conveyor {index} command failed: {e}")

def stop_all_conveyors():
    set_conveyor(STP1_INDEX, 0)
    set_conveyor(STP2_INDEX, 0)

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
        print(f"Connection failed (status {state[0]}).")
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
        stop_all_conveyors()
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
    global positions, series_list, active_series_idx
    if os.path.exists(COORDS_FILE):
        try:
            with open(COORDS_FILE, "r") as f:
                content = f.read().strip()
            if not content:
                raise ValueError("file is empty")
            data = json.loads(content)
            positions = data.get("positions", {})
            if "series" in data and data["series"]:
                series_list = data["series"]
            elif "tasks" in data:
                # legacy single-task-list format
                series_list = [data["tasks"]]
            else:
                series_list = [[]]
            for series in series_list:
                for t in series:
                    if "type" not in t:
                        t["type"] = "pick_place"
            active_series_idx = 0
            n_tasks = sum(len(s) for s in series_list)
            print(f"Loaded {len(positions)} positions, {len(series_list)} series ({n_tasks} tasks total) from {COORDS_FILE}")
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
    series_list = [[]]
    active_series_idx = 0

def save_data():
    with open(COORDS_FILE, "w") as f:
        json.dump({"positions": positions, "series": series_list}, f, indent=2)
    n_tasks = sum(len(s) for s in series_list)
    print(f"Saved {len(positions)} positions, {len(series_list)} series ({n_tasks} tasks) to {COORDS_FILE}")

# ============================================================
# ACTIVE SERIES HELPERS
# ============================================================
def cur_tasks():
    return series_list[active_series_idx]

# ============================================================
# TEACHING / POSITIONS
# ============================================================
def teach_position(name):
    print("\n" + "=" * 50)
    print(f"  TEACHING: '{name}'")
    print("=" * 50)
    print("  1. Hold the button on top of the dobot arm.")
    print("  2. Drag the arm to the target position.")
    print("  3. Release the button.")
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

def delete_position(name):
    if name in positions:
        del positions[name]
        save_data()
        print(f"Deleted position '{name}'.")
    else:
        print(f"'{name}' not found.")

def test_move(name):
    if name not in positions:
        print(f"'{name}' not found.")
        return
    x, y, z, r = positions[name]
    print(f"Moving to '{name}'.  Will hover at Z={SAFE_HOVER_Z} first.")
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
# SERIES MANAGEMENT
# ============================================================
def list_series():
    print(f"\n--- Series ({len(series_list)} total) ---")
    for i, s in enumerate(series_list, 1):
        marker = " <-- active" if (i - 1) == active_series_idx else ""
        print(f"  Series {i}: {len(s)} tasks{marker}")

def switch_active_series(n_str):
    global active_series_idx
    try:
        n = int(n_str)
    except ValueError:
        print("Series number must be an integer.")
        return
    if not (1 <= n <= len(series_list)):
        print(f"Series number out of range (1..{len(series_list)}).")
        return
    active_series_idx = n - 1
    print(f"Active series is now {n}.")

def add_new_series():
    series_list.append([])
    save_data()
    print(f"Added empty series. Now {len(series_list)} series total.")

def delete_active_series():
    global active_series_idx
    if len(series_list) == 1:
        series_list[0].clear()
        print("Only one series exists; cleared its tasks (didn't delete the series itself).")
    else:
        del series_list[active_series_idx]
        if active_series_idx >= len(series_list):
            active_series_idx = len(series_list) - 1
        print(f"Deleted series. Now on series {active_series_idx + 1} of {len(series_list)}.")
    save_data()

# ============================================================
# TASK MANAGEMENT (operates on cur_tasks() = active series)
# ============================================================
def list_tasks():
    tasks = cur_tasks()
    if not tasks:
        print(f"Series {active_series_idx + 1} has no tasks.")
        return
    print(f"\n--- Series {active_series_idx + 1} tasks ({len(tasks)}) ---")
    for i, t in enumerate(tasks, 1):
        ttype = t.get("type", "pick_place")
        if ttype == "wait":
            print(f"  [{i}] WAIT {t.get('seconds', 0)}s")
        elif ttype == "conveyor":
            idx = t.get("index", 0)
            spd = t.get("speed", 0)
            dur = t.get("duration", 0)
            name = "STP1" if idx == 0 else "STP2"
            if dur > 0:
                print(f"  [{i}] CONVEYOR {name} speed={spd} for {dur}s")
            elif spd == 0:
                print(f"  [{i}] CONVEYOR {name} STOP")
            else:
                print(f"  [{i}] CONVEYOR {name} speed={spd} (start, no auto-stop)")
        else:
            print(f"  [{i}] PICK '{t['pick']}' -> PLACE '{t['place']}'")

def add_task():
    list_positions()
    if len(positions) < 2:
        print("Need at least two saved positions.")
        return
    pick = input("Pick position name: ").strip()
    if pick not in positions:
        print(f"'{pick}' not found.")
        return
    place = input("Place position name: ").strip()
    if place not in positions:
        print(f"'{place}' not found.")
        return
    cur_tasks().append({"type": "pick_place", "pick": pick, "place": place})
    save_data()
    print(f"Added pick-place to series {active_series_idx + 1} (position {len(cur_tasks())}).")

def add_wait_task(seconds_str, position_str=None):
    try:
        sec = float(seconds_str)
    except ValueError:
        print(f"Invalid duration: '{seconds_str}'.")
        return
    if sec < 0:
        print("Duration must be non-negative.")
        return
    new_task = {"type": "wait", "seconds": sec}
    tasks = cur_tasks()
    if position_str is None:
        tasks.append(new_task)
        idx = len(tasks)
    else:
        try:
            pos = int(position_str)
        except ValueError:
            print(f"Invalid position: '{position_str}'.")
            return
        pos = max(1, min(len(tasks) + 1, pos))
        tasks.insert(pos - 1, new_task)
        idx = pos
    save_data()
    print(f"Added wait ({sec}s) to series {active_series_idx + 1} at position {idx}.")

def add_conveyor_task(speed_str, duration_str=None, index_str=None, position_str=None):
    try:
        speed = int(float(speed_str))
    except ValueError:
        print(f"Invalid speed: '{speed_str}'.")
        return
    duration = 0.0
    if duration_str is not None:
        try:
            duration = float(duration_str)
        except ValueError:
            print(f"Invalid duration: '{duration_str}'.")
            return
        if duration < 0:
            print("Duration must be non-negative.")
            return
    index = 0
    if index_str is not None:
        try:
            index = int(index_str)
        except ValueError:
            print(f"Invalid index: '{index_str}'.")
            return
        if index not in (0, 1):
            print("Index must be 0 (STP1) or 1 (STP2).")
            return
    new_task = {"type": "conveyor", "index": index, "speed": speed, "duration": duration}
    tasks = cur_tasks()
    if position_str is None:
        tasks.append(new_task)
        pos = len(tasks)
    else:
        try:
            pos = int(position_str)
        except ValueError:
            print(f"Invalid position: '{position_str}'.")
            return
        pos = max(1, min(len(tasks) + 1, pos))
        tasks.insert(pos - 1, new_task)
    save_data()
    name = "STP1" if index == 0 else "STP2"
    if duration > 0:
        print(f"Added conveyor ({name} speed={speed} for {duration}s) to series {active_series_idx + 1} at position {pos}.")
    elif speed == 0:
        print(f"Added conveyor STOP ({name}) to series {active_series_idx + 1} at position {pos}.")
    else:
        print(f"Added conveyor ({name} speed={speed}) to series {active_series_idx + 1} at position {pos}.")

def manual_conveyor(speed_str, duration_str=None, index_str=None):
    try:
        speed = int(float(speed_str))
    except ValueError:
        print(f"Invalid speed: '{speed_str}'.")
        return
    index = 0
    if index_str is not None:
        try:
            index = int(index_str)
        except ValueError:
            print(f"Invalid index: '{index_str}'.")
            return
    name = "STP1" if index == 0 else "STP2"
    if duration_str is not None:
        try:
            duration = float(duration_str)
        except ValueError:
            print(f"Invalid duration: '{duration_str}'.")
            return
        print(f"  Conveyor {name} ON at {speed} pps for {duration}s...")
        set_conveyor(index, speed)
        try:
            time.sleep(duration)
        finally:
            set_conveyor(index, 0)
        print(f"  Conveyor {name} stopped.")
    else:
        if speed == 0:
            print(f"  Stopping conveyor {name}.")
        else:
            print(f"  Conveyor {name} ON at {speed} pps (no auto-stop).")
        set_conveyor(index, speed)

def move_task(from_str, to_str):
    tasks = cur_tasks()
    try:
        from_idx = int(from_str) - 1
        to_idx = int(to_str) - 1
    except ValueError:
        print("Both arguments must be integers.")
        return
    if not (0 <= from_idx < len(tasks)):
        print(f"'from' index out of range (1..{len(tasks)}).")
        return
    to_idx = max(0, min(len(tasks) - 1, to_idx))
    task = tasks.pop(from_idx)
    tasks.insert(to_idx, task)
    save_data()
    print(f"Moved task {from_idx + 1} -> {to_idx + 1}")
    list_tasks()

def delete_task(idx_str):
    tasks = cur_tasks()
    try:
        idx = int(idx_str) - 1
        if 0 <= idx < len(tasks):
            removed = tasks.pop(idx)
            save_data()
            print(f"Removed task from series {active_series_idx + 1}: {removed}")
        else:
            print("Index out of range.")
    except ValueError:
        print("Invalid index.")

# ============================================================
# PICK AND PLACE
# ============================================================
def do_pick(name):
    x, y, z, r = positions[name]
    print(f"    PICK '{name}' ({x:.1f}, {y:.1f}, {z:.1f})")
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
    print(f"    PLACE '{name}' ({x:.1f}, {y:.1f}, {z:.1f})")
    if DRY_RUN:
        return
    ptp(x, y, SAFE_HOVER_Z, r)
    ptp(x, y, z, r)
    suction(False)
    queued_wait(500)
    idx = ptp(x, y, SAFE_HOVER_Z, r)
    wait_for_cmd(idx)

# ============================================================
# PLAY MODE — runs each series on its own PLC trigger
# ============================================================
def play():
    if not series_list or all(len(s) == 0 for s in series_list):
        print("No tasks in any series. Add tasks first.")
        return

    missing = set()
    for series in series_list:
        for t in series:
            if t.get("type", "pick_place") == "pick_place":
                if t.get("pick") not in positions:
                    missing.add(t.get("pick"))
                if t.get("place") not in positions:
                    missing.add(t.get("place"))
    if missing:
        print(f"Missing positions: {', '.join(sorted(s for s in missing if s))}")
        return

    n_series = len(series_list)
    print(f"\n=== PLAY: {n_series} series, LOOP_FOREVER={LOOP_FOREVER} ===")
    print("Press Ctrl+C to stop.")

    cycle_num = 1
    try:
        while True:
            for series_idx, series in enumerate(series_list):
                if not series:
                    print(f"\n--- Cycle {cycle_num}, Series {series_idx + 1}/{n_series} (empty, skipping) ---")
                    continue

                print(f"\n--- Cycle {cycle_num}, Series {series_idx + 1}/{n_series} ({len(series)} tasks) ---")

                if USE_PLC:
                    wait_for_plc_start()

                if "start" in positions:
                    x, y, z, r = positions["start"]
                    wait_for_cmd(ptp(x, y, z, r))

                try:
                    for i, t in enumerate(series, 1):
                        ttype = t.get("type", "pick_place")
                        if ttype == "wait":
                            sec = t.get("seconds", 0)
                            print(f"  [Task {i}/{len(series)}] WAIT {sec}s")
                            time.sleep(sec)
                        elif ttype == "conveyor":
                            idx = t.get("index", 0)
                            speed = t.get("speed", 0)
                            dur = t.get("duration", 0)
                            name = "STP1" if idx == 0 else "STP2"
                            if dur > 0:
                                print(f"  [Task {i}/{len(series)}] CONVEYOR {name} speed={speed} for {dur}s")
                                set_conveyor(idx, speed)
                                try:
                                    time.sleep(dur)
                                finally:
                                    set_conveyor(idx, 0)
                            elif speed == 0:
                                print(f"  [Task {i}/{len(series)}] CONVEYOR {name} STOP")
                                set_conveyor(idx, 0)
                            else:
                                print(f"  [Task {i}/{len(series)}] CONVEYOR {name} speed={speed} (running)")
                                set_conveyor(idx, speed)
                        else:
                            print(f"  [Task {i}/{len(series)}] {t['pick']} -> {t['place']}")
                            do_pick(t["pick"])
                            do_place(t["place"])
                finally:
                    stop_all_conveyors()

                if "start" in positions:
                    x, y, z, r = positions["start"]
                    wait_for_cmd(ptp(x, y, z, r))

                print(f"=== Series {series_idx + 1} DONE ===")

                if USE_PLC:
                    signal_plc_done()

            cycle_num += 1
            if not LOOP_FOREVER:
                print("\nAll series complete (single pass).")
                break
    except KeyboardInterrupt:
        print("\nPlay loop interrupted by Ctrl+C.")
    finally:
        stop_all_conveyors()

# ============================================================
# MENUS
# ============================================================
def teach_menu():
    while True:
        active_count = len(cur_tasks())
        print("\n" + "-" * 50)
        print(f"  TEACH MENU  [active: series {active_series_idx + 1}/{len(series_list)}, "
              f"{active_count} tasks]")
        print("-" * 50)
        print("  --- Positions ---")
        print("  t <name>                Teach a position by name")
        print("  l                       List saved positions")
        print("  d <name>                Delete a position")
        print("  m <name>                Test-move to a position")
        print("  --- Tasks (active series) ---")
        print("  +                       Add a pick-place task at end")
        print("  w <sec> [pos]           Add a wait task")
        print("  c <speed> [dur] [idx]   Add a conveyor task")
        print("  cc <speed> [dur] [idx]  Run conveyor NOW (test only)")
        print("  mv <from> <to>          Move a task")
        print("  -                       Remove a task by index")
        print("  ls                      List tasks in active series")
        print("  --- Series ---")
        print("  sl                      List all series")
        print("  sg <n>                  Switch active series to N")
        print("  s+                      Add new empty series at end")
        print("  s-                      Delete (or clear) the active series")
        print("  --- Other ---")
        print("  s                       Save now")
        print("  p                       Switch to play mode")
        print("  q                       Back to main menu")
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
        elif action == "w":
            if not arg:
                arg = input("Wait seconds (and optional position): ").strip()
            wparts = arg.split() if arg else []
            if len(wparts) == 1:
                add_wait_task(wparts[0])
            elif len(wparts) >= 2:
                add_wait_task(wparts[0], wparts[1])
            else:
                print("Usage: w <seconds> [position]")
        elif action == "c":
            if not arg:
                arg = input("Speed [duration] [index]: ").strip()
            cparts = arg.split() if arg else []
            if len(cparts) == 1:
                add_conveyor_task(cparts[0])
            elif len(cparts) == 2:
                add_conveyor_task(cparts[0], cparts[1])
            elif len(cparts) == 3:
                add_conveyor_task(cparts[0], cparts[1], cparts[2])
            elif len(cparts) >= 4:
                add_conveyor_task(cparts[0], cparts[1], cparts[2], cparts[3])
            else:
                print("Usage: c <speed> [duration] [index] [position]")
        elif action == "cc":
            if not arg:
                arg = input("Speed [duration] [index]: ").strip()
            ccparts = arg.split() if arg else []
            if len(ccparts) == 1:
                manual_conveyor(ccparts[0])
            elif len(ccparts) == 2:
                manual_conveyor(ccparts[0], ccparts[1])
            elif len(ccparts) >= 3:
                manual_conveyor(ccparts[0], ccparts[1], ccparts[2])
            else:
                print("Usage: cc <speed> [duration] [index]")
        elif action == "mv":
            if not arg:
                arg = input("Move <from> <to>: ").strip()
            mparts = arg.split() if arg else []
            if len(mparts) >= 2:
                move_task(mparts[0], mparts[1])
            else:
                print("Usage: mv <from> <to>")
        elif action == "-":
            if not arg:
                list_tasks()
                arg = input("Task index to remove: ").strip()
            if arg:
                delete_task(arg)
        elif action == "ls":
            list_tasks()
        elif action == "sl":
            list_series()
        elif action == "sg":
            if not arg:
                arg = input("Switch to series number: ").strip()
            if arg:
                switch_active_series(arg)
        elif action == "s+":
            add_new_series()
        elif action == "s-":
            confirm = input(f"Delete/clear series {active_series_idx + 1}? (y/N) > ").strip().lower()
            if confirm == "y":
                delete_active_series()
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
    print("  DOBOT 3 — Multi-series pick & place + PLC + conveyor")
    print("=" * 50)
    load_data()
    connect()

    try:
        while True:
            print(f"\nMain menu (currently {len(series_list)} series defined):")
            print("  [1] Teach mode")
            print("  [2] Play mode (waits for PLC trigger before each series)")
            print("  [3] List positions")
            print("  [4] List series")
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
                list_series()
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