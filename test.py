import DobotDllType as dType
import time

api = dType.load()
state = dType.ConnectDobot(api, "COM", 115200)
print("Connected:", state)

# 1. Check for alarms FIRST
try:
    alarms = dType.GetAlarmsState(api)
    print("Alarms:", alarms)
    dType.ClearAllAlarmsState(api)
    print("Alarms cleared.")
except Exception as e:
    print("Alarm check failed:", e)

# 2. Try selecting sub-device explicitly
# Try BOTH 0 and 1 — flip this number on a second run if no movement
SUB_DEVICE = 0
try:
    dType.SetDeviceWithIndex(api, SUB_DEVICE)
    print(f"Selected sub-device {SUB_DEVICE}")
except AttributeError:
    print("SetDeviceWithIndex not available in this DLL version.")
except Exception as e:
    print("SetDeviceWithIndex error:", e)

dType.SetQueuedCmdClear(api)
dType.SetQueuedCmdStartExec(api)

# 3. Home the arm — this physically moves it to the home position.
#    If THIS doesn't move, the problem isn't the PTP target, it's deeper.
print("\n>>> HOMING — watch the arm <<<")
home_idx = dType.SetHOMECmd(api, 0, 1)[0]
print("Home queued, idx =", home_idx)
deadline = time.time() + 30
while dType.GetQueuedCmdCurrentIndex(api)[0] < home_idx:
    if time.time() > deadline:
        print("Home timed out after 30s.")
        break
    time.sleep(0.5)
print("Pose after home:", dType.GetPose(api))

# 4. Now try a small PTP move
print("\n>>> PTP MOVE — watch the arm <<<")
idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 200, 0, 50, 0, 1)[0]
print("PTP queued, idx =", idx)
deadline = time.time() + 15
while dType.GetQueuedCmdCurrentIndex(api)[0] < idx:
    if time.time() > deadline:
        print("PTP timed out.")
        break
    time.sleep(0.2)
print("Pose after PTP:", dType.GetPose(api))

dType.DisconnectDobot(api)