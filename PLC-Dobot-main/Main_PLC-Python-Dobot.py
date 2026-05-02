####################################################################################################################################
#
#   Imported Libraries
#
####################################################################################################################################

import pycomm3
from pycomm3 import LogixDriver
import cv2
import numpy as np
import DobotDllType as dType
import time
from tracker import *

####################################################################################################################################
#
#   Functions for Sorting Methods, Conveyor Speed, and PLC Tag Reading
#
####################################################################################################################################

def Random_Sort(count):
    print("Inside Random. Count: ", count)
    PLC_SysRunning, PLC_Stop, PLC_EStop, PLC_Conveyor, PLC_Color,Sort_Color_Array,Sort_Random,Sort_Manual,Manual_Color,Sort_Same,Sort_Array,PLC_Conveyor_Speed,Count_Reset = PLC_Read()
    if count < 5:
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 50, 250, 150, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 50, 250, -48 + count*25, 50, isQueued = 1)#Drops Block off
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 50, 250, 150, 50, isQueued = 1)#lifts straight up
        dType.dSleep(2000)
        dType.SetPTPCmd(api, 0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
        else:
            pass
        count += 1
        plc.write('Program:MainProgram.Block_Count', count)
    else:
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api, 1, 50, -250, 100, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api,0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
        else:
            pass
    return count

def Same_Sort(color, rc, gc, bc, yc):
    PLC_SysRunning, PLC_Stop, PLC_EStop, PLC_Conveyor, PLC_Color,Sort_Color_Array,Sort_Random,Sort_Manual,Manual_Color,Sort_Same,Sort_Array,PLC_Conveyor_Speed,Count_Reset = PLC_Read()
    if (color == 'Red') & (rc < 5):
        print("Gang2: inside function")
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api,dType.PTPMode.PTPMOVLXYZMode, 140, 225, 100, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 140, 225, -48 + rc*25, 50, isQueued = 1)#Drops Block off
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 140, 225, 150, 50, isQueued = 1)#lifts straight up
        dType.dSleep(2000)
        dType.SetPTPCmd(api, 0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        plc.write('Program:MainProgram.Block_Detected', 1)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
        else:
            pass
        rc += 1
    elif (color == 'Green') & (gc < 5):
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 140, 140, 100, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 140, 140, -48 + gc*25, 50, isQueued = 1)#Drops Block off
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 140, 140, 150, 50, isQueued = 1)#lifts straight up
        dType.dSleep(2000)
        dType.SetPTPCmd(api, 0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
        else:
            pass
        gc += 1
    elif (color == 'Blue') & (bc < 5):
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 190, 190, 100, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 190, 190, -48 + bc*25, 50, isQueued = 1)#Drops Block off
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 190, 190, 150, 50, isQueued = 1)#lifts straight up
        dType.dSleep(2000)
        dType.SetPTPCmd(api, 0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
        else:
            pass
        bc += 1
    elif (color == 'Yellow') & (yc < 5):
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 90, 225, 100, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 90, 225, -48 + yc*25, 50, isQueued = 1)#Drops Block off
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 90, 225, 150, 50, isQueued = 1)#lifts straight up
        dType.dSleep(2000)
        dType.SetPTPCmd(api, 0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
        else:
            pass
        yc += 1
    else:
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api, 0, 50, -250, 100, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api, 0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
        else:
            pass
    count=[rc,gc,bc,yc]
    count.sort()
    plc.write('Program:MainProgram.Block_Count', count[0])
    if rc >= 7:
        print("Maxed Out Red")
    if gc >= 7:
        print("Maxed Out Green")
    if bc >= 7:
        print("Maxed Out Blue")
    if yc >= 7:
        print("Maxed Out Yellow")
    return rc, gc, bc, yc

def Array_Sort(CA, CA_Count, color):
    PLC_SysRunning, PLC_Stop, PLC_EStop, PLC_Conveyor, PLC_Color,Sort_Color_Array,Sort_Random,Sort_Manual,Manual_Color,Sort_Same,Sort_Array,PLC_Conveyor_Speed,Count_Reset = PLC_Read()
    if color == 'Red':
        color='R'
    elif color == 'Green':
        color='G'
    elif color=='Blue':
        color='B'
    elif color=='Yellow':
        color='Y'
    else:
        pass
    if CA_Count <= 4:   
        CA=CA[CA_Count].upper()

    if (CA == color) & (CA_Count < 5):
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 50, 200, 100, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 50, 200, -48 + CA_Count*25, 50, isQueued = 1)#Drops Block off
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 50, 200, 150, 50, isQueued = 1)#lifts straight up
        dType.dSleep(2000)
        dType.SetPTPCmd(api, 0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
        CA_Count += 1
        plc.write('Program:MainProgram.Block_Count', CA_Count)
    else:
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 50, -250, 100, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api, 0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
    return CA_Count

def Manual_Sort(count, color, man_color):
    PLC_SysRunning, PLC_Stop, PLC_EStop, PLC_Conveyor, PLC_Color,Sort_Color_Array,Sort_Random,Sort_Manual,Manual_Color,Sort_Same,Sort_Array,PLC_Conveyor_Speed,Count_Reset = PLC_Read()
    print(color)
    print(man_color)
    print(count)
    if (man_color == color) & (count < 5):
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 125, 175, 100, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 125, 175, -48 + count*25, 50, isQueued = 1)#Drops Block off
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 125, 175, 150, 50, isQueued = 1)#lifts straight up
        dType.dSleep(2000)
        dType.SetPTPCmd(api, 0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
        else:
            pass
        count += 1
        plc.write('Program:MainProgram.Block_Count', count)
    else:
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 150, -250, 100, 50, isQueued = 1)#Goes over drop off location
        dType.dSleep(2000)
        dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued = 1)#Suction off
        dType.SetPTPCmd(api, 0, 243.5, 1, 50, 50, isQueued = 1)#Goes back to home
        dType.dSleep(3500)
        dType.SetQueuedCmdStartExec(api)
        if PLC_EStop[1] == 1:
            dType.SetQueuedCmdForceStopExec(api)
        elif PLC_Stop[1] == 1:
            dType.SetQueuedCmdStopExec(api)
        else:
            pass
    if count >= 5:
        print("Maxed Out")
    return count

def PLC_Read():
    with LogixDriver('192.168.222.51') as plc:                               
            PLC_SysRunning = plc.read('Program:MainProgram.System_Running')               
            PLC_Stop = plc.read('Program:MainProgram.Stop')                    
            PLC_EStop = plc.read('Program:MainProgram.ESTOP')
            PLC_Conveyor = plc.read('Program:MainProgram.Conv_Run')
            PLC_Color = plc.read('Program:MainProgram.Color')
            Sort_Color_Array = plc.read('Program:MainProgram.Sorting{5}')
            Sort_Random = plc.read('Program:MainProgram.Random')
            Sort_Manual = plc.read('Program:MainProgram.Manual')
            Manual_Color = plc.read('Program:MainProgram.Manual_Color')
            Sort_Same = plc.read('Program:MainProgram.Same')
            Sort_Array = plc.read('Program:MainProgram.Array')
            PLC_Conveyor_Speed = plc.read('Program:MainProgram.Conv_Speed')
            Manual_Color = plc.read('Program:MainProgram.Color')
            Sort_Same = plc.read('Program:MainProgram.Same')
            Sort_Array = plc.read('Program:MainProgram.Array_S')
            PLC_Conveyor_Speed = plc.read('Program:MainProgram.Conv_Speed')
            Count_Reset = plc.read('Program:MainProgram.Reset_Count')
            return PLC_SysRunning, PLC_Stop, PLC_EStop, PLC_Conveyor, PLC_Color,Sort_Color_Array,Sort_Random,Sort_Manual,Manual_Color,Sort_Same,Sort_Array,PLC_Conveyor_Speed,Count_Reset

def Dob_Pick(x,y):
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 250, y, 90, 50, isQueued = 1)
    dType.dSleep(1000)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, 5, 50, isQueued = 1)
    dType.dSleep(2000)
    dType.SetEndEffectorSuctionCup(api, 1, 1, isQueued = 1) # suction on
    dType.dSleep(1000)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 250, 73, 100, 50, isQueued = 1)    # lifts straight up
    PLC_SysRunning, PLC_Stop, PLC_EStop, PLC_Conveyor, PLC_Color,Sort_Color_Array,Sort_Random,Sort_Manual,Manual_Color,Sort_Same,Sort_Array,PLC_Conveyor_Speed,Count_Reset = PLC_Read()
    dType.SetQueuedCmdStartExec(api)
    if PLC_Stop[1] == 1:
        dType.SetQueuedCmdStopExec(api)
    elif PLC_EStop[1] == 1:
        dType.SetQueuedCmdForceStopExec
    else:
        pass
    BS = 1
    return BS

def Con_Run(PLC):
    STEP_PER_CRICLE = 360.0 / 1.8 * 10.0 * 16.0
    MM_PER_CRICLE = 3.1415926535898 * 36.0
    vel = float(PLC) * STEP_PER_CRICLE / MM_PER_CRICLE
    return int(vel)

def Count_Reset_Fun():
    with LogixDriver('192.168.222.51') as plc:     
        plc.write('Program:MainProgram.Block_Count', 0)
    Block_Count = 0
    red_count = 0
    green_count = 0
    blue_count = 0
    yellow_count = 0
    Array_Count = 0
    Manual_Count = 0
    return Block_Count,red_count,green_count,blue_count,yellow_count,Array_Count,Manual_Count


####################################################################################################################################
#
#   Dobot Connection and initial speed and home parameters
#
####################################################################################################################################

# Dobot connection status
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

# Connect Dobot
api = dType.load()
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

# Setting Dobot speeds and home coordinates
dType.SetQueuedCmdClear(api)
dType.SetHOMEParams(api, 243.5, 1, 50, 50, isQueued = 1)
dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)
dType.SetHOMECmd(api, temp=0, isQueued = 1)
dType.SetQueuedCmdStartExec(api)
time.sleep(23)

while True:
####################################################################################################################################
#
#   Read in all PLC Inputs for System Start
#
####################################################################################################################################
    with LogixDriver('192.168.222.51') as plc:                               
            plc.write('Program:MainProgram.Block_Detected', 0)
            PLC_SysRunning, PLC_Stop, PLC_EStop, PLC_Conveyor, PLC_Color,Sort_Color_Array,Sort_Random,Sort_Manual,Manual_Color,Sort_Same,Sort_Array,PLC_Conveyor_Speed,Count_Reset = PLC_Read()
            # Initalize counter blocks
            break_shape = 0
            start = 1
            Block_Color = 'NULL'
            Chosen_Color = ''
            y1=90
            array=[]
            Sort_Color_Array = []
            Block_Count,red_count,green_count,blue_count,yellow_count,Array_Count,Manual_Count = Count_Reset_Fun()
            plc.write('Program:MainProgram.Block_Count', 0)

####################################################################################################################################
#
#   Start of the System Depending on PLC input
#
####################################################################################################################################
           
            while PLC_SysRunning[1] == 1:
                print("System is Running.")
                # Reread PLC Inputs for Updates
                plc.write('Program:MainProgram.Block_Detected', 0)
                PLC_SysRunning, PLC_Stop, PLC_EStop, PLC_Conveyor, PLC_Color,Sort_Color_Array,Sort_Random,Sort_Manual,Manual_Color,Sort_Same,Sort_Array,PLC_Conveyor_Speed,Count_Reset = PLC_Read()
                if PLC_EStop[1] == 1:
                    dType.SetQueuedCmdForceStopExec(api)
                elif PLC_Stop[1] == 1:
                    dType.SetQueuedCmdStopExec(api)

                if Count_Reset[1] == 1:
                    Block_Count,red_count,green_count,blue_count,yellow_count,Array_Count,Manual_Count = Count_Reset_Fun()


####################################################################################################################################
#
#   Conveyor start and stop depending on PLC inputs
#
####################################################################################################################################
                if PLC_Conveyor[1] == 1:
                    vel = Con_Run(PLC_Conveyor_Speed[1])
                    dType.SetEMotor(api, 1, 1, -(vel), isQueued = 0)

####################################################################################################################################
#
#   Start of the Computer Vision While loop
#
####################################################################################################################################
                while break_shape == 0:
                    if start == 1:
                        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
                        start = 0
                    x = -1
                    y = 0
                    _, frame = cap.read()
                    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                    PLC_SysRunning, PLC_Stop, PLC_EStop, PLC_Conveyor, PLC_Color,Sort_Color_Array,Sort_Random,Sort_Manual,Manual_Color,Sort_Same,Sort_Array,PLC_Conveyor_Speed,Count_Reset = PLC_Read()
                    vel = Con_Run(PLC_Conveyor_Speed[1])
                    if PLC_SysRunning[1] == 0:
                        dType.SetEMotor(api, 1, 0, 0, isQueued = 0)
                    else:
                        dType.SetEMotor(api, 1, 1, -(vel), isQueued = 0)
                    
                    if PLC_Stop[1] == 1:
                        dType.SetQueuedCmdStopExec
                    elif PLC_EStop[1] == 1:
                        dType.SetQueuedCmdForceStopExec
                    else:
                        pass

                    if Count_Reset[1] == 1:
                        Block_Count,red_count,green_count,blue_count,yellow_count,Array_Count,Manual_Count = Count_Reset_Fun()
                    #Define Red Array
                    low_red = np.array([160,50,50])
   
                    high_red = np.array([180,255,255])
   
                    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
   
                    red = cv2.bitwise_and(frame, frame, mask=red_mask)

                    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                    #Define Green Array
                    low_green = np.array([50,100,100])
   
                    high_green = np.array([70,255,255])
   
                    green_mask = cv2.inRange(hsv_frame, low_green, high_green)
   
                    green = cv2.bitwise_and(frame, frame, mask=green_mask)

                    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   
                    #Define Blue Array
                    low_blue = np.array([100,150,0])

                    high_blue = np.array([140,255,255])
   
                    blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
   
                    blue = cv2.bitwise_and(frame, frame, mask=blue_mask)

                    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                    #Define Yellow Array
                    low_yellow = np.array([10,100,100])
   
                    high_yellow = np.array([38,255,255])
   
                    yellow_mask = cv2.inRange(hsv_frame, low_yellow, high_yellow)
   
                    yellow = cv2.bitwise_and(frame, frame, mask=yellow_mask)

                    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


                    for cnt in red_contours:
                        # Calculate area and remove small elements
                        area = cv2.contourArea(cnt)

                        if area > 12000:
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                            x = int(x + w/2)
                            y = int(y + h/2)
                            Block_Color = 'Red'
                            plc.write('Program:MainProgram.Current_Color', 'Red')
            

                    for cnt in green_contours:
                        # Calculate area and remove small elements
                        area = cv2.contourArea(cnt)

                        if area > 12000:
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                            x = int(x + w/2)
                            y = int(y + h/2)
                            Block_Color = 'Green'
                            plc.write('Program:MainProgram.Current_Color', 'Green')
            

                    for cnt in blue_contours:
                        # Calculate area and remove small elements
                        area = cv2.contourArea(cnt)

                        if area > 12000:
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
                            x = int(x + w/2)
                            y = int(y + h/2)
                            Block_Color = 'Blue'
                            plc.write('Program:MainProgram.Current_Color', 'Blue')

                    for cnt in yellow_contours:
                        # Calculate area and remove small elements
                        area = cv2.contourArea(cnt)

                        if area > 10000:
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)
                            x = int(x + w/2)
                            y = int(y + h/2)
                            Block_Color = 'Yellow'
                            plc.write('Program:MainProgram.Current_Color', 'Yellow')
                   
                    if y > 220:
                        print(x,y)
                        plc.write('Program:MainProgram.Block_Detected', 1)
                        PLC_SysRunning, PLC_Stop, PLC_EStop, PLC_Conveyor, PLC_Color,Sort_Color_Array,Sort_Random,Sort_Manual,Manual_Color,Sort_Same,Sort_Array,PLC_Conveyor_Speed,Count_Reset = PLC_Read()
                        if PLC_Conveyor[1] == 0:
                            dType.SetEMotor(api, 1, 0, 0, isQueued = 0) 

                        if 220 < y <= 250:
                            y1 = 73
                        elif 250 < y <= 300:
                            y1 = 68
                            print(y1)
                        elif 300 < y <= 350:
                            y1 = 53
                            print(y1)
                        elif 350 < y <= 400:
                            y1 = 48
                            print(y1)

                        #### Conveyor Speed Decision for y coordinate
                        print(vel)
                        if 0 < vel <= 2264:
                            y1 = y1 + 12
                        elif 2264 < vel <= 4245:
                            y1 = y1 + 7
                        elif 4245 < vel <= 7072:
                            y1 = y1 + 5
                        elif 7072 < vel <= 8490:
                            y1 = y1
                        elif 8490 < vel <= 11035:
                            y1 = y1 - 3
                        elif 11035 < vel <= 12733:
                            y1 = y1 - 5
                        elif 12733 < vel <= 14713:
                            y1 = y1 - 10
                        elif 14713 < vel <=15562:
                            y1 = y1 - 10
                        elif 15562 < vel <=16128:
                            y1 = y1 - 13
                        else:
                            y1 = y1 - 16
                        
                        if 0 < x <= 150:
                            print(y)
                            cap.release()
                            cv2.destroyAllWindows()
                            dType.SetQueuedCmdClear(api)
                            x = x/4+ 220
                            break_shape = Dob_Pick(x,y1)
                        elif 150 < x <= 240:
                            print(y)
                            cap.release()
                            cv2.destroyAllWindows()
                            dType.SetQueuedCmdClear(api)
                            x = x/4.15 + 210 
                            break_shape = Dob_Pick(x,y1)
                        elif 240 < x <= 300:
                            print(y)
                            cap.release()
                            cv2.destroyAllWindows()
                            dType.SetQueuedCmdClear(api)
                            x = x/4.42 + 210 
                            break_shape = Dob_Pick(x,y1)
                        elif 300 < x <= 350:
                            print(y)
                            cap.release()
                            cv2.destroyAllWindows()
                            dType.SetQueuedCmdClear(api)
                            x = x/4.75 + 210 
                            break_shape = Dob_Pick(x,y1)
                        elif 350 < x <= 405:
                            print(y)
                            cap.release()
                            cv2.destroyAllWindows()
                            dType.SetQueuedCmdClear(api)
                            x = x/5 + 210
                            break_shape = Dob_Pick(x,y1)
                        elif x > 405:
                            cap.release()
                            cv2.destroyAllWindows()
                            dType.SetQueuedCmdClear(api)
                            x = x/5.1 + 210
                            break_shape = Dob_Pick(x,y1)
                break_shape = 0
                print(x,y1)
                start = 1
                print(Block_Color)
        
####################################################################################################################################
#
#   Sorting method executed based on PLC
#
####################################################################################################################################
                if Sort_Manual[1] == 1:
                    plc.write('Program:MainProgram.Current_Sort', 'Manual Sort')
                    if Manual_Color[1] == 1:
                        Chosen_Color = 'Red'
                    elif Manual_Color[1] == 2:
                        Chosen_Color = 'Green'
                    elif Manual_Color[1] == 3:
                        Chosen_Color = 'Yellow'
                    else:
                        Chosen_Color = 'Blue'
                    Manual_Count = Manual_Sort(Manual_Count, Block_Color, Chosen_Color)
                elif Sort_Same[1] == 1:
                    plc.write('Program:MainProgram.Current_Sort', 'Same Sort')
                    red_count, green_count, blue_count, yellow_count = Same_Sort(Block_Color, red_count, green_count, blue_count, yellow_count)
                elif Sort_Array[1] == 1:
                    plc.write('Program:MainProgram.Current_Sort', 'Array Sort')
                    if Array_Count ==0:
                        for i in range(5):
                            array.append(Sort_Color_Array[1][i])
                    Array_Count = Array_Sort(array,Array_Count,Block_Color)
                else:
                   plc.write('Program:MainProgram.Current_Sort', 'Random Sort')
                   Block_Count = Random_Sort(Block_Count)
