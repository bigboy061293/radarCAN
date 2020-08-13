from pymavlink import mavutil
import threading
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import time
import can
import speedProfileGen_Lin
import os
import subprocess, signal
from datetime import datetime
_DEFINE_LOG_ENABLE = 1
RPI_SERIAL = '/dev/ttyS0'

master = mavutil.mavlink_connection(RPI_SERIAL, baud = 115200)
# max speed, max speed at, stop at, time step
a = speedProfileGen_Lin.linProfile(10, 10, 15, 0.01, "Max Speed 10 at 10s, stop at 15s, step 0.01")

def connectTo650(master):
    msg = None
    while not msg:
        master.mav.ping_send(
            time.time(), # Unix time
            0, # Ping number
            0, # Request ping of all systems
            0 # Request pping of all components
        )
        msg = master.recv_match()
        print msg
        time.sleep(0.5)

connectTo650(master)



master.mav.command_long_send(
     master.target_system,
     master.target_component,
     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL ,
     0,
     65, 10000, 0, 0, 0, 0, 0)
    
# master.mav.command_long_send(
     # master.target_system,
     # master.target_component,
     # mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL ,
     # 0,
     # 35, 10000, 0, 0, 0, 0, 0)
    
    
    
inLoiter = False
inAlthold = False            
inAuto = False
inGuided = False
inRTL = False
currLat = 0
currLon = 0
currAlt = 0

rcRoll = 1500
rcPitch = 1500
rcJoyYaw = 0
rcJoyThrottle = 0
rcAvoid = 1000
rcTerain = 1000
rcThrottle = 1000 
rcYaw = 1500
timeBoot = 0
attitudeRoll = 0
attitudePitch = 0
rcButtonPress = 0
rcPre = 0
rcNow = 0
rcNowButt = 0
rcPreButt = 0
startTesting = 1

inModeTestManual = 0

timeFromBoot_ms = 0
jsX = 0
jsY = 0
jsZ = 0
jsYaw = 0
class myThread2 (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        #self.counter = counter
    def run(self):
        # global sectors
        global inLoiter
        global inAlthold
        global inAuto
        global inGuided
        global currLat
        global currLon
        global currAlt
        global inRTL
        global rcRoll
        global rcPitch
        global rcButtonPress
        global rcNow
        global rcPre
        global rcNowButt
        global rcPreButt
        global startTesting
        global rcThrottle
        global rcYaw
        global timeFromBoot_ms
        global rcJoyYaw
        global rcJoyThrottle
        global inModeTestManual
        # global rcTerain
        # global rcAvoid
        # global timeBoot
        # global attitudeRoll
        # global attitudePitch
        while True:
        
            msg = master.recv_match()
            if not msg:
                continue
            #print msg
            if msg.get_type() == 'HEARTBEAT':
                if msg.custom_mode !=0 :
                
                    if msg.custom_mode == 5: #in Loiter mode
                        inLoiter = True
                    else:
                        inLoiter = False
                    
                    
                    if msg.custom_mode == 2: #in Althold mode
                        inAlthold = True
                    else:
                        inAlthold = False
                        
                    if msg.custom_mode == 3: #in Auto mode
                        inAuto = True
                    else:
                        inAuto = False
                    
                    if msg.custom_mode == 4: #in Guided mode
                        inGuided = True
                    else:
                        inGuided = False
                        
                    if msg.custom_mode == 6: #in RTL mode
                        inRTL = True
                    else:
                        inRTL = False
                continue
                
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                currLat = msg.lat
                currLon = msg.lon
                currAlt = msg.relative_alt
                continue
            if msg.get_type() == 'RC_CHANNELS':
                #print msg
                #print msg.chan15_raw
                timeBoot = msg.time_boot_ms
                rcRoll = msg.chan1_raw
                rcPitch = msg.chan2_raw
                rcJoyThrottle = msg.chan13_raw
                rcJoyYaw = msg.chan14_raw
                rcTerain = msg.chan8_raw
                rcAvoid = msg.chan9_raw
                rcThrottle = msg.chan3_raw
                rcYaw = msg.chan4_raw
                
                timeFromBoot_ms = msg.time_boot_ms
                if msg.chan15_raw - rcPre > 2:
                    if rcButtonPress > 5:
                        rcButtonPress = 0
                        
                    rcButtonPress+= 1
                rcPre = msg.chan15_raw
                if msg.chan15_raw > 1500:
                    inModeTestManual = 1
                else:
                    inModeTestManual = 0
                #print inModeTestManual
            
                if msg.chan16_raw - rcPreButt > 1:
                    startTesting = 1 - startTesting
                    
                rcPreButt = msg.chan16_raw
                #print msg.chan15_raw, msg.chan16_raw
                
                if rcRoll < 1000:
                    rcRoll = 1000
                elif rcRoll > 2000:
                    rcRoll = 2000
                    
                if rcPitch < 1000:
                    rcPitch = 1000
                elif rcPitch > 2000:
                    rcPitch = 2000
                    
                # if rcJoyLR < 1000:
                    # rcJoyLR = 1000
                # elif rcJoyLR > 2000:
                    # rcJoyLR = 2000
                    
                # if rcJoyFB < 1000:
                    # rcJoyFB = 1000
                # elif rcJoyFB > 2000:
                    # rcJoyFB = 2000
                    
                if rcThrottle < 1000:
                    rcThrottle = 1000
                elif rcThrottle > 2000:
                    rcThrottle = 2000
                    
                if rcYaw < 1000:
                    rcYaw = 1000
                elif rcYaw > 2000:
                    rcYaw = 2000
                continue
            if msg.get_type() == 'ATTITUDE':
                attitudePitch = msg.pitch
                attitudeRoll = msg.roll
                continue

def setSpeedSimpleFrameBodyFLU(vx,vy,vz):
        master.mav.set_position_target_local_ned_send(
                        0,
                        master.target_system,
                        master.target_component,
                        13,#MAV_FRAME_BODY_FLU 
                        0b110111000111, # control velocity
                        0, #N
                        0, #E 
                        0, #D
                        vx, #vx
                        vy, #vy
                        vz, #vz
                        0, #ax
                        0, #ay
                        0, #az
                        0, #yaw angle
                        0 #yaw rate
                        )
        
def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)
if _DEFINE_LOG_ENABLE:
    now = datetime.now()
    currenTimeCreateFile = now.strftime("%H%M%S%d")
    currenTimeCreateFile = 'log/' + currenTimeCreateFile
    fileNow = open(currenTimeCreateFile,'w+')
    fileNow.write("MANUAL_CONTROL")
    fileNow.write("\r\n")
class controlViaMav (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
        global jsX
        global jsY
        global jsZ
        global jsYaw
        while True:
                #translate value in, to
                jsX = int(translate(rcPitch,800,2100,-1000,1000))
                jsY = int(translate(rcRoll,800,2100,-1000,1000))
                jsZ = int(translate(rcJoyThrottle,800,2100,-1000,1000))
                jsYaw = int(translate(rcJoyYaw,800,2100,-1000,1000))
                bt = [1,0]
                #print jsX, jsY, jsZ, jsYaw
                #print rcPitch, rcRoll, rcThrottle, rcYaw
                if _DEFINE_LOG_ENABLE:
                        fileNow.write(str(timeFromBoot_ms) + ':' + 
                        str(jsX) + ',' + str(rcPitch) + ',' +
                        str(jsY) + ',' + str(rcRoll) + ',' +
                        str(jsZ) + ',' + str(rcJoyThrottle) + ',' +
                        str(jsYaw) + ',' + str(rcJoyYaw))
                        fileNow.write("\r\n")
                
                mss = mavlink2.MAVLink_manual_control_message(
                        master.target_system,
                    jsX, #min distance
                    jsY, #max distance
                    jsZ, # current
                    jsYaw, #type
                    1)
                #print inModeTestManual
                if inModeTestManual:
                    
                    
                    master.mav.send(mss)
                    print jsX, jsY, jsZ, jsYaw
                
                time.sleep(0.01)
                
        

thread2 = myThread2(2, "Receiving MSG")
threadControlViaMav = controlViaMav(3,"Control Via Mav")

thread2.start()
threadControlViaMav.start()

while True:
    pass
        
