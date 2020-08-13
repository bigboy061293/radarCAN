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
a = speedProfileGen_Lin.linProfile(10, 10, 15, 0.01, "Max Speed 10m/s at 10s, stop at 15s, step 0.01")
b = speedProfileGen_Lin.linProfile(10, 7, 9.5, 0.01, "Max Speed 10m/s at 7s, stop at 9.5s, step 0.01")
c = speedProfileGen_Lin.linProfile(10, 5, 6, 0.01, "Max Speed 10m/s at 5s, stop at 6s, step 0.01")
d = speedProfileGen_Lin.linProfile(10, 2, 2.5, 0.01, "Max Speed 10m/s at 2s, stop at 2.5s, step 0.01")
e = speedProfileGen_Lin.linProfile(10, 1, 1.1, 0.01, "Max Speed 10m/s at 1s, stop at 1.1s, step 0.01")
f = speedProfileGen_Lin.linProfile(10, 10, 10.1, 0.01, "Max Speed 10m/s at 1s, stop at 10.1s, step 0.01")
g = speedProfileGen_Lin.linProfile(10, 10, 10.1, 0.05, "Max Speed 10m/s at 1s, stop at 10.1s, step 0.05")

a7 = speedProfileGen_Lin.linProfile(7, 10, 15, 0.01, "Max Speed 7m/s at 10s, stop at 15s, step 0.01")
b7 = speedProfileGen_Lin.linProfile(7, 7, 9.5, 0.01, "Max Speed 7m/s at 7s, stop at 9.5s, step 0.01")
c7 = speedProfileGen_Lin.linProfile(7, 5, 6, 0.01, "Max Speed 7m/s at 5s, stop at 6s, step 0.01")
d7 = speedProfileGen_Lin.linProfile(7, 2, 2.5, 0.01, "Max Speed 7m/s at 2s, stop at 2.5s, step 0.01")
e7 = speedProfileGen_Lin.linProfile(7, 1, 1.1, 0.01, "Max Speed 7m/s at 1s, stop at 1.1s, step 0.01")

a2 = speedProfileGen_Lin.linProfile(2, 10, 15, 0.01, "Max Speed 2m/s at 10s, stop at 15s, step 0.01")
b2 = speedProfileGen_Lin.linProfile(2, 7, 9.5, 0.01, "Max Speed 2m/s at 7s, stop at 9.5s, step 0.01")
c2 = speedProfileGen_Lin.linProfile(2, 5, 6, 0.01, "Max Speed 2m/s at 5s, stop at 6s, step 0.01")
d2 = speedProfileGen_Lin.linProfile(2, 2, 2.5, 0.01, "Max Speed 2m/s at 2s, stop at 2.5s, step 0.01")
e2 = speedProfileGen_Lin.linProfile(2, 1, 1.1, 0.01, "Max Speed 2m/s at 1s, stop at 1.1s, step 0.01")

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
    
master.mav.command_long_send(
     master.target_system,
     master.target_component,
     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL ,
     0,
     65, 10000, 0, 0, 0, 0, 0)
    
    
    
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
rcJoyLR = 0
rcjoyFB = 0
rcAvoid = 1000
rcTerain = 1000
timeBoot = 0
attitudeRoll = 0
attitudePitch = 0
rcButtonPress = 0
rcPre = 0
rcNow = 0
rcNowButt = 0
rcPreButt = 0
startTesting = 1
timeFromBoot_ms = 0
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
        global timeFromBoot_ms
        # global rcJoyLR
        # global rcJoyFB
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
                #print msg.chan15_raw
                timeBoot = msg.time_boot_ms
                rcRoll = msg.chan1_raw
                rcPitch = msg.chan2_raw
                rcJoyLR = msg.chan13_raw
                rcJoyFB = msg.chan14_raw
                rcTerain = msg.chan8_raw
                rcAvoid = msg.chan9_raw
                timeFromBoot_ms = msg.time_boot_ms
                if msg.chan15_raw - rcPre > 2:
                    if rcButtonPress > 5:
                        rcButtonPress = 0
                        
                    rcButtonPress+= 1
                rcPre = msg.chan15_raw
            
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
                    
                if rcJoyLR < 1000:
                    rcJoyLR = 1000
                elif rcJoyLR > 2000:
                    rcJoyLR = 2000
                    
                if rcJoyFB < 1000:
                    rcJoyFB = 1000
                elif rcJoyFB > 2000:
                    rcJoyFB = 2000
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
                        8,#MAV_FRAME_BODY https://github.com/ArduPilot/pymavlink/blob/f55377066e5f5dba2c63b7a5d116454b1b0ce303/generator/swift/Tests/MAVLinkTests/Testdata/common.xml
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
        
def doExam(a):
        if _DEFINE_LOG_ENABLE:
                now = datetime.now()
                currenTimeCreateFile = now.strftime("%H%M%S%d")
                currenTimeCreateFile = 'log/' + currenTimeCreateFile
                fileNow = open(currenTimeCreateFile,'w+')
                fileNow.write(a.nameProfile)
                fileNow.write("\r\n")
                
        print a.nameProfile
        
        for i in range(a.profileLin.size):
                setSpeedSimpleFrameBodyFLU(a.profileLin[i],0,0)
                print a.profileLin[i]
                if _DEFINE_LOG_ENABLE:
                        fileNow.write(str(timeFromBoot_ms) + ':' + str(i*a.timeStep) + ':' + str(a.profileLin[i]))
                        fileNow.write("\r\n")
                time.sleep(a.timeStep)
        if _DEFINE_LOG_ENABLE:
                fileNow.close()
                
                
class controlViaMav (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
        doExam(g)
        os.kill(os.getpid(), signal.SIGKILL)
                
        

thread2 = myThread2(2, "Receiving MSG")
threadControlViaMav = controlViaMav(3,"Control Via Mav")

thread2.start()
threadControlViaMav.start()

while True:
    pass
        
