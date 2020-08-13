from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

import time
import math
import inspect
import threading
import os
import ctypes

import vector2D

import subprocess, signal
import numpy as np
import can
from datetime import datetime
SIMULATOR_UDP = 'udp:127.0.0.1:14550'
UDP_IP_LOCAL = "127.0.0.1"
UDP_PORT = 2992
UDP_IP_650_TCP = 'tcp:192.168.0.210:20002'
RPI_SERIAL = '/dev/ttyS0'
can_interface = 'can0'
bus = can.interface.Bus(can_interface, bustype = 'socketcan_native')
_DEFINE_LOG_ENABLE = 0
_USING_TELEMETRY_2 = 0
master = mavutil.mavlink_connection(UDP_IP_650_TCP, dialect = "ardupilotmega")
#master = mavutil.mavlink_connection(RPI_SERIAL, baud = 115200)
#mavutil.set_dialect("ardupilotmega")
if _DEFINE_LOG_ENABLE:
    now = datetime.now()
    currenTimeCreateFile = now.strftime("%H%M%S%d")
    currenTimeCreateFile = 'log/' + currenTimeCreateFile
    fileNow = open(currenTimeCreateFile,'w+')

def latLongReturnAngle(latA, longA, latB, longB):
    temp = math.atan2(longB - longA, latB - latA) * (180.00 / math.pi)
    if temp <=0:
        temp = 360 + temp
    return temp
    
print latLongReturnAngle(11.0509364,106.6654202,11.0514102,106.6636714)

def timeFrom00(timeInInt):
	miSec = timeInInt % 1000000
	timeInInt = timeInInt / 1000000
	sec = timeInInt % 100
	timeInInt = timeInInt / 100
	min = timeInInt % 100
	hour = timeInInt / 100
	timeReal = (hour * 3600 + min * 60 + sec) * 1000000 + miSec
	return(timeReal)
    
def xyReturnSec(x,y, degreeDec):
	
		abc = math.atan2(y,x) * (180.00 / math.pi)
		abc = int(abc * 100)
		while not abc % (degreeDec*100) == 0:
			
			#print (abc * 100) % degreeOfSec
			abc = abc + 1
			#print abc
		seccc = round(abc / (degreeDec*100))
		mag = math.sqrt(pow(x,2) + pow(y,2))
		return seccc, mag

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
        
dumpy =[0,0,0,0]

if _USING_TELEMETRY_2:
    connectTo650(master)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        35, 10000, 0, 0, 0, 0, 0) #RC_CHANNELS
    #MAV_CMD_SET_MESSAGE_INTERVAL
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        24, 10000, 0, 0, 0, 0, 0) #GPS
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        182, 10000, 0, 0, 0, 0, 0) #AHRS3
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        30, 10000, 0, 0, 0, 0, 0) #ATTITUDE


terrainHeight = 0 #(cm)

maxDist = 9999
distanceToStopinAuto = 400 #in cm
distanceToStopinManual = distanceToStopinAuto
distanceToWarn = 700
obstaclesAround = []
localSecsNum = 9

for i in range(360):
    obstaclesAround.append(maxDist)

min8Array =[maxDist, maxDist, maxDist, maxDist, maxDist, maxDist, maxDist, maxDist]
min8ArrayInter =[maxDist, maxDist, maxDist, maxDist, maxDist, maxDist, maxDist, maxDist]

lpfTap = 5
lpf = np.zeros((9,lpfTap))

#return
def doLpf():
    global lpf
    #for i in range(lpfTap):
    min_0 = sum(lpf[0,:])/lpfTap
    min_1 = sum(lpf[1,:])/lpfTap
    min_2 = sum(lpf[2,:])/lpfTap
    min_3 = sum(lpf[3,:])/lpfTap
    min_4 = sum(lpf[4,:])/lpfTap
    min_5 = sum(lpf[5,:])/lpfTap
    min_6 = sum(lpf[6,:])/lpfTap
    min_7 = sum(lpf[7,:])/lpfTap
    height_ = sum(lpf[8,:])/lpfTap
    for i in range(8):
        lpf[i,lpfTap-1] = min8Array[i]
        for j in range(1,lpfTap):
            if j == lpfTap-1:
                break
            
            
            lpf[i,j] = lpf[i,j+1]
            #print i,j
            #print lpf[i,j]
            #print ' ----'
    lpf[8,lpfTap-1] = terrainHeight
    for j in reversed(range(lpfTap-1)):
        
        if j == 0:
            break
        
        lpf[8,j-1] = lpf[8,j]
   
    return min_0,min_1,min_2,min_3,min_4,min_5,min_6,min_7,height_

lpfCOG = np.zeros(lpfTap)

def doLpfCOG():
    global lpfCOG
    
    returnCog = sum(lpfCOG[:])/lpfTap
    
    for j in range(0,lpfTap-1):        
        lpfCOG[j] = lpfCOG[j+1]
    lpfCOG[lpfTap-1] = gpsCog
   
    return returnCog

latLongArrayLength = 4
latArray = np.zeros(latLongArrayLength)
longArray = np.zeros(latLongArrayLength)


def updateGPSArray():
    global longArray
    global latArray
    for j in range(0,latLongArrayLength-1):        
        latArray[j] = latArray[j+1]
        longArray[j] = longArray[j+1]
        
    longArray[latLongArrayLength-1] = gpsLong
    latArray[latLongArrayLength-1] = gpsLat
    
    longMean = sum(longArray[:])/latLongArrayLength
    latMean = sum(latArray[:])/latLongArrayLength
    #print gpsLong, gpsLat
    return latLongReturnAngle(latArray[0],longArray[0],latArray[latLongArrayLength-1],longArray[latLongArrayLength-1])
    
    
    
    
    
    
class myThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        #self.counter = counter
    def run(self):
        #try:
        while True:
            global stpThr1
            
        
    def stop(self):
        self._stop_event.set()
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

timeFromBoot_ms = 0
timeFromBoot_us = 0

gpsCog = 0
ahrs2Yaw = 0

rollSpeed = 0
pitchSpeed = 0

gpsLong = 0
gpsLat = 0
class myThread2 (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        #self.counter = counter
    def run(self):
        global gpsLong
        global gpsLat
        global rollSpeed
        global pitchSpeed
        global sectors
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
        global rcJoyLR
        global rcJoyFB
        global rcTerain
        global rcAvoid
        global timeBoot
        global attitudeRoll
        global attitudePitch
        global timeFromBoot_ms
        global timeFromBoot_us
        global gpsCog
        global ahrs2Yaw
        while True:
        
            msg = master.recv_match()
            if not msg:
                continue
        
            if msg.get_type() == 'ATTITUDE':
                rollSpeed = msg.rollspeed * 180 / 3.14159
                pitchSpeed = msg.pitchspeed * 180 / 3.14159
                #print rollSpeed, pitchSpeed
            if msg.get_type() == 'GPS_RAW_INT':
                gpsCog = msg.cog / 100
                gpsLat = msg.lat 
                gpsLong = msg.lon 
                #print 'a'
                
            if msg.get_type() == 'AHRS2':
                ahrs2Yaw = msg.yaw * 180 / 3.141593
                if ahrs2Yaw >= 360:
                    ahrs2Yaw = ahrs2Yaw - 360
                if ahrs2Yaw <0:
                    ahrs2Yaw = ahrs2Yaw + 360
                    
            if msg.get_type() == 'VIBRATION':
                timeFromBoot_us = msg.time_usec
            if msg.get_type() == 'HEARTBEAT':
                if msg.custom_mode !=0 :
                    #https://ardupilot.org/dev/docs/apmcopter-adding-a-new-flight-mode.html
                    #17 is break
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
                timeBoot = msg.time_boot_ms
                rcRoll = msg.chan1_raw
                rcPitch = msg.chan2_raw
                rcJoyLR = msg.chan13_raw
                rcJoyFB = msg.chan14_raw
                rcTerain = msg.chan8_raw
                rcAvoid = msg.chan9_raw
                timeFromBoot_ms = msg.time_boot_ms
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
                
            
indexMin = 0
dic_front = {
1: 0,
2: 0,
3: 0}
dic_right = {
1: 0,
2: 0,
3: 0}
dic_rear = {
1: 0,
2: 0,
3: 0}
dic_left = {
1: 0,
2: 0,
3: 0}




class threadReadRadarPolling (threading.Thread):
    
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
        global dic_front
        global dic_right
        global dic_rear
        global dic_left
        global terrainHeight
        global min8Array
        global obstaclesAround
    
        while True:
            message = bus.recv() 
            if message.arbitration_id == 0X60B:
                y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
                x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
                _, mag = xyReturnSec(x,y,1)		
                mag = mag * 100
                sector = (message.data[6] >> 3) & 0X03		
            
                dic_front[sector] = round(mag,3)
                if sector == 1:
                    for i in range(112,157):
                        obstaclesAround[i] = min( int(mag), dic_left[3])
                elif sector == 2:
                    for i in range(67,122):
                        obstaclesAround[i] = round(int(mag),3)
                elif sector == 3:
                    for i in range(22,67):
                        obstaclesAround[i] = min (int(mag), dic_right[1])
                    
            
            elif message.arbitration_id == 0X61B:	
                y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
                x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
                _, mag = xyReturnSec(x,y,1)		
                mag = mag * 100
                sector = (message.data[6] >> 3) & 0X03		
            
                dic_right[sector] = round(mag,3)
                
                if sector == 1:
                    for i in range(22,67):
                        obstaclesAround[i] = min( int(mag), dic_front[3])
                elif sector == 2:
                    for i in range(0,22):
                        obstaclesAround[i] = round(int(mag),3)
                    for i in range(337,360):
                        obstaclesAround[i] = round(int(mag),3)
                elif sector == 3:
                    for i in range(292,337):
                        obstaclesAround[i] = min( int(mag), dic_rear[1])
            
            elif message.arbitration_id == 0X62B:
                y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
                x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
                _, mag = xyReturnSec(x,y,1)		
                mag = mag * 100
                sector = (message.data[6] >> 3) & 0X03

                dic_rear[sector] = round(mag,3)
                if sector == 1:
                    for i in range(292,337):
                        obstaclesAround[i] = min( int(mag), dic_right[3])
                elif sector == 2:
                    for i in range(247,292):
                        obstaclesAround[i] = round(int(mag),3)
                elif sector == 3:
                    for i in range(202,247):
                        obstaclesAround[i] = min( int(mag), dic_left[1])
                
            
            elif message.arbitration_id == 0X63B:	
                y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
                x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
                _, mag = xyReturnSec(x,y,1)		
                mag = mag * 100
                sector = (message.data[6] >> 3) & 0X03		
            
                dic_left[sector] = round(mag,3)
                if sector == 1:
                    for i in range(202,247):
                        obstaclesAround[i] = min( int(mag), dic_front[3])
                elif sector == 2:
                    for i in range(157,202):
                        obstaclesAround[i] = round(int(mag),3)
                elif sector == 3:
                    for i in range(112,157):
                        obstaclesAround[i] = min( int(mag), dic_rear[3])

            elif message.arbitration_id == 0X70C:
                terrainHeight = message.data[2]*256 + message.data[3]
            min8Array[1] = min(obstaclesAround[22:67])
            min8Array[0] = min(obstaclesAround[67:112])
            min8Array[7] = min(obstaclesAround[112:157])
            min8Array[6] = min(obstaclesAround[157:202])
            min8Array[5] = min(obstaclesAround[202:247])
            min8Array[4] = min(obstaclesAround[247:292])
            min8Array[3] = min(obstaclesAround[292:337])
            min8Array[2] = min(min(obstaclesAround[337:360]), min(obstaclesAround[0:22]))



indexMin = 0

class sendMin (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        
    def run(self):
        global indexMin
        while True:
           
            mss = mavlink2.MAVLink_distance_sensor_message(
                    0, #time
                    0, #min distance
                    6000, #max distance
                    terrainHeight, # current
                    3, #type
                    4, #id
                    25, #ori
                    0, #cova
                    0, #fov
                    0, #fov
                    dumpy
                    )
            # sending terrain end  ....................
            
            
            master.mav.send(mss)
            time.sleep(0.005)
            
            indexMin = indexMin + 1
            if indexMin >= 8:
                indexMin = 0
            #if indexMin % 2 == 0:
            singleThreadSendMin(indexMin)
            

def singleThreadSendMin(inx):
    msss = mavlink2.MAVLink_distance_sensor_message(
                    0, #time
                    10, #min distance
                    maxDist, #max distance
                    min8Array[inx], # current
                    3, #type
                    1, #id
                    inx, #ori
                    255)
    master.mav.send(msss)
    
    time.sleep(0.01)

moveVector = vector2D.simpleVectorVelocity(1,1)
class move(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        
    def run(self):
        
        while True:
            
            #time.sleep(0.1)
            pass



thread1 = myThread(1, "Thread main")

thread2 = myThread2(2, "Receiving MSG")
thread5 = sendMin(5, "Send min")

_threadReadRadarPolling = threadReadRadarPolling(6, "Thread main")
_threadMove = move(3, "Move")
thread1.start()


thread2.start()
_threadMove.start()
thread5.start()

_threadReadRadarPolling.start()
_threadReadRadarPolling.join()

