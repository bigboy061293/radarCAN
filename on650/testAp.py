from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import socket
import time
import math
import inspect
import threading
import os
import ctypes
import RPi.GPIO as GPIO
import subprocess, signal
import numpy as np
import can
from datetime import datetime
SIMULATOR_UDP = 'udp:127.0.0.1:14550'
UDP_IP_LOCAL = "127.0.0.1"
UDP_PORT = 2992
UDP_IP_650_TCP = 'tcp:192.168.0.210:20002'
RPI_SERIAL = '/dev/ttyS0'
can_interface ='can0'
bus = can.interface.Bus(can_interface, bustype = 'socketcan_native')
_DEFINE_LOG_ENABLE = 0

master = mavutil.mavlink_connection(UDP_IP_650_TCP, dialect = "ardupilotmega")
#master = mavutil.mavlink_connection(RPI_SERIAL, baud = 115200)
#mavutil.set_dialect("ardupilotmega")
if _DEFINE_LOG_ENABLE:
    now = datetime.now()
    currenTimeCreateFile = now.strftime("%H%M%S%d")
    currenTimeCreateFile = 'log/' + currenTimeCreateFile
    fileNow = open(currenTimeCreateFile,'w+')


def timeFrom00(timeInInt):
	miSec = timeInInt % 1000000
	timeInInt = timeInInt / 1000000
	sec = timeInInt % 100
	timeInInt = timeInInt / 100
	min = timeInInt % 100
	hour = timeInInt / 100
	timeReal = (hour * 3600 + min * 60 + sec) * 1000000 + miSec
	#print (hour)
	#print (min)
	#print (sec)
	#print (miSec)
	
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
        #print msg
        time.sleep(0.5)

distance = []
dumpy =[0,0,0,0]
for i in range(72):
    distance.append(1000)

connectTo650(master)


radarMaxRange = 99.99
radarMinRange = 0.5
numberOfSecs = 118
degree = 180
degreeOfSec = 1.53
radarBeam = 110
startBeam = 31
stopBeam = 149
startSec = 0
stopSec = 0
sectors = []

for i in range(numberOfSecs):
	sectors.append(0)
def initSecs(numberOfSecs):
	global startSec
	global stopSec
	for i in range(numberOfSecs):
		if i * degreeOfSec <= startBeam:
			sectors[i] = 0
		elif ((i+1) * degreeOfSec) >= stopBeam and i * degreeOfSec > 90:
			sectors[i ] = 0
		else:
			if startSec == 0:
				startSec = i
				
			sectors[i] = radarMaxRange
	stopSec = numberOfSecs - startSec - 1

#initSecsFreeToBoder(numberOfSecs)

startSec = startSec + 2
stopSec = stopSec -1
terrainHeight = 0 #(cm)

maxDist = 9999
distanceToStopinAuto = 400 #in cm
distanceToStopinManual = distanceToStopinAuto
distanceToWarn = 900
obstaclesAround = []
localSecsNum = 9


min0 = maxDist
min1 = maxDist
min2 = maxDist
min3 = maxDist
min4 = maxDist
min5 = maxDist
min6 = maxDist
min7 = maxDist

for i in range(360):
    obstaclesAround.append(maxDist)

def setLoiterSpeedAndGetACK(loiterSpeed):
    #while True:
        master.mav.param_set_send(
            master.target_system, master.target_component,
            b'LOIT_SPEED',
            int(loiterSpeed),
            mavutil.mavlink.MAV_PARAM_TYPE_UINT16)

def setWaypointSpeedAndGetACK(waypointSpeed):
    #while True:
        master.mav.param_set_send(
            master.target_system, master.target_component,
            b'WPNAV_SPEED',
            int(waypointSpeed),
            mavutil.mavlink.MAV_PARAM_TYPE_UINT16)            
            
def setLoiterAccelerateAndGetACK(loiterAccelerate):
    #while True:
        master.mav.param_set_send(
            master.target_system, master.target_component,
            b'LOIT_ACC_MAX',
            int(loiterAccelerate),
            mavutil.mavlink.MAV_PARAM_TYPE_UINT16)

partFront = np.zeros(360)
partLeft = np.zeros(360)
partRear = np.zeros(360)
partRight = np.zeros(360)

partFront.fill(maxDist)
partRight.fill(maxDist)
partLeft.fill(maxDist)
partRear.fill(maxDist)

min8Array =[maxDist, maxDist, maxDist, maxDist, maxDist, maxDist, maxDist, maxDist]
min8ArrayInter =[maxDist, maxDist, maxDist, maxDist, maxDist, maxDist, maxDist, maxDist]

lpfTap = 10
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

def latLongReturnAngle(latA, longA, latB, longB):
    temp = math.atan2(longB - longA, latB - latA) * (180.00 / math.pi)
    if temp <=0:
        temp = 360 + temp
    return temp
fixAngle = latLongReturnAngle(11.0509912,106.6653590,11.0512115,106.6645029)
print 'fixed angle', fixAngle
abcc = []
for i in range(360):
    abcc.append(int(maxDist))
def parseSec(setors):
    global data
    global sectors
    global msss
    global mss
    global beepPeriod
    global beepDuty
    global beeperUsedFor
    global terrainHeight
    global partFront
    global partRight
    global partRear
    global partLeft
    global min0
    global min1
    global min2
    global min3
    global min4
    global min5
    global min6
    global min7
    global min8Array
    global toAPLoitSpeed
    global toAPAccMax
    global toAPWpSpeed
    #now = datetime.now()
    #timeStart = long(now.strftime("%H%M%S%f"))
    #timeStart = timeFrom00(timeStart)
  
   # print dic_rear
   # return
    for i in dic_front:
        if dic_front[i] <= 0.01:
            dic_front[i] = maxDist
        if dic_front[i] >= 40:
            dic_front[i] = maxDist
    if _DEFINE_LOG_ENABLE:
        fileNow.write(str(timeFromBoot_ms) + ', front, ' + str (dic_front))
        fileNow.write("\r\n")
        
        
    for i in range(30,67):
        abcc[i] = int(float(dic_front[3]) * 100)
    for i in range(67,112):
        abcc[i] = int(float(dic_front[2]) * 100)
    for i in range(112,150):
        abcc[i] = int(float(dic_front[1]) * 100)
        
    partFront = np.asarray(abcc)
    
    
    ################################################################3
    
    
   
    for i in dic_right:
        if dic_right[i] <= 0.1:
            dic_right[i] = maxDist
        if dic_right[i] >= 40:
            dic_rear[i] = maxDist
    for i in range(30,67):
        abcc[i] = int(float(dic_right[3]) * 100)
    for i in range(67,112):
        abcc[i] = int(float(dic_right[2]) * 100)
    for i in range(112,150):
        abcc[i] = int(float(dic_right[1]) * 100)
        
    partRight = np.asarray(abcc)
    partRight = np.roll(partRight, 270)
    
    ################################################################
    
    #print dic_rear
    #return
    for i in dic_rear:
        if dic_rear[i] <= 0.1:
            dic_rear[i] = maxDist
        if dic_rear[i] >= 40:
            dic_rear[i] = maxDist
    for i in range(30,67):
        abcc[i] = int(float(dic_rear[3]) * 100)
    for i in range(67,112):
        abcc[i] = int(float(dic_rear[2]) * 100)
    for i in range(112,150):
        abcc[i] = int(float(dic_rear[1]) * 100)
        
    partRear = np.asarray(abcc)
    partRear = np.roll(partRear, 180)
    
    ################################################################
    
   
    for i in dic_left:
        if dic_left[i] <= 0.1:
            dic_left[i] = maxDist
        if dic_left[i] >= 40:
            dic_left[i] = maxDist
    for i in range(30,67):
        abcc[i] = int(float(dic_left[3]) * 100)
    for i in range(67,112):
        abcc[i] = int(float(dic_left[2]) * 100)
    for i in range(112,150):
        abcc[i] = int(float(dic_left[1]) * 100)
        
    partLeft = np.asarray(abcc)
    partLeft = np.roll(partLeft, 90)
        
    #print partFront.size
    #print partLeft.size
    #print partRight.size
    #print partRear.size
    #return
    cp1 = np.fmin(partFront,partRight)
    cp2 = np.fmin(partRear,partLeft)
    #now = datetime.now()
    #timeNow = long(now.strftime("%H%M%S%f"))
    obstaclesAround = np.minimum(cp1,cp2)
    for i in range(len(obstaclesAround)):
        if obstaclesAround[i] == 0:
            obstaclesAround[i] = maxDist
    #print obstaclesAround
    #return
    min8Array[1] = min(obstaclesAround[22:67])
    min8Array[0] = min(obstaclesAround[67:112])
    min8Array[7] = min(obstaclesAround[112:157])
    min8Array[6] = min(obstaclesAround[157:202])
    min8Array[5] = min(obstaclesAround[202:247])
    min8Array[4] = min(obstaclesAround[247:292])
    min8Array[3] = min(obstaclesAround[292:337])
    min8Array[2] = min(min(obstaclesAround[337:360]), min(obstaclesAround[0:22]))

    minAll = min(obstaclesAround[0:360])
    obstaclesAroundQuanti = np.asarray(obstaclesAround)
    #print partRear
    #return
   
   #if inLoiter and abs(attitudePitch) < 7 and abs(attitudeRoll) < 7 : #used for ket hop voi radar dia hinh
    if inLoiter:
        if rcAvoid > 1700:
        
            rcRollNorm = round((float(rcRoll) - 1500) /500,2)
            rcPitchNorm = round((float(rcPitch) - 1500)/500,2)
            #print '----'
            #print rcRoll, rcPitch
            rcHoriMag = math.sqrt(pow(rcRollNorm,2) + pow(rcPitchNorm,2))
            
            if rcRollNorm == 0.00 or rcRollNorm == -0.00:
                rcHoriAngle = math.copysign(90.00, rcPitchNorm)
            else:
                rcHoriAngle = (math.atan2(rcPitchNorm,rcRollNorm) * 180.00 /math.pi)
            if rcHoriAngle < 0:
                rcHoriAngle = rcHoriAngle + 359
            rcHoriAngle = int(round(rcHoriAngle))
    
            if rcHoriMag >= 0.05:# filtering magitude of RC stick
                
              
                if obstaclesAroundQuanti[rcHoriAngle] < distanceToWarn:
                    ratioToRc = float(obstaclesAroundQuanti[rcHoriAngle] - distanceToStopinManual) / float((distanceToWarn - distanceToStopinManual))
                  
                    if ratioToRc <= 0.00:
                        ratioToRc = 0
                    
                    
                    ltSpeed =  (ratioToRc*LOIT_SPEED)
                    if ltSpeed <= 10:
                        ltSpeed = 10
                    ltAcc =  (ratioToRc*LOIT_ACC_MAX)
                    if ltAcc <= 100:
                        ltAcc = 100
                        
                        
                    toAPLoitSpeed = ltSpeed

            else:
                toAPLoitSpeed = LOIT_SPEED
                toAPAccMax = LOIT_ACC_MAX
            
        else:
            toAPLoitSpeed = LOIT_SPEED
            toAPAccMax = LOIT_ACC_MAX
        
        
        
        #return 'DS'
        
    #else:
    #    for i in range(8):
    #        min8Array[i] = maxDist+1
    if inAuto or inRTL or inGuided:
      
        fooInd = int(fixAngle - ahrs2Yaw) + 90
        
        if fooInd >= 360:
            fooInd = fooInd - 360
        elif fooInd < 0:
            fooInd = fooInd + 360
        """
        # filter the fooInd
        minSecNum = maxDist
        tempAngle = 0
        fooIndx = fooInd
        for i in range(-44,45):
            
            if fooInd + i < 0:
                tempAngle = fooInd +i + 360
            elif fooInd + i >= 360:
                tempAngle = fooInd + i - 360
            else:
                tempAngle = fooInd + i
        
            if minSecNum >= obstaclesAround[tempAngle]:
                minSecNum = obstaclesAround[tempAngle]
                fooIndx = tempAngle
        
        fooInd = fooIndx
        
        ###################
        """
        """
        if obstaclesAround[fooInd] < distanceToWarn:
            ratioToRc = float(obstaclesAround[fooInd] - distanceToStopinAuto) / float((distanceToWarn - distanceToStopinAuto))
            if ratioToRc <= 0.00:
                ratioToRc = 0
            
            wpSpeed = ratioToRc * WP_SPEED
            if wpSpeed <= 10:
                wpSeed = 10
                
            toAPWpSpeed = wpSpeed
        else:
            toAPWpSpeed = WP_SPEED
        """
        print toAPWpSpeed
    
        if (obstaclesAround[fooInd] < distanceToStopinAuto):
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                5) # change to Loiter Mode
            print ('Obs in Auto, change to Loiter')
    else: 
        toAPWpSpeed = WP_SPEED
    #now = datetime.now()
    #timeEnd = long(now.strftime("%H%M%S%f"))
    #timeEnd = timeFrom00(timeEnd)
    #print timeEnd - timeStart
    #print timeEnd
    
foo = []
for i in range(numberOfSecs):
    foo.append(0)

stpThr1 = False


def setMaxDistanceManual():
    dstMan = 40
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b'AVOID_MARGIN',
        -1
        )  

    message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    return float(message['param_value']) * 100

#distanceToStopinManual = setMaxDistanceManual()
#distanceToStopinManual = 700
print 'Stop set', setMaxDistanceManual()
print 'Stop Alg', distanceToStopinManual

print 'Warn Alg', distanceToWarn

def get_LOIT_ACC_MAX():
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b'LOIT_ACC_MAX',
        -1
        )  

    message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    return float(message['param_value'])

def get_LOIT_SPEED():
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b'LOIT_SPEED',
        -1
        )  

    message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    return float(message['param_value'])

def get_WP_SPEED():
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b'WPNAV_SPEED',
        -1
        )  

    message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    return float(message['param_value'])
        
LOIT_ACC_MAX = get_LOIT_ACC_MAX()
LOIT_SPEED = get_LOIT_SPEED()
WP_SPEED = get_WP_SPEED()
toAPLoitSpeed = LOIT_SPEED
toAPAccMax = LOIT_ACC_MAX
toAPWpSpeed = WP_SPEED
print 'WP Speed Max, LT Speed Max', WP_SPEED,LOIT_SPEED

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
            global sectors
            stat = parseSec(sectors)
            #print timeFromBoot_ms
            #time.sleep(0.01)
        
    def stop(self):
        self._stop_event.set()

class readInputThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        
        #self.counter = counter
    def run(self):
        #global beeperDc
        global beepPeriod
        global beepDuty
        global beeperUsedFor
        try:
            while True:
               # print(os.getpid())
                command = raw_input()
                if command == 'echo':
                    print ('.^^.')
                elif command[0] == 'b':
                    #print(command)
                    try:
                        aaa = command.split(',')
                        if aaa[1] == 'a':
                            beeperUsedFor[0] = 1
                            beeperUsedFor[1] = 0
                            beeperUsedFor[2] = 0
                            continue
                        if aaa[1] == 'b':
                            beeperUsedFor[0] = 0
                            beeperUsedFor[1] = 1
                            beeperUsedFor[2] = 0
                            continue
                        if aaa[1] == 'c':
                            beeperUsedFor[0] = 0
                            beeperUsedFor[1] = 0
                            beeperUsedFor[2] = 1
                            continue
                        if aaa[1] == 'check':
                            print (beeperUsedFor)
                            continue
                        if beeperUsedFor[0]:
                            beepPeriod = float(aaa[1])
                            beepDuty = float(aaa[2])
                    except:
                        print ("Wrong beeper command")
                elif command == 't':
                    fileNow.close()
                    print ('Terminating process: ', os.getpid())
                    #thingsDoneTer()
                    os.kill(os.getpid(), signal.SIGKILL)
                   
                #time.sleep(0.5)
                
               
        except Exception: 
            print('Except from Input')
            #self.bucket.put(sys.exc_info())
            
beepPeriod = 1
beepDuty = 0

beeperUsedFor = [1,0,0]

def beepUntil(period, duty):
  
    
    GPIO.output(beeper, GPIO.HIGH)
   
    time.sleep(period * float(duty)/100)
    GPIO.output(beeper, GPIO.LOW)
   
    time.sleep(period * (1-float(duty)/100))
    
class beeperThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        #self.counter = counter
    def run(self):
        try:
            while True:
                beepUntil(beepPeriod,beepDuty)
                #time.sleep(1)
               #beepUntil(0.1)
               #pwm.ChangeDutyCycle(beeperDc)
               #time.sleep(0.5)
               #print beeperDc
               
        except Exception: 
            print ('bbbbb')


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
def initMessage(self):
    master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    0, mode_id, 0, 0, 0, 0, 0) 

class myThread2 (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        #self.counter = counter
    def run(self):
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
            #print msg
            if msg.get_type() == 'GPS_RAW_INT':
                gpsCog = msg.cog / 100
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

class sendMin (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        
    def run(self):
        global indexMin
        while True:
            #min8Array[0], min8Array[1], min8Array[2], min8Array[3], min8Array[4], min8Array[5], min8Array[6], min8Array[7], terrainHeight =  doLpf()
            setLoiterSpeedAndGetACK(toAPLoitSpeed)
            setWaypointSpeedAndGetACK(toAPWpSpeed)
            
            #setLoiterAccelerateAndGetACK(LOIT_ACC_MAX)
            time.sleep(0.005)
            
            # sending terrain....................
            
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
        while True:
            #now = datetime.now()
            #print now
            #timeStart = long(now.strftime("%H%M%S%f"))
            #timeStart = timeFrom00(timeStart)
            #if (timeStart - timeBefore) < 5000:
            #	continue
            
            #print timeStart
            #timeBefore = timeStart
            message = bus.recv() 

            if message.arbitration_id == 0X60B:
                y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
                x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
                _, mag = xyReturnSec(x,y,1)		
                sector = (message.data[6] >> 3) & 0X03		
            
                dic_front[sector] = round(mag,3)
                #print dic_front
                
            #elif message.arbitration_id == 0X60A:
                
            #    str_front = 'front, ' + str(dic_front)
            
            elif message.arbitration_id == 0X61B:	
                y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
                x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
                _, mag = xyReturnSec(x,y,1)		
                sector = (message.data[6] >> 3) & 0X03		
            
                dic_right[sector] = round(mag,3)
                #print dic_right
                
            #elif message.arbitration_id == 0X61A:
                
            #    str_right = 'right, ' + str(dic_right)
            
            
            elif message.arbitration_id == 0X62B:
                y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
                x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
                _, mag = xyReturnSec(x,y,1)		
                sector = (message.data[6] >> 3) & 0X03
                #print sector,round(mag,3)
                #dic_rear[sector] = round(mag,3)
                #if sector == 1:
                #    dic_rear[1] = round(mag,3)
                #elif sector == 2:
                #    dic_rear[2] = round(mag,3)
                #elif sector == 3:
                #    dic_rear[3] = round(mag,3)
                #dic_rear.update(sector = round(mag,3))
                #if sector in dic_rear:
                dic_rear[sector] = round(mag,3)
                    #print dic_rear
                #print dic_rear[1], dic_rear[2], dic_rear[3]
                #print sector
                #print 'cmmmmmm'
                #print 'cmmm'
                
                
            #elif message.arbitration_id == 0X62A:
                
            #    str_rear = 'rear, ' +  str(dic_rear)
                #print dic_rear
            
            elif message.arbitration_id == 0X63B:	
                y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
                x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
                _, mag = xyReturnSec(x,y,1)		
                sector = (message.data[6] >> 3) & 0X03		
            
                dic_left[sector] = round(mag,3)
                #print dic_left
            #elif message.arbitration_id == 0X63A:
                
            #    str_left = 'left, ' + str(dic_left)
            elif message.arbitration_id == 0X70C:
                terrainHeight = message.data[2]*256 + message.data[3]
                #print terrainHeight


thread1 = myThread(1, "Thread main")

thread2 = myThread2(2, "Receiving MSG")
#thread3 = beeperThread(3, "Buzzer control")
#thread4 = readInputThread(4, "Read input")
thread5 = sendMin(5, "Send min")

_threadReadRadarPolling = threadReadRadarPolling(6, "Thread main")

thread1.start()


thread2.start()
#thread2.join()

#thread3.start()
#thread1.join()

#thread4.start()
thread5.start()


_threadReadRadarPolling.start()
_threadReadRadarPolling.join()
#time.sleep(1)
