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
from datetime import datetime
SIMULATOR_UDP = 'udp:127.0.0.1:14550'
UDP_IP_LOCAL = "127.0.0.1"
UDP_PORT = 2992
UDP_IP_650_TCP = 'tcp:192.168.0.210:20002'
RPI_SERIAL = '/dev/ttyS0'

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP_LOCAL, UDP_PORT))
#sock.setblocking(1)
master = mavutil.mavlink_connection(UDP_IP_650_TCP, dialect = "ardupilotmega")
#master = mavutil.mavlink_connection(RPI_SERIAL, baud = 115200)
#mavutil.set_dialect("ardupilotmega")


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
class simpleVectorVelocity:
    vx = 0
    vy = 0
    vxN = 0
    vyN = 0
    def __init__(self, vx, vy):
        self.vx = vx
        self.vy = vy
    def magnitudeIs(self):
        return math.sqrt(math.pow(self.vx,2) + math.pow(self.vy,2))
    def phaseToXIs(self):
        return math.atan2(self.vy, self.vx) * 180 / math.pi

    def normalize(self):
        #newMag = self.magnitudeIs()
        self.vyN = math.sin(self.phaseToXIs() * math.pi / 180) 
        self.vxN = math.cos(self.phaseToXIs()* math.pi / 180) 
    def vxNorm(self):
        
        self.vxN = math.cos(self.phaseToXIs()* math.pi / 180) 
        return (self.vxN)
    def vyNorm(self):
        self.vyN = math.sin(self.phaseToXIs() * math.pi / 180) 
        return (self.vyN)

def connectTo650(master):
    msg = None
    while not msg:
        master.mav.ping_send(
            time.time(), # Unix time
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
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
    
def initSecsFreeToBoder(numberOfSecs):
	global startSec
	global stopSec
	for i in range(numberOfSecs):
		if i * degreeOfSec <= startBeam:
			sectors[i] = radarMaxRange
		elif ((i+1) * degreeOfSec) >= stopBeam and i * degreeOfSec > 90:
			sectors[i ] = radarMaxRange
		else:
			if startSec == 0:
				startSec = i
			sectors[i] = radarMaxRange
	stopSec = numberOfSecs - startSec

def initSectorsSeventyTwo(self):
    return 0
#startSec = startSec + 2
#stopSec = stopSec -1
print numberOfSecs
initSecsFreeToBoder(numberOfSecs)

startSec = startSec + 2
stopSec = stopSec -1
terrainHeight = 0 #(cm)

maxDist = 9999
distanceToStopinAuto = 500 #in cm
distanceToStopinManual = distanceToStopinAuto
distanceToWarn = 1500
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
def setLoiterAccelerateAndGetACK(loiterAccelerate):
    #while True:
        master.mav.param_set_send(
            master.target_system, master.target_component,
            b'LOIT_ACC_MAX',
            int(loiterAccelerate),
            mavutil.mavlink.MAV_PARAM_TYPE_UINT16)

        #message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        #print message
        #print 'in the set loiter', message['param_value']
        #if message['param_value'] == int(loiterSpeed):
        #    return 'S - Cnged'

            
ring360 = []
partFront = np.zeros(360)
partLeft = np.zeros(360)
partRear = np.zeros(360)
partRight = np.zeros(360)


partFront.fill(maxDist)
partRight.fill(maxDist)
partLeft.fill(maxDist)
partRear.fill(maxDist)

partFrontTemp = np.zeros(360)
partLeftTemp = np.zeros(360)
partRearTemp = np.zeros(360)
partRightTemp = np.zeros(360)


partFrontTemp.fill(maxDist)
partRightTemp.fill(maxDist)
partLeftTemp.fill(maxDist)
partRearTemp.fill(maxDist)

min8Array =[maxDist, maxDist, maxDist, maxDist, maxDist, maxDist, maxDist, maxDist]

for i in range(360):
    ring360.append(maxDist)
  

def parseSec(setors):
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
    now = datetime.now()
    timeStart = long(now.strftime("%H%M%S%f"))
    timeStart = timeFrom00(timeStart)
    #while True:
    #data,address = sock.recvfrom(100) # buffer size is 1024 bytes
        #while data:
        #    print data
        #    data = sock.recvfrom(100)
        #print data
        #if data != None:
        #    break
    
    data,address = sock.recvfrom(100)

    #print data
    
    #return
    abc = data.split(',')
    #print abc
    if abc[0] == 'E' and abc[1] == ' nomr72':
        #print ('MR72 disconnected')
        return 'E'
    elif abc[0] == 'E' and abc[1] == ' nonra24':
        #print ('NRA24 disconnected')
        return 'E'
    elif abc[0] == 'H':
        terrainHeight = float(abc[1])
        #print(terrainHeight)
        if terrainHeight >= 5000:
            terrainHeight = 5000
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
        master.mav.send(mss)
        return 'H'
    else:
        abcc = []
        for i in range(1,len(abc)-1):
            abcc.append(int(float(abc[i]) * 100))
            #aa.append(i*100)
            if abcc[i-1] >= 9999:
                abcc[i-1] = maxDist
            #print abcc[i-1]
        #print abcc
        
        if abc[0] == 'front':
            #pass
            abcc.reverse()
            #partFront[0:36] = np.asarray(abcc)
            #print partFront
            partFrontTemp[0:33] = abcc[0]
            partFrontTemp[33:78] = abcc[1]
            partFrontTemp[78:112] = abcc[2]
            partFront = np.roll(partFrontTemp, 33)
            
           
           
            
          
        if abc[0] == 'left':
            
            partLeftTemp[0:33] = abcc[0]
            partLeftTemp[33:78] = abcc[1]
            partLeftTemp[78:112] = abcc[2]
            partLeft = np.roll(partLeftTemp, 123)
            
            
        if abc[0] == 'rear':
            abcc.reverse()
            partRearTemp[0:33] = abcc[0]
            partRearTemp[33:78] = abcc[1]
            partRearTemp[78:112] = abcc[2]
            partRear = np.roll(partRearTemp, 213)
            #pass
        if abc[0] == 'right':
            #pass
            partRightTemp[0:33] = abcc[0]
            partRightTemp[33:78] = abcc[1]
            partRightTemp[78:112] = abcc[2]
            partRight = np.roll(partRightTemp, 303)
            
            
         
        #print partFront.size
        #print partLeft.size
        #print partRight.size
        #print partRear.size
        
        
        
        cp1 = np.fmin(partFront,partLeft)
        cp2 = np.fmin(partRear,partRight)
       
        obstaclesAround = np.minimum(cp1,cp2)
        #print obstaclesAround
        #return
        #obstaclesAroundMM = np.roll(obstaclesAround, -22)
        #print partRight
        #print partFront
        #abc2 = abcc
        #print abc2
        #abc.reverse()
        
    
        #print(obstaclesAround[22:66])
        #print '\n'
        min8Array[0] = min(obstaclesAround[22:67])
        min8Array[7] = min(obstaclesAround[67:112])
        min8Array[6] = min(obstaclesAround[112:157])
        min8Array[5] = min(obstaclesAround[157:202])
        min8Array[4] = min(obstaclesAround[202:247])
        min8Array[3] = min(obstaclesAround[247:292])
        min8Array[2] = min(obstaclesAround[292:337])
        min8Array[1] = min(min(obstaclesAround[337:360]), min(obstaclesAround[0:22]))
        minAll = min(obstaclesAround[0:360])
        
        print min8Array[0], min8Array[1], min8Array[2], min8Array[3], min8Array[4], min8Array[5], min8Array[6], min8Array[7]
        #print minAll
        
        
        """
        obstaclesAroundPrime =  np.where(obstaclesAround==10000, 0, obstaclesAround) 
        print '1', obstaclesAroundPrime[0:59]
        print '2', obstaclesAroundPrime[60:119]
        print '3', obstaclesAroundPrime[120:179]
        print '4', obstaclesAroundPrime[180:239]
        print '5', obstaclesAroundPrime[240:299]
        print '6', obstaclesAroundPrime[300:359]
        return
        """
        
        
        
        
        
        if inLoiter: #and terrainHeight >= 120: used for ket hop voi radar dia hinh
           
            tempDist1 = distanceToStopinManual + float(distanceToWarn - distanceToStopinManual)/2
            tempDist2 = distanceToStopinManual + float(distanceToWarn - distanceToStopinManual)/6
            #print distanceToStopinManual,minAll,tempDist1
            if  (distanceToStopinManual<= minAll <= tempDist1 
                and beeperUsedFor[1]):
                beepDuty = 50
                beepPeriod = 0.5
               
                
            elif  (distanceToStopinManual<= minAll <= tempDist2
                and beeperUsedFor[1]):
                beepDuty = 50
                beepPeriod = 0.2
                
            elif (minAll < distanceToStopinManual
                and beeperUsedFor[1]):
                beepDuty = 100
                beepPeriod = 1
                
            elif beeperUsedFor[1] :
                beepDuty = 0
                beepPeriod = 1
            
            
            if rcAvoid > 1700:
                
                #time.sleep(0.005)
                
                rcRollNorm = round((float(rcRoll) - 1500) /500,2)
                rcPitchNorm = round((float(rcPitch) - 1500)/500,2)
                #print '----'
                print rcRoll, rcPitch
                rcHoriMag = math.sqrt(pow(rcRollNorm,2) + pow(rcPitchNorm,2))
                
                if rcRollNorm == 0.00 or rcRollNorm == -0.00:
                    rcHoriAngle = math.copysign(90.00, rcPitchNorm)
                else:
                    rcHoriAngle = (math.atan2(rcPitchNorm,rcRollNorm) * 180.00 /math.pi)
                if rcHoriAngle < 0:
                    rcHoriAngle = rcHoriAngle + 359
                rcHoriAngle = int(round(rcHoriAngle))
                
                
                
                #print rcHoriAngle
                #print rcHoriMag
                     
                #print secFromAngle
                #print 'dbh'
                if rcHoriMag >= 0.05:
                    
                    minSecNum = maxDist +1
                    fooIndex = -1
                    tempSecNum = 0
                    
                    # filter, lay xung quanh do + - 15 do
                    for i in range (-10,11):
                        
                        if rcHoriAngle + i < 0:
                            tempSecNum = rcHoriAngle + i + 360
                        elif rcHoriAngle + i > 359:
                            tempSecNum = rcHoriAngle + i - 360
                        else: 
                            tempSecNum = rcHoriAngle + i
                      
                        if (minSecNum > obstaclesAround[tempSecNum]):
                        
                            minSecNum = obstaclesAround[tempSecNum]
                            fooIndex = tempSecNum
                    #print fooIndex
                
                    if fooIndex!= -1:
                        if obstaclesAround[fooIndex] < distanceToWarn:
                            
                            
                
                            # this is used for dieu chinh RC Override, in progress
                            print fooIndex
                            ratioToRc = (obstaclesAround[fooIndex] - distanceToStopinManual) / (distanceToWarn - distanceToStopinManual)
                            print ratioToRc
                            if ratioToRc <= 0.00:
                                ratioToRc = 0
                            #agn = fooIndex*1.53 + 34
                            
                            ltSpeed =  (ratioToRc*LOIT_SPEED)
                            ltAcc =  (ratioToRc*LOIT_ACC_MAX)
                            
                            print ltAcc
                            print ltSpeed
                            
                            
                            print '----------------'
                          
                            #setLoiterSpeedAndGetACK(ltSpeed)
                            #setLoiterAccelerateAndGetACK(ltAcc)
                            
                            toAPLoitSpeed = ltSpeed
                            toAPAccMax = ltAcc
                        
                          
                    else:
                       #pass
                      
                        
                        #setLoiterSpeedAndGetACK(LOIT_SPEED)
                        #setLoiterAccelerateAndGetACK(LOIT_ACC_MAX)
                        toAPLoitSpeed = LOIT_SPEED
                        toAPAccMax = LOIT_ACC_MAX
                        
                
                else:
                    #pass  
               
                    #setLoiterSpeedAndGetACK(LOIT_SPEED)
                    #setLoiterAccelerateAndGetACK(LOIT_ACC_MAX)
                    toAPLoitSpeed = LOIT_SPEED
                    toAPAccMax = LOIT_ACC_MAX
                
            else:
             
                #setLoiterSpeedAndGetACK(LOIT_SPEED)
                #setLoiterAccelerateAndGetACK(LOIT_ACC_MAX)
                toAPLoitSpeed = LOIT_SPEED
                toAPAccMax = LOIT_ACC_MAX
            
            
            
            return 'DS'
            
        if inAuto or inRTL:
            if (min0 < distanceToStopinAuto or min1 < distanceToStopinAuto or min2 < distanceToStopinAuto
                or min3 < distanceToStopinAuto or min4 < distanceToStopinAuto or min5 < distanceToStopinAuto
                or min6 < distanceToStopinAuto or min7 <distanceToStopinAuto):
                    while True:
                    
                        master.mav.command_long_send(
                        master.target_system,
                        master.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                            0,
                            0, 0, 0, 0, currLat, currLon, currAlt + 2) # not continuos lift up because of staying in this state just 1 moment
                        time.sleep(0.01)
                        msg = master.recv_match()
                        if not msg:
                            continue
            
                        if msg.get_type() == 'COMMAND_ACK':
                            if msg.command == 17 and msg.result == 0:
                                break
                    print 'Obs in Auto, change to Loiter'
                        
            
                        
                   #    MAV_CMD_NAV_LOITER_UNLIM
                    
            
            
        #print len(abc)
        """
        msss = mavlink2.MAVLink_obstacle_distance_message(
                    0, #time
                    3, #sensor type
                    #abc[2:len(abc) - 2], #distance range in cm
                    abc,
                    1, #angular width
                    10, #min distance
                    maxDist, #max distance
                    1.53,
                    -55.08,
                    12
                    )
        time.sleep(0.01)
        master.mav.send(msss)
        """
       # print msss
    
        #return 'O'
           
    now = datetime.now()
    timeEnd = long(now.strftime("%H%M%S%f"))
    timeEnd = timeFrom00(timeEnd)
    
    
    #print timeEnd - timeStart
        
            
    

foo = []
for i in range(numberOfSecs):
    foo.append(0)

stpThr1 = False

def moveTOAngleSec(self):
    #moveRightUntil(0.1)
    #msg =  master.recv_match()
    #if  msg and  (msg.get_type() == 'LOCAL_POSITION_NED'):
    #    print msg.x, msg.y, msg.z
   
    maxFoo = 0
    currPos = -1
    
    parseSec(sectors)
    for i in range(numberOfSecs):
        foo[i] = 0
    for i in range(numberOfSecs):
        
        if sectors[i] >= 35: # check if there are some sector with distance >= 35m
            #foo.append(sec)
            foo[i] = foo[i-1] + 1
            if maxFoo <= foo[i]:
                maxFoo = foo[i]
                currPos = i
    print(foo)
    print(maxFoo)
    print(currPos)
    #foor = filter(lambda thr: thr>=38 , sectors)
    #print(sectors)
    
    #decision making
    pointSec = (2*currPos - maxFoo + 1)/2
    modulos = maxFoo % 2
    
    if currPos >=0:
        if modulos == 0:
            pointAngle =  (pointSec * degreeOfSec) - degreeOfSec/2
        else:
            pointAngle = (pointSec * degreeOfSec) + degreeOfSec/2
    else: 
        pointAngle = 0
        
    simpleVectorVelocity(math.cos(pointAngle/180 * math.pi),math.sin(pointAngle/180 * math.pi))
    
    print pointAngle
            

def setMaxDistanceManual():
    dstMan = 40
    """
    while True:
        master.mav.param_request_read_send(
            master.target_system, master.target_component,
            b'AVOID_MARGIN',
            -1)
                    
        msg = master.recv_match()
                    
        if not msg:
            continue
        if msg.get_type() == 'PARAM_VALUE' and msg.param_id == 'AVOID_MARGIN':
            dstMan = float(msg.param_value) * 100
    """
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b'AVOID_MARGIN',
        -1
        )  

    message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    return float(message['param_value']) * 100

distanceToStopinManual = setMaxDistanceManual()
#print distanceToStopinManual

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
        
LOIT_ACC_MAX = get_LOIT_ACC_MAX()
LOIT_SPEED = get_LOIT_SPEED()
toAPLoitSpeed = LOIT_SPEED
toAPAccMax = LOIT_ACC_MAX
print LOIT_ACC_MAX,LOIT_SPEED

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
            #time.sleep(0.001)
        #except Exception: 
        #    stpThr1 = True
        #    print ('aaa')
    def stop(self):
        self._stop_event.set()

def thingsDoneTer(self):
    #beeperDc = 0
    return 'Done'
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
                    print '.^^.'
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
                            print beeperUsedFor
                            continue
                        if beeperUsedFor[0]:
                            beepPeriod = float(aaa[1])
                            beepDuty = float(aaa[2])
                        
                    except:
                        print "Wrong beeper command"
                elif command == 't':
                    print 'Terminating process: ', os.getpid()
                    #thingsDoneTer()
                    os.kill(os.getpid(), signal.SIGKILL)
                   
                #time.sleep(0.5)
                
               
        except Exception: 
            print('Except from Input')
            #self.bucket.put(sys.exc_info())
            
GPIO.setwarnings(False)
GPIO.cleanup()
beeper = 7

GPIO.setmode(GPIO.BOARD)
GPIO.setup(beeper, GPIO.OUT)


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
                timeBoot = msg.time_boot_ms
                rcRoll = msg.chan1_raw
                rcPitch = msg.chan2_raw
                rcJoyLR = msg.chan13_raw
                rcJoyFB = msg.chan14_raw
                rcTerain = msg.chan8_raw
                rcAvoid = msg.chan9_raw
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
            #time.sleep(0.001)
indexMin = 0


class sendMin (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        
    def run(self):
        global indexMin
        while True:
            setLoiterSpeedAndGetACK(LOIT_SPEED)
            setLoiterAccelerateAndGetACK(LOIT_ACC_MAX)
            time.sleep(0.01)
            #aabb = []
            #for i in range(72):
            #    aabb.append(maxDist)
            #for i in range(8):
            #    aabb[i] = min8Array[i]
            #print "asd"
            """
            msss = mavlink2.MAVLink_obstacle_distance_message(
                    0, #time
                    3, #min distance
                    aabb, #max distance
                    1, # current
                    10, #type
                    maxDist,
                    45,
                    0,
                    12) #id
            time.sleep(0.001)
            master.mav.send(msss)
            """
            #singleThreadSendMin(indexMin)
            time.sleep(0.05)
            #print "done sent"
            indexMin = indexMin + 1
            if indexMin >= 8:
                indexMin = 0
            singleThreadSendMin(indexMin)
            #print indexMin
            #for i in range(8):
            #    time.sleep(0.01)
            #    singleThreadSendMin(i)
            #singleThreadSendMin(1,0.001)
            #singleThreadSendMin(2,0.001)
            #singleThreadSendMin(3,0.001)
            #singleThreadSendMin(4,0.001)
            #singleThreadSendMin(5,0.001)
            #singleThreadSendMin(6,0.001)
            #singleThreadSendMin(7,0.001)
            
            
            
          

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
    #time.sleep(delayF)
        
       
#entering the main program
beepUntil(0.7,20)
beepUntil(0.5,10)
beepUntil(0.5,10)
beepUntil(0.7,50)


thread1 = myThread(1, "Thread main")
thread2 = myThread2(2, "Receiving MSG")
thread3 = beeperThread(3, "Buzzer control")
thread4 = readInputThread(4, "Read input")
thread5 = sendMin(5, "Send min")
#time.sleep(1)

thread1.start()
#thread1.join()

thread2.start()
#thread2.join()

thread3.start()
#thread1.join()

thread4.start()
#thread1.join()
thread5.start()

#time.sleep(1)


thread1.join()
thread2.join()
#thread3.join()
#thread4.join()
#thread5.join()

#threading.Thread(target = singleThreadSendMin(0.1), daemon = True).start()
#thread2.start()

while True:
    #singleThreadSendMin()
    print "i amd fucked here"
### OA of AP
"""
while True:
    parseSec(sectors)
    
    #print sectors
    sectors = [i * 100 for i in sectors]
    
    msss = mavlink2.MAVLink_obstacle_distance_message(
    10, #time
    3, #sensor type
    sectors[startSec:stopSec], #distance range in cm
    1, #angular width
    30, #min distance
    10000, #max distance
    1.53,
    -50,
    1
    )
    master.mav.send(msss)
    

"""
"""
### My faithful Alg
while True:
    
    #moveRightUntil(0.1)
    #msg =  master.recv_match()
    #if  msg and  (msg.get_type() == 'LOCAL_POSITION_NED'):
    #    print msg.x, msg.y, msg.z
   
    maxFoo = 0
    currPos = -1
    
    parseSec(sectors)
    for i in range(numberOfSecs):
        foo[i] = 0
    for i in range(numberOfSecs):
        
        if sectors[i] >= 35:
            #foo.append(sec)
            foo[i] = foo[i-1] + 1
            if maxFoo <= foo[i]:
                maxFoo = foo[i]
                currPos = i
    print(foo)
    print(maxFoo)
    print(currPos)
    #foor = filter(lambda thr: thr>=38 , sectors)
    #print(sectors)
    
    #decision making
    pointSec = (2*currPos - maxFoo + 1)/2
    modulos = maxFoo % 2
    
    if currPos >=0:
        if modulos == 0:
            pointAngle =  (pointSec * degreeOfSec) - degreeOfSec/2
        else:
            pointAngle = (pointSec * degreeOfSec) + degreeOfSec/2
    else: 
        pointAngle = 0
        
    simpleVectorVelocity(math.cos(pointAngle/180 * math.pi),math.sin(pointAngle/180 * math.pi))
    print pointAngle
            
    
    #time.sleep(0.1)
#if simpleMoveRight(10) == 'Done moving right':
#    print('Okay be de')
"""
