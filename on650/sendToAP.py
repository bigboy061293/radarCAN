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
SIMULATOR_UDP = 'udp:127.0.0.1:14550'
UDP_IP_LOCAL = "127.0.0.1"
UDP_PORT = 2992
UDP_IP_650_TCP = 'tcp:192.168.0.210:20002'
RPI_SERIAL = '/dev/ttyS0'

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP_LOCAL, UDP_PORT))

master = mavutil.mavlink_connection(UDP_IP_650_TCP, dialect = "ardupilotmega")
#master = mavutil.mavlink_connection(RPI_SERIAL, baud = 115200)
mavutil.set_dialect("ardupilotmega")

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




def connectToSim(master):
    msg = None
    while not msg:
        master.mav.ping_send(
            time.time(), # Unix time
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)

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


def setDesiredMessages(master):
    
    master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL , 0,
    0, 2, 0, 0, 0, 0, 0) 
    
    #master.mav.request_data_stream_send(
    #    master.target_system, #
    #    master.target_component, #
    #   mavutil.mavlink.MAV_DATA_STREAM_ALL, #max distance
    #   10, 
    #1 )
     
    #ms = mavlink2.MAVLink_request_data_stream_message(
    #    master.target_system, #time
    #    master.target_component, #min distance
    #    1, #max distance
    #    1, # current
    #    1)
    #master.mav.send(ms)
 

distance = []
dumpy =[0,0,0,0]
for i in range(72):
    distance.append(1000)
mss = mavlink2.MAVLink_distance_sensor_message(
    0, #time
    10, #min distance
    10000, #max distance
    200, # current
    3, #type
    1, #id
    0, #ori
    0, #cova
    0, #fov
    0, #fov
    dumpy
    )
connectTo650(master)
time.sleep(0.5)
#print master.target_system
#print master.target_component
#setDesiredMessages(master)

time.sleep(0.5)

#check return messages
#while True:
#    msg = master.recv_match()
#    while not msg:
#        continue
#    print msg
    

"""
master.mav.param_set_send(
    master.target_system, master.target_component,
    b'OA_DB_SIZE',
    -12,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)
"""
"""
while True:
    master.mav.send(ms)
    #master.mav.send(mss)
    time.sleep(0.05)
"""

"""
mavlink2.Mavlink.obstacle_distance_encode( 0, #time
    3, #sensor type
    distance, #distance range in cm
    10, #angular width
    10, #min distance
    10000 #max distance
    )
"""
"""
while True:
    pass
 
    mavlink2.obstacle_distance_send(
    0, #time
    3, #sensor type
    distance, #distance range in cm
    10, #angular width
    10, #min distance
    10000 #max distance
    )
    
    time.sleep(0.1)
"""


def simpleMoveRight(metterRight):
    distanceFromDest = 10000000
    # reserve the previous mode
    msg = master.recv_match()
    while not msg or not (msg.get_type() == 'HEARTBEAT'):
        msg = master.recv_match()
        pass

    modeBefore = msg.custom_mode
    print(modeBefore)
   
    while not msg or not (msg.get_type() == 'LOCAL_POSITION_NED'):
        msg = master.recv_match()
        pass
    xBefore = msg.x
    yBefore = msg.y
    zBefore = msg.z
    
    
    print xBefore,yBefore,zBefore
    
    destInY = yBefore +  metterRight

     # change mode to Guided
    master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4)

    #SET_POSITION_TARGET_LOCAL_NED
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        1, # frame coor MAV_FRAME_GLOBAL_INT
        0b110111111000, # control pos
        xBefore,
        destInY,
        zBefore,
        1, #vx
        1, #vy
        1, #vz
        0, #ax
        0, #ay
        0, #az
        0, #yaw angle
        0 #yaw rate
        )
    # move right
    while distanceFromDest > 0.5:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'HEARTBEAT':
           if msg.custom_mode != 4:
               return ('Mode change by user, not completed!')
        if msg.get_type() == 'LOCAL_POSITION_NED':
            distanceFromDest = abs(msg.y - destInY)
           
          
            print distanceFromDest
            print '----'
     # change mode to back to previous mode
    master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    modeBefore)
    #print ("Done moving right!")
    return ('Done moving right')


def moveRightUntil(velocity):
     # reserve the previous mode
    #msg = master.recv_match()
    # change mode to Guided
    master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4)
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        1, # frame coor MAV_FRAME_GLOBAL_INT
        0b110111000111, # control pos
        0,
        0,
        0,
        0, #vx
        velocity, #vy
        0, #vz
        0, #ax
        0, #ay
        0, #az
        0, #yaw angle
        0 #yaw rate
        )

def moveFollowVector(vectorMove, movCoef):
     # reserve the previous mode
    #msg = master.recv_match()
    # change mode to Guided
    print(vectorMove.vxN)
    master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4)
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        1, # frame coor MAV_FRAME_GLOBAL_INT
        0b110111000111, # control pos
        0,
        0,
        0,
        vectorMove.vyNorm() * movCoef  , #vx
        vectorMove.vxNorm() * movCoef, #vy
        0, #vz
        0, #ax
        0, #ay
        0, #az
        0, #yaw angle
        0 #yaw rate
        )
#def moveVecto
    

"""
while True:
    
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'LOCAL_POSITION_NED':
        print msg.x, msg.y, msg.z
"""        
#while True:
    #print master.mode_mapping()
#moveRightUntil(0.1)


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
            


def parseSec(setors):
    global sectors
    global msss
    global mss
    global beepPeriod
    global beepDuty
    global beeperUsedFor
    global terrainHeight
    while True:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        if data != None:
            break
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
     
        
        aa = []
        
        
        
        
        for i in range (len(abc)):
            abc[i] = int(float(abc[i]) * 100)
            aa.append(i*100)
            if abc[i] >= 9999:
                abc[i] = maxDist + 1
        
        for i in range(360):
            if i < 34 or i >= 144:
                continue
            obstaclesAround[i] = abc[int(round((i - 34) / 1.53))]
            
        abc2 = abc
        abc.reverse()
        
    
        
        min1 = min(abc[0:20])
        min2 = min(abc[21:50])
        min3 = min(abc[51:71])
        minAll = min(abc[0:71])
        
        
        
    
        
             
        
        
        
        
        
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
            
            
            
            """
            rcJoyLRNew = -1000 + 2000 * ((float(rcJoyLR) - 1000) / 1000)
            
            rcJoyFBNew = -1000 + 2000 * ((float(rcJoyFB) - 1000) / 1000)
            print rcJoyLRNew, rcJoyFBNew
            
            msss = mavlink1.MAVLink_manual_control_message(
                            master.target_component,
                            rcJoyLRNew,
                            rcJoyFBNew, 
                            0, 
                            0,
                            0)
            master.mav.send(msss)
            
            """
            if rcAvoid > 1700:
            
                rcRollNorm = round((float(rcRoll) - 1500) /500,2)
                rcPitchNorm = round((float(rcPitch) - 1500)/500,2)
    
                #print rcRollNorm, rcPitchNorm
                if rcRollNorm == 0.00 or rcRollNorm == -0.00:
                    rcHoriAngle = math.copysign(90.00, rcPitchNorm)
                else:
                    rcHoriAngle = (math.atan2(rcPitchNorm,rcRollNorm) * 180.00 /math.pi)
                    
                #print rcHoriAngle
                if rcHoriAngle > 144 or rcHoriAngle < 34:
                    secFromAngle = -1
                else:
                    secFromAngle = int((rcHoriAngle-34)/1.53)
                    if secFromAngle < 0:
                        secFromAngle = 0
                    if secFromAngle > 71:
                        secFromAngle = 71
                #print secFromAngle
                
                if secFromAngle != -1:
                    minSecNum = maxDist +1
                    fooIndex = -1
                    tempSecNum = 0
                    
                    for i in range (-4,5):
                        
                        if secFromAngle + i < 0:
                            tempSecNum = 0
                        elif secFromAngle + i > 71:
                            tempSecNum = 71
                        else: 
                            tempSecNum = secFromAngle + i
                            
    
                        if (minSecNum > abc2[tempSecNum]):
                        
                            minSecNum = abc2[tempSecNum]
                            fooIndex = tempSecNum
                    #print fooIndex
                
                    if fooIndex!= -1:
                        if abc2[fooIndex] < distanceToWarn:
                            
                            
                            
                            
                            # this is used for dieu chinh RC Override, in progress
                            print fooIndex
                            ratioToRc = (abc2[fooIndex] - distanceToStopinManual) / (distanceToWarn - distanceToStopinManual)
                            print ratioToRc
                            if ratioToRc <= 0.00:
                                ratioToRc = 0
                            #agn = fooIndex*1.53 + 34
                            
                            ltSpeed =  (ratioToRc*LOIT_SPEED)
                            ltAcc =  (ratioToRc*LOIT_ACC_MAX)
                            
                            print ltAcc
                            print ltSpeed
                            
                            
                            print '----------------'
                            """
                            master.mav.param_set_send(
                                master.target_system, master.target_component,
                                b'LOIT_SPEED',
                                ltSpeed,
                                mavutil.mavlink.MAV_PARAM_TYPE_UINT16
                                )
                            """
                            setLoiterSpeedAndGetACK(ltSpeed)
                            setLoiterAccelerateAndGetACK(ltAcc)
                            #time.sleep(0.01)
                            """
                            master.mav.param_set_send(
                                master.target_system, master.target_component,
                                b'LOIT_ACC_MAX',
                                ltAcc,
                                mavutil.mavlink.MAV_PARAM_TYPE_UINT16
                                )
                            """
                            
                            """
                            rcRollNew =int( 1500 + (rcRoll - 1500) * ratioToRc * math.cos(agn*math.pi/180.00) )
                            rcPitchNew = int( 1500 + (rcPitch -1500) * ratioToRc * math.sin(agn*math.pi/180.00))
                            print abc2[fooIndex]
                            print rcRoll, rcPitch
                            print rcRollNew, rcPitchNew
                            print '---------------------------------'
                            """
                            """
                            msss = mavlink1.MAVLink_rc_channels_override_message(
                                master.target_system,
                                master.target_component,
                                rcRollNew, #time
                                rcPitchNew, #min distance
                                0, #3
                                0, #4
                                0, #5
                                0, #6
                                0, #7
                                0)
                            master.mav.send(msss)
                            """
                            #time.sleep(0.01)
                    else:
                       #pass
                      
                        """master.mav.param_set_send(
                                master.target_system, master.target_component,
                                b'LOIT_SPEED',
                                LOIT_SPEED,
                                mavutil.mavlink.MAV_PARAM_TYPE_UINT16
                                )     
                        """
                        setLoiterSpeedAndGetACK(LOIT_SPEED)
                        setLoiterAccelerateAndGetACK(LOIT_ACC_MAX)
                        """
                        #time.sleep(0.01)
                        master.mav.param_set_send(
                                master.target_system, master.target_component,
                                b'LOIT_ACC_MAX',
                                LOIT_ACC_MAX,
                                mavutil.mavlink.MAV_PARAM_TYPE_UINT16
                                )  
                        """
                        
                
                else:
                    #pass  
                    """
                    master.mav.param_set_send(
                                master.target_system, master.target_component,
                                b'LOIT_SPEED',
                                LOIT_SPEED,
                                mavutil.mavlink.MAV_PARAM_TYPE_UINT16
                                )   
                    """
                    setLoiterSpeedAndGetACK(LOIT_SPEED)
                    setLoiterAccelerateAndGetACK(LOIT_ACC_MAX)
                    """
                    #time.sleep(0.01)
                    master.mav.param_set_send(
                                master.target_system, master.target_component,
                                b'LOIT_ACC_MAX',
                                LOIT_ACC_MAX,
                                mavutil.mavlink.MAV_PARAM_TYPE_UINT16
                                )  
                    """
                
            else:
                """
                master.mav.param_set_send(
                                master.target_system, master.target_component,
                                b'LOIT_SPEED',
                                LOIT_SPEED,
                                mavutil.mavlink.MAV_PARAM_TYPE_UINT16
                                )     
                        
                        #time.sleep(0.01)
                """
                setLoiterSpeedAndGetACK(LOIT_SPEED)
                setLoiterAccelerateAndGetACK(LOIT_ACC_MAX)
                """
                master.mav.param_set_send(
                                master.target_system, master.target_component,
                                b'LOIT_ACC_MAX',
                                LOIT_ACC_MAX,
                                mavutil.mavlink.MAV_PARAM_TYPE_UINT16
                                )  
                """
                        #print abc2[fooIndex], distanceToStopinManual, distanceToWarn
            #print min1, min2, min3
            """
            msss = mavlink1.MAVLink_distance_sensor_message(
                        0, #time
                        10, #min distance
                        maxDist, #max distance
                        min1, # current
                        3, #type
                        1, #id
                        7, #ori
                        255)
            master.mav.send(msss)
            time.sleep(0.01)
            """
            msss = mavlink1.MAVLink_distance_sensor_message(
                        0, #time
                        10, #min distance
                        maxDist, #max distance
                        min2, # current
                        3, #type
                        1, #id
                        0, #ori
                        255)
            
            master.mav.send(msss)
            #time.sleep(0.01)
            """
            msss = mavlink2.MAVLink_distance_sensor_message(
                        0, #time
                        10, #min distance
                        maxDist, #max distance
                        min3, # current
                        3, #type
                        1, #id
                        1, #ori
                        255)
            master.mav.send(msss)
            #time.sleep(0.01)
            """
            return 'DS'
            
        if inAuto or inRTL:
            if min1 < distanceToStopinAuto or min2 < distanceToStopinAuto or min3 < distanceToStopinAuto:
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
        return 'O'
        
            
    

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
print LOIT_ACC_MAX,LOIT_SPEED

class myThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        #self.counter = counter
    def run(self):
        try:
            while True:
                global stpThr1
                global sectors
                stat = parseSec(sectors)

        except Exception: 
            stpThr1 = True
            print ('aaa')
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
                    

#entering the main program
beepUntil(0.7,20)
beepUntil(0.5,10)
beepUntil(0.5,10)
beepUntil(0.7,50)


thread1 = myThread(1, "Thread main")
thread2 = myThread2(2, "Receiving MSG")
thread3 = beeperThread(3, "Buzzer control")
thread4 = readInputThread(4, "Read input")
#time.sleep(1)

thread1.start()
#thread1.join()

thread2.start()
#thread2.join()

thread3.start()
#thread1.join()

thread4.start()
#thread1.join()

#time.sleep(1)


thread1.join()
thread2.join()
thread3.join()
thread4.join()
#thread2.start()

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
