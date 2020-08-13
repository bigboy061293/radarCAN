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
SIMULATOR_UDP = 'udp:127.0.0.1:14550'
UDP_IP_LOCAL = "127.0.0.1"
UDP_PORT = 2992
UDP_IP_650 = 'tcp:192.168.0.210:20002'

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP_LOCAL, UDP_PORT))

master = mavutil.mavlink_connection(UDP_IP_650, dialect = "ardupilotmega")
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
        time.sleep(0.5)




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

def sendDistanceObstacleSensor(self, stateToSend, arrayToSend):
    

def parseSec(setors):
    global sectors
    global msss
    global mss
    while True:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        if data != None:
            break
    #print data
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
        print(terrainHeight)
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
        abc.reverse()
        
        if inLoiter:
            min1 = min(abc[0:20])
            min2 = min(abc[21:50])
            min3 = min(abc[51:71])
            #print min1, min2, min3
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
            time.sleep(0.01)
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
            return 'DS'
            
            
        #print len(abc)
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
            
inLoiter = False
inAlthold = False            
          
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
        while True:
        
            msg = master.recv_match()
            if not msg:
                continue
            
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
                    
                    
           
            
            #print inLoiter
           
           # print 'aaaaaaa'

#os.system("python radarReadAndSendUDPNew.py &")




time.sleep(1)

thread1 = myThread(1, "Thread main")
thread2 = myThread2(2, "Receiving MSG")

#time.sleep(1)

thread1.start()
thread2.start()

#time.sleep(1)


thread1.join()
thread2.join()
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
