"""

enum class Number {
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    DRIFT =        11,  // semi-automous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
    THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
    SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
    FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
    ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    NEW_MODE =     25,  // your new flight mode
};

"""

from pymavlink import mavutil
import time
import math
SIMULATOR_UDP = 'udp:127.0.0.1:14550'
TCP_650 = 'tcp:192.168.0.210:20002'



master = mavutil.mavlink_connection(TCP_650)
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
    
connectToSim(master)
#aa = simpleVectorVelocity(10,10)
#moveFollowVector(aa,0.5)
#time.sleep(10)
#bb = simpleVectorVelocity(30,60)
#moveFollowVector(bb,30)
#time.sleep(4)

master.mav.request_data_stream_send(master.target_system, master.target_component, 
		mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
		
while True:
	
	msg = master.recv_match()
	if not msg:
		continue
	print msg
	#if msg.get_type() == 'LOCAL_POSITION_NED':
	#	print msg.x, msg.y, msg.z
  
#while True:
    #print master.mode_mapping()
#moveRightUntil(0.1)

	

