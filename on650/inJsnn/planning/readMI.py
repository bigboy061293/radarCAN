from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import time
import math
import inspect
import threading
import os
import ctypes
import subprocess, signal
import numpy as np

UDP_IP_650_TCP = 'tcp:192.168.0.210:20002'
master = mavutil.mavlink_connection(UDP_IP_650_TCP, dialect = "ardupilotmega")
missionList = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0]])
currentMission = 1 #set cai nay la current mission trong MISSION_CURRENT #42
print missionList
def latLongReturnAngle(latA, longA, latB, longB): #11.2131313
    temp = math.atan2(longB - longA, latB - latA) * (180.00 / math.pi)
    if temp <=0:
        temp = 360 + temp
    return temp
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
        time.sleep(0.5)

connectTo650(master)

master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b'AVOID_MARGIN',
        -1
        )  

message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()

def downloadAndPrint():
		global missionList
		mss = mavlink2.MAVLink_mission_request_list_message(
										master.target_system,
										master.target_component,
										0
                    )
		master.mav.send(mss)
		itera = 0
		print '----------------------------------------------'
		while True:
				msg = master.recv_match()
				if not msg:
					continue
				if msg.get_type() == 'MISSION_COUNT':
					print msg
					itera = int(msg.count)
					if itera == 0:
							return 'No missions'
					else:
							for i in range(itera):
									mss = mavlink2.MAVLink_mission_request_int_message(
										master.target_system,
										master.target_component,
										i,
										0)
									master.mav.send(mss)
									while True:
											msg = master.recv_match()
											if not msg:
												continue
											if msg.get_type() == 'MISSION_ITEM_INT':
												print msg
												print np.size(missionList)
												if i != 0:
													missionList = np.append(missionList, 
																	[[0,0,0,0,0,0,0,0,0,0,0,0,0,0]], axis = 0)
												
												missionList[i,0] = int(msg.target_system)
												missionList[i,1] = int(msg.target_component)
												missionList[i,2] = int(msg.seq)
												missionList[i,3] = int(msg.frame)
												missionList[i,4] = int(msg.command)
												missionList[i,5] = int(msg.current)
												missionList[i,6] = int(msg.autocontinue)
												missionList[i,7] = float(msg.param1)
												missionList[i,8] = float(msg.param2)
												missionList[i,9] = float(msg.param3)
												missionList[i,10] = float(msg.param4)
												missionList[i,11] = int(msg.x)
												missionList[i,12] = int(msg.y)
												missionList[i,13] = float(msg.z)												
												break
									if i == itera -1:
										return 'Done'
					
					
				 
print downloadAndPrint()
for i in range(np.size(missionList,0)):
		if missionList[i,4] == 16:
			print missionList[i]
print latLongReturnAngle(10, 19, float(missionList[currentMission,11]) / 7 , float(missionList[currentMission,12]) / 7)
