
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

master = mavutil.mavlink_connection(UDP_IP_650_TCP, dialect = "ardupilotmega")
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
connectTo650(master)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)





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
rcThrottle = 1500
rcYaw = 1500
rcJoyLR = 0
rcjoyFB = 0
rcAvoid = 1000
rcTerain = 1000
timeBoot = 0

jtX = 0
jtY = 0
jtZ = 0
jtR = 0


class getMessages(threading.Thread):
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
		global rcThrottle
		global rcYaw
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
				rcThrottle = msg.chan3_raw
				rcYaw = msg.chan4_raw
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
	

class readInputThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        
        #self.counter = counter
    def run(self):
       
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
                  #          beeperUsedFor[0] = 1
                  #          beeperUsedFor[1] = 0
                  #          beeperUsedFor[2] = 0
                            continue
                        if aaa[1] == 'b':
                  #          beeperUsedFor[0] = 0
                  #          beeperUsedFor[1] = 1
                  #          beeperUsedFor[2] = 0
                            continue
                        if aaa[1] == 'c':
                  #          beeperUsedFor[0] = 0
                  #          beeperUsedFor[1] = 0
                  #          beeperUsedFor[2] = 1
                            continue
                        if aaa[1] == 'check':
                  #          print beeperUsedFor
                            continue
                        if beeperUsedFor[0]:
							pass
                  #          beepPeriod = float(aaa[1])
                  #          beepDuty = float(aaa[2])
                        
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

class inTheGuided(threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name

	def run(self):
		while True:
			if inGuided:
			
				jtX = translate(rcPitch,1000,2000,-1000,1000)
				jtY = translate(rcRoll,1000,2000,-1000,1000)
				jtZ = translate(rcThrottle,1000,2000,-1000,1000)
				jtR = translate(rcYaw,1000,2000,-1000,1000)
				print jtX, jtY, jtZ, jtR
				master.mav.manual_control_send(
					master.target_system,
					jtX,
					jtY,
					jtZ,
					jtR,
					1)
				time.sleep(0.01)
	




thread1 = inTheGuided(1, "Thread main")
thread2 = getMessages(2, "Receiving MSG")
thread3 = readInputThread(3, "Read input")

thread1.start()
thread2.start()
thread3.start()

thread1.join()
thread2.join()

