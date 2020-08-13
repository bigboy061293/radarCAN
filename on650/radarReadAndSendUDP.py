"""

define stuff

"""

thisDroneDatalinkAddress = 'tcp:192.168.0.210:20002'

import can
import time
import turtle
import curses
import socket
import math
from datetime import datetime

#import panda as pd


from pymavlink import mavutil

UDP_IP = "192.168.0.111"
UDP_IP_LOCAL = "127.0.0.1"
UDP_PORT = 2992
beginToProcessRadar = False

arrayMR72Count = 0
screenW = 1000
screenH = 600
radarShownScale = 20
radarHeightLimit = 2

""" 

init sectors

"""

radarMaxRange = 40
numberOfSecs = 20
degree = 180
degreeOfSec = 180/numberOfSecs
radarBeam = 110
startBeam = 35
stopBeam = 145
startSec = 0
stopSec = 0
sectors = []
"""
sectors = [0, #sec 1
			0, #sec 2
			radarMaxRange, #sec 3
			radarMaxRange, #sec 4
			radarMaxRange, #sec 5
			radarMaxRange, #sec 6
			radarMaxRange, #sec 7
			radarMaxRange, #sec 8
			0, #sec 9
			0 #sec 10
			]
			"""
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

initSecs(numberOfSecs)
print stopSec, startSec
print sectors

"""
for i in range(numberOfSecs):
	abc = (i) * degreeOfSec
	print abc
	print (degree - radarBeam) / 2
	if ((abc >= 0) and (abc <  (degree - radarBeam) / 2 )) or ((abc <= degree) and (abc >  (radarBeam + (degree - radarBeam) / 2 ))):
		sectors.append(radarMaxRange)
	else:
		sectors.append(0)
"""
def xyReturnSec(x,y, degreeDec):
		abc = math.atan2(y,x) * 180 / math.pi
		#print abc
		while not (round(abc) % degreeOfSec == 0):
			abc = abc + 1;
		seccc = round(abc / degreeOfSec)
		mag = math.sqrt(pow(x,2) + pow(y,2))
		return seccc - 1, mag
#print(sectors)
#print (xyReturnSec(-10,0.111,18))


#while True:
#pass

""" 

initialize stream

"""


now = datetime.now()
currenTimeCreateFile = now.strftime("%H%M%S%f%d")
currenTimeCreateFile = 'log/' + currenTimeCreateFile
fileNow = open(currenTimeCreateFile,'w+')
fileNow.write('{ID} {LATx} {LONGY}')
fileNow.write('\r\n')
fileNow.close()

"""

initialize can

"""

print('----- Starting CAN -----')
can_interface ='can0'
bus = can.interface.Bus(can_interface, bustype = 'socketcan_native')


time.sleep(0.1)



"""

mavlink and pymavlink

"""
#print('----- Starting MAVLINK -----')
#master = mavutil.mavlink_connection(thisDroneDatalinkAddress)
#master.wait_heartbeat()
print('----- Heartbeat receive -----')



time.sleep(0.1)


class radarObject:
	
	ID = 0
	IDx = 0
	latX = 0.0
	longY = 0.0
	shownOff = 0
	"""
	global ID
	global latX
	global longY
	global shownOff
	"""
	def __init__ (self,ID,latX,longY,shownOff):
		self.ID = ID
		
		self.latX = latX
		self.longY = longY
		self.shownOff = shownOff
		
		
	def parseMessgageToArrayMR72 (self,message):
		
		self.ID = message.data[0]
		self.longY = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
		self.latX = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
		self.shownOff = 0
	
		

arrayMR72 = []
arrayMR72marker = []
arrayMR72Turtle = []
for i in range(256):
	arrayMR72.append(radarObject(0,0,0,0))



		
		



def radarType(message):
	if ((((message.arbitration_id >> 8) & 0X0F) == 0X07) and ((message.arbitration_id  & 0X0F) == 0X0C)):
		return 'NR24'
	elif ((((message.arbitration_id >> 8) & 0X0F) == 0X06) and ((message.arbitration_id  & 0X0F) == 0X0B)):
		return 'MR72'
	else:
		return 'NA'
	
def terrainHeight(message):
	return (message.data[2]*256 + message.data[3])

timeNow = 0
timeBefore = 0

def sendToUDP(arraryMR72):
	global sectors
	global arrayMR72
	global timeNow
	global timeBefore
	now = datetime.now()
	timeNow = int(now.strftime("%H%M%S%f"))
	for i in range(256):
		if arrayMR72[i].shownOff == 1:
			
		
			#strID = str(arrayMR72[i].ID)
			#strLatX = str(arrayMR72[i].latX)
			#strLongY = str(arrayMR72[i].longY)
			
			sec, mag = xyReturnSec(arrayMR72[i].latX,arrayMR72[i].longY,18)
			#print startSec, stopSec
			if sec in range (startSec,stopSec+1):
				if sectors[int(sec)] >= mag:
					sectors[int(sec)] = mag
			#	print mag
			#strAll = strID + ',' + strLatX + ',' +  strLongY
			
			
			#if (timeNow - timeBefore) > 30000:
			#	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			#	sock.sendto(strAll, (UDP_IP, UDP_PORT))

		
			arrayMR72[i].shownOff = 0
	#print timeNow
	if (timeNow - timeBefore) > 30:
				
				strAll = ''
				
				for i in range (startSec,stopSec+1):
					#print(i)
					if i == stopSec:
						strAll = strAll + str(sectors[i])
					else:
						strAll = strAll + str(sectors[i]) + ','
					
				print strAll
				sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
				sock.sendto(strAll, (UDP_IP_LOCAL, UDP_PORT))
				
				initSecs(numberOfSecs)
				
	
	timeBefore = timeNow
	#wn.update()

def processRadar(message):
	global arrayMR72

	if message.arbitration_id == 0X60A:
		beginToProcessRadar = True
		
		# beginToProcessRadar will be set to False after calculating the vector and visualize
		if (message.data[0] == 0X01) and (message.data[2] == 0x00) and(message.data[3] == 0x00) and (message.data[4] == 0x00) and (message.data[5] == 0x00) and	(message.data[6] == 0x00) and (message.data[7] == 0x00):
			#print('This is NRA24')
			pass
		else:
			sendToUDP(arrayMR72)

		
	elif message.arbitration_id == 0X60B:
		
		beginToProcessRadar = False
		
		abc = radarObject(0,0,0,0)
		abc.parseMessgageToArrayMR72(message)
		
		arrayMR72[abc.ID].ID = abc.ID
		
		
		arrayMR72[abc.ID].latX = abc.latX
		arrayMR72[abc.ID].longY = abc.longY
		arrayMR72[abc.ID].shownOff = 1
	elif message.arbitration_id == 0X70C:
		terrain0 = message.data[2]*256 + message.data[3]
		strAll = 'H,' + str(terrain0)
		
		now = datetime.now()
		newLine = now.strftime("%H#%M#%S#%f")
			
		
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		sock.sendto(strAll, (UDP_IP_LOCAL, UDP_PORT))
		



def thread_processRadar(message):
	while True:
		message = bus.recv()
		processRadar(message)


try:
	while True:
		
		message = bus.recv()
		processRadar(message)
except KeyboardInterrupt:
	try:
		abc = open(currenTimeCreateFile, "r") # or "a+", whatever you need
	except IOError:
		print "Could not open file! Please close Excel!"
	fileNow.close()
	print('Done.')
