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
import threading

exitFlag = 0
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

radarMaxRange = 99.99
radarMinRange = 0.5
numberOfSecs = 112
degree = 180
degreeOfSec = 1.53
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
	global sectors
	for i in range(numberOfSecs):
		sectors[i] = radarMaxRange
	

initSecs(numberOfSecs)
#print stopSec, startSec
#print sectors

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
	
		abc = math.atan2(y,x) * (180.00 / math.pi)
		abc = int(abc * 100)
		while not abc % (degreeOfSec*100) == 0:
			
			#print (abc * 100) % degreeOfSec
			abc = abc + 1
			#print abc
		seccc = round(abc / (degreeOfSec*100))
		mag = math.sqrt(pow(x,2) + pow(y,2))
		return seccc, mag
#print(xyReturnSec(-1,0,1.53))
#while True:
#	pass
#print(sectors)
#print (xyReturnSec(-10,0.111,18))


#while True:
#pass

""" 

initialize stream

"""

"""
now = datetime.now()
currenTimeCreateFile = now.strftime("%H%M%S%f%d")
currenTimeCreateFile = 'log/' + currenTimeCreateFile
fileNow = open(currenTimeCreateFile,'w+')
fileNow.write('{ID} {LATx} {LONGY}')
fileNow.write('\r\n')
fileNow.close()
"""
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
arrayMR72_1 = []
arrayMR72_2 = []
arrayMR72_3 = []

for i in range(256):
	arrayMR72.append(radarObject(0,0,0,0))
	arrayMR72_1.append(radarObject(0,0,0,0))
	arrayMR72_2.append(radarObject(0,0,0,0))
	arrayMR72_3.append(radarObject(0,0,0,0))

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
def timeFrom00(timeInInt):
	miSec = timeInInt % 1000000
	timeInInt = timeInInt / 1000000
	sec = timeInInt % 100
	timeInInt = timeInInt / 100
	min = timeInInt % 100
	hour = timeInInt / 100
	timeReal = long((hour * 3600 + min * 60 + sec) * 1000000 + miSec)
	#print (hour)
	#print (min)
	#print (sec)
	#print (miSec)
	
	return(timeReal)
def sendToUDP(arraryMR72, sendWhatMR72):
	global sectors
	global arrayMR72
	global timeNow
	global timeBefore
	now = datetime.now()
	timeNow = long(now.strftime("%H%M%S%f"))
	timeNow = timeFrom00(timeNow)
	#print timeNow
	#print ' ---- ', timeNow - timeBefore
	#30000
	#print timeNow - timeBefore
	if abs(timeNow - timeBefore) > 500:
		#print 'here in'
		"""
		for i in range(256):
			if arrayMR72[i].shownOff == 1:
				sec, mag = xyReturnSec(arrayMR72[i].latX,arrayMR72[i].longY,1)
					if sectors[int(round(sec))] >= mag:
						sectors[int(round(sec))] = mag
				arrayMR72[i].shownOff = 0
		"""
		#strAll = ''
		if sendWhatMR72 == 0:
			strAll = 'front,'
			for i in range(256):
				if arrayMR72[i].shownOff == 1:
					sec, mag = xyReturnSec(arrayMR72[i].latX,arrayMR72[i].longY,1)
					#print int(round(sec))
					id = int(round(sec))
					if id >= numberOfSecs:
						id = numberOfSecs -1
					if sectors[id] >= mag:
						sectors[id] = mag
					arrayMR72[i].shownOff = 0
				
				
		elif sendWhatMR72 == 1:
			strAll = 'right,'
			for i in range(256):
				if arrayMR72_1[i].shownOff == 1:
					sec, mag = xyReturnSec(arrayMR72_1[i].latX,arrayMR72_1[i].longY,1)
					id = int(round(sec))
					if id >= numberOfSecs:
						id = numberOfSecs -1
					if sectors[id] >= mag:
						sectors[id] = mag
					arrayMR72_1[i].shownOff = 0
				
		elif sendWhatMR72 == 2:
			strAll = 'rear,'
			for i in range(256):
				if arrayMR72_2[i].shownOff == 1:
					sec, mag = xyReturnSec(arrayMR72_2[i].latX,arrayMR72_2[i].longY,1)
					id = int(round(sec))
					if id >= numberOfSecs:
						id = numberOfSecs -1
					if sectors[id] >= mag:
						sectors[id] = mag
					arrayMR72_2[i].shownOff = 0
		elif sendWhatMR72 == 3:
			strAll = 'left,'
			for i in range(256):
				if arrayMR72_3[i].shownOff == 1:
					sec, mag = xyReturnSec(arrayMR72_3[i].latX,arrayMR72_3[i].longY,1)
					id = int(round(sec))
					if id >= numberOfSecs:
						id = numberOfSecs -1
					if sectors[id] >= mag:
						sectors[id] = mag
					arrayMR72_3[i].shownOff = 0
		else: 
			return 'wrong, nothing to sent'
				
		for i in range (numberOfSecs):
			if i == stopSec-1:
				strAll = strAll + str(sectors[i])
			else:
				strAll = strAll + str(sectors[i]) + ','
		
		initSecs(numberOfSecs)			
		#print strAll
		#print '\n'
		
		#abc = sectors[startSec:stopSec]
		#abc.reverse()
		
		#print 'Sec 7', abc[0:20]
		#print 'Sec 0', abc[21:50]
		#print 'Sec 1', abc[51:71]
		
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		sock.sendto(strAll, (UDP_IP_LOCAL, UDP_PORT))		
		
		timeBefore = timeNow
				
	
	

timeOutMR72 = 1000000
timeOutNRA24 = 1000
cntTimeOutMR72 = 0
cntTimeOutMR72_1 = 0
cntTimeOutMR72_2 = 0
cntTimeOutMR72_3 = 0
cntTimeOutNRA24 = 0
MR72stillConnect = True
NRA24StillConnect = True
def processRadar(message):
	global arrayMR72
	global arrayMR72_1
	global arrayMR72_2
	global arrayMR72_3
	global cntTimeOutMR72
	global cntTimeOutMR72_1
	global cntTimeOutMR72_2
	global cntTimeOutMR72_3
	
	if len(message.data) == 8:
		
		if message.arbitration_id == 0X60A or message.arbitration_id == 0X61A or message.arbitration_id == 0X62A or message.arbitration_id == 0X63A:
			beginToProcessRadar = True
			
			# beginToProcessRadar will be set to False after calculating the vector and visualize
			if (message.data[0] == 0X01) and (message.data[2] == 0x00) and(message.data[3] == 0x00) and (message.data[4] == 0x00) and (message.data[5] == 0x00) and	(message.data[6] == 0x00) and (message.data[7] == 0x00):
				#print('This is NRA24')
				pass
			else:
				if message.arbitration_id == 0X60A:
					sendToUDP(arrayMR72,0)
					cntTimeOutMR72 = 0
					return
					#print '0'
				if message.arbitration_id == 0X61A:
					sendToUDP(arrayMR72_1,1)
					cntTimeOutMR72_1 = 0
					return
					#print '1'
				if message.arbitration_id == 0X62A:
					sendToUDP(arrayMR72_2,2)
					cntTimeOutMR72_2 = 0
					return
					#print '2'
				if message.arbitration_id == 0X63A:
					sendToUDP(arrayMR72_3,3)
					cntTimeOutMR72_3 = 0
					return
					#print '3'
				
	
			
		elif message.arbitration_id == 0X60B:
			beginToProcessRadar = False
			abc = radarObject(0,0,0,0)
			abc.parseMessgageToArrayMR72(message)
			arrayMR72[abc.ID].ID = abc.ID
			arrayMR72[abc.ID].latX = abc.latX
			arrayMR72[abc.ID].longY = abc.longY
			arrayMR72[abc.ID].shownOff = 1
			
		elif message.arbitration_id == 0X61B:
			beginToProcessRadar = False
			abc = radarObject(0,0,0,0)
			abc.parseMessgageToArrayMR72(message)
			arrayMR72_1[abc.ID].ID = abc.ID
			arrayMR72_1[abc.ID].latX = abc.latX
			arrayMR72_1[abc.ID].longY = abc.longY
			arrayMR72_1[abc.ID].shownOff = 1
		elif message.arbitration_id == 0X62B:
			beginToProcessRadar = False
			abc = radarObject(0,0,0,0)
			abc.parseMessgageToArrayMR72(message)
			arrayMR72_2[abc.ID].ID = abc.ID
			arrayMR72_2[abc.ID].latX = abc.latX
			arrayMR72_2[abc.ID].longY = abc.longY
			arrayMR72_2[abc.ID].shownOff = 1
		elif message.arbitration_id == 0X63B:
			beginToProcessRadar = False
			abc = radarObject(0,0,0,0)
			abc.parseMessgageToArrayMR72(message)
			arrayMR72_3[abc.ID].ID = abc.ID
			arrayMR72_3[abc.ID].latX = abc.latX
			arrayMR72_3[abc.ID].longY = abc.longY
			arrayMR72_3[abc.ID].shownOff = 1
		elif message.arbitration_id == 0X70C:
			cntTimeOutNRA24 = 0
			terrain0 = message.data[2]*256 + message.data[3]
			strAll = 'H,' + str(terrain0)
			
			now = datetime.now()
			newLine = now.strftime("%H#%M#%S#%f")
				
			
			sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			sock.sendto(strAll, (UDP_IP_LOCAL, UDP_PORT))
	else:
		print 'CAN bi loi roi!!!'


class myThread (threading.Thread):
	def __init__(self, threadID, name, counter):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.counter = counter
	def run(self):
		print "Starting " + self.name
		try:
			while True:
				message = bus.recv()
				
				processRadar(message)
				#print ('Still alive')
				print message
				#print '--'
				#time.sleep(0.001)
		except KeyboardInterrupt:
			try:
				#abc = open(currenTimeCreateFile, "r") # or "a+", whatever you need
				pass
			except IOError:
				print "Could not open file! Please close Excel!"
			#fileNow.close()
			print('Done.')
		print "Exiting " + self.name


"""
timeOutMR72 = 1000
timeOutNRA24 = 1000
cntTimeOutMR72 = 0t
cntTimeOutNRA24 = 0
MR72stillConnect = True
NRA24StillConnect = True
"""

class countingThread (threading.Thread):
	def __init__(self, threadID, name, counter):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.counter = counter
	def run(self):
		global MR72stillConnect
		global NRA24StillConnect
		global cntTimeOutMR72
		global cntTimeOutMR72_1
		global cntTimeOutMR72_2
		global cntTimeOutMR72_3
		global cntTimeOutNRA24
		print "Starting " + self.name
		while True:
				#print 'a'
				#time.sleep(0.1)
				
				if cntTimeOutMR72 >= timeOutMR72:
					MR72stillConnect = False
					#print ("No MR72 Front")
					sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					sock.sendto('E, nomr72', (UDP_IP_LOCAL, UDP_PORT))
				else:
					MR72stillConnect = True
					cntTimeOutMR72 +=1

					
				if cntTimeOutMR72_1 >= timeOutMR72:
					MR72stillConnect = False
					#print ("No MR72 Right")
					sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					sock.sendto('E, nomr72', (UDP_IP_LOCAL, UDP_PORT))
				else:
					MR72stillConnect = True
					cntTimeOutMR72_1 +=1

					
				if cntTimeOutMR72_2 >= timeOutMR72:
					MR72stillConnect = False
					#print ("No MR72 Rear")
					sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					sock.sendto('E, nomr72', (UDP_IP_LOCAL, UDP_PORT))
				else:
					MR72stillConnect = True
					cntTimeOutMR72_2 +=1

				
				if cntTimeOutMR72_3 >= timeOutMR72:
					MR72stillConnect = False
					#print ("No MR72 Left")
					sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					sock.sendto('E, nomr72', (UDP_IP_LOCAL, UDP_PORT))
				
				else:
					MR72stillConnect = True
					cntTimeOutMR72_3 +=1
					
				if cntTimeOutNRA24 >= timeOutNRA24:
					NRA24StillConnect = False
					#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					#sock.sendto('E, nonra24', (UDP_IP_LOCAL, UDP_PORT))
				else:
					NRA24StillConnect = True
					cntTimeOutNRA24 +=1
					

# Create new threads
thread1 = myThread(1, "Thread main", 1)
thread2 = countingThread(2, "Counter", 1)
#thread2 = myThread(2, "Thread-2", 2)

# Start new Threads
thread1.start()
thread2.start()



"""

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
		#abc = open(currenTimeCreateFile, "r") # or "a+", whatever you need
		pass
	except IOError:
		print "Could not open file! Please close Excel!"
	#fileNow.close()
	print('Done.')
"""
