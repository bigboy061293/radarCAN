
import can
import time
import socket
import math
import threading
from datetime import datetime

UDP_IP = "192.168.0.111"
UDP_IP_LOCAL = "127.0.0.1"
UDP_PORT = 2992

print('----- Starting CAN -----')
can_interface ='can0'
bus = can.interface.Bus(can_interface, bustype = 'socketcan_native')
def timeFrom00(timeInInt):
	miSec = timeInInt % 1000000
	timeInInt = timeInInt / 1000000
	sec = timeInInt % 100
	timeInInt = timeInInt / 100
	min = timeInInt % 100
	hour = timeInInt / 100
	timeReal = (hour * 3600 + min * 60 + sec) * 1000000 + miSec
	return timeReal

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
str_front = "front, {"
str_right = "right, {"
str_rear = "rear, {"
str_left = "left, {"
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
timeStart = 0
timeBefore = 0
timeBefore_0 = 0
timeBefore_1 = 0
timeBefore_2 = 0
timeBefore_3 = 0
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msssggg = None
message = bus.recv()
class threadLayMessage(threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
	def run(self):
		#global message
		#global str_front
		#global
		global timeStart
		global timeBefore
		while True:
			#print 'sending'
			#now = datetime.now()
			
			#timeStart = long(now.strftime("%H%M%S%f"))
			#timeStart = timeFrom00(timeStart)
			#if timeStart - timeBefore < 50000:
			#	continue
			#timeBefore = timeStart
			sock.sendto(str_front, (UDP_IP_LOCAL, UDP_PORT))
			sock.sendto(str_right, (UDP_IP_LOCAL, UDP_PORT))
			sock.sendto(str_rear, (UDP_IP_LOCAL, UDP_PORT))
			sock.sendto(str_left, (UDP_IP_LOCAL, UDP_PORT))
			time.sleep(0.05)
			
class threadReadRadarPolling (threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
	
	def run(self):
		#global timeStart
		#global timeBefore
		global str_front
		global str_right
		global str_rear
		global str_left
		global message
		print "Starting " + self.name
		try:
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
				
				#processRadar(message)
				
				#print message
				#continue
				if message.arbitration_id == 0X60B:	
					y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
					x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
					_, mag = xyReturnSec(x,y,1)		
					sector = (message.data[6] >> 3) & 0X03		
					#str_front +=  str(int(sector)) + ":" + str(round(mag,2))
					#if sector !=3:
					#	str_front+=','
					dic_front[sector] = round(mag,2)
				elif message.arbitration_id == 0X60A:
					print 'front, ' + str(dic_front)
					str_front = 'front, ' + str(dic_front)
					#print str_front
					#if timeStart - timeBefore_0 < 5000:
					#	pass
						#continue
					#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					
					#time_before_0 = timeStart
					
					
					
					#now = datetime.now()
					#timeNow = long(now.strftime("%H%M%S%f"))
					#print timeNow, str_front
					#str_front = "front, {"
				elif message.arbitration_id == 0X61B:	
					y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
					x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
					_, mag = xyReturnSec(x,y,1)		
					sector = (message.data[6] >> 3) & 0X03		
					#str_right += str(int(sector)) + ":" + str(round(mag,2))
					#if sector !=3:
					#	str_right+=','
					dic_right[sector] = round(mag,2)
				elif message.arbitration_id == 0X61A:
					print 'right, ' + str(dic_right)
					str_right = 'right, ' + str(dic_right)
					#print str_right
					#if timeStart - timeBefore_1 < 5000:
						#continue
					#	pass
					#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					#sock.sendto(str_right, (UDP_IP_LOCAL, UDP_PORT))
					#time_before_1 = timeStart
					
					#str_right = "right, {"					
				if message.arbitration_id == 0X62B:	
					y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
					x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
					_, mag = xyReturnSec(x,y,1)		
					sector = (message.data[6] >> 3) & 0X03		
					#str_rear += str(int(sector)) + ":" + str(round(mag,2))
					#if sector !=3:
					#	str_rear+=','
					dic_rear[sector] = round(mag,2)
				elif message.arbitration_id == 0X62A:
					print 'rear, ' +  str(dic_rear)
					str_rear = 'rear, ' +  str(dic_rear)
					#print str_rear
					#if timeStart - timeBefore_2 < 5000:
						#continue
					#	pass
					#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					#sock.sendto(str_rear, (UDP_IP_LOCAL, UDP_PORT))
					#time_before_2 = timeStart					
					
					#str_rear = "rear, {"
				if message.arbitration_id == 0X63B:	
					y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
					x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
					_, mag = xyReturnSec(x,y,1)		
					sector = (message.data[6] >> 3) & 0X03		
					
					#str_left += str(int(sector)) + ":" + str(round(mag,2))
					#if sector !=3:
					#	str_left+=','
					dic_left[sector] = round(mag,2)
				elif message.arbitration_id == 0X63A:
					print 'left, ' + str(dic_left)
					str_left = 'left, ' + str(dic_left)
					print str_left
					#if timeStart - timeBefore_3 < 5000:
						#continue
					#	pass
					#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					#sock.sendto(str_left, (UDP_IP_LOCAL, UDP_PORT))
					#time_before_3 = timeStart
					
					#str_left = "left, {"				
		except KeyboardInterrupt:
			try:
				
				pass
			except IOError:
				print "Could not open file! Please close Excel!"
			
			print('Done.')
		print "Exiting " + self.name


_threadReadRadarPolling = threadReadRadarPolling(1, "Thread main")
_threadReadRadarPolling.start()
_threadLayMessage = threadLayMessage(2, "Lay message")
_threadLayMessage.start()
#_threadReadRadarPolling.join()
#_threadLayMessage.join()
