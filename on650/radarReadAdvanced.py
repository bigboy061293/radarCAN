import can
import time
import socket
import math
import threading


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
class threadReadRadarPolling (threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
	
	def run(self):
		global str_front
		global str_right
		global str_rear
		global str_left
		print "Starting " + self.name
		try:
			while True:
				message = bus.recv()
				
				#processRadar(message)
				
				#print message
				
				if message.arbitration_id == 0X60B:	
					y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
					x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
					sec, mag = xyReturnSec(x,y,1)					
					str_front +=  str(int(sec)) + ":" + str(round(mag,2)) + "," 
				elif message.arbitration_id == 0X60A:
					str_front += '}'					
					print str_front
					sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					sock.sendto(str_front, (UDP_IP_LOCAL, UDP_PORT))
					#print str_front
					str_front = "front, {"
				elif message.arbitration_id == 0X61B:	
					y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
					x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
					sec, mag = xyReturnSec(x,y,1)					
					str_right += str(int(sec)) + ":" + str(round(mag,2)) + "," 
				elif message.arbitration_id == 0X61A:
					str_right += '}'					
					print str_right
					sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					sock.sendto(str_right, (UDP_IP_LOCAL, UDP_PORT))
					str_right = "right, {"					
				if message.arbitration_id == 0X62B:	
					y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
					x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
					sec, mag = xyReturnSec(x,y,1)					
					str_rear += str(int(sec)) + ":" + str(round(mag,2)) + "," 
				elif message.arbitration_id == 0X62A:
					str_rear += '}'
					print str_rear					
					sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					sock.sendto(str_rear, (UDP_IP_LOCAL, UDP_PORT))
					str_rear = "rear, {"
				if message.arbitration_id == 0X63B:	
					y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
					x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
					sec, mag = xyReturnSec(x,y,1)					
					str_left += str(int(sec)) + ":" + str(round(mag,2)) + "," 
				elif message.arbitration_id == 0X63A:
					str_left += '}'
					print str_left
					sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
					sock.sendto(str_left, (UDP_IP_LOCAL, UDP_PORT))
					str_left = "left, {"				
		except KeyboardInterrupt:
			try:
				
				pass
			except IOError:
				print "Could not open file! Please close Excel!"
			
			print('Done.')
		print "Exiting " + self.name


_threadReadRadarPolling = threadReadRadarPolling(1, "Thread main")
_threadReadRadarPolling.start()


