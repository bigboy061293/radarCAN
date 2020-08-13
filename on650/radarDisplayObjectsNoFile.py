"""

define stuff

"""

thisDroneDatalinkAddress = 'tcp:192.168.0.210:20002'

import can
import time
import turtle
import curses
import socket

from datetime import datetime

#import panda as pd


from pymavlink import mavutil

UDP_IP = "192.168.0.111"
UDP_PORT = 2992
beginToProcessRadar = False

arrayMR72Count = 0
screenW = 1000
screenH = 600
radarShownScale = 20
radarHeightLimit = 2
#wn = turtle.Screen()
#wn.title('RtR radar visulization')
#wn.bgcolor('black')
#wn.setup(screenW,screenH)
#wn.tracer(0)

#wnGUI = turtle.Screen()
#wn.title('MF GUI')
#wn.bgcolor('black')
#wn.setup(screenW,screenH)
#wn.tracer(0)

#grid = turtle.Turtle()
#MR72objs = turtle.Turtle()
#grid.color('white')
#grid.circle(120,180)

"""

initialize stream

"""


#HEADER_SIZE = 10
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.bind((socket.gethostname(), 1243))
#s.listen(5)

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
	
		



#arraymr2 = 

arrayMR72 = []
arrayMR72marker = []
arrayMR72Turtle = []
for i in range(256):
	arrayMR72.append(radarObject(0,0,0,0))
	#arrayMR72Turtle.append(turtle.Turtle())
	#arrayMR72Turtle[i].speed(0)
	#arrayMR72Turtle[i].shape('circle')
	#arrayMR72Turtle[i].color('red')
	#arrayMR72Turtle[i].shapesize(stretch_wid = 0.5, stretch_len = 0.5)
	#arrayMR72Turtle[i].penup()
	#arrayMR72Turtle[i].goto(0,0)
	#arrayMR72Turtle[i].hideturtle()


	
#vltest = turtle.Turtle()

#vltest.speed(0)
#vltest.shape('circle')
#vltest.color('red')
#vltest.shapesize(stretch_wid = 0.5, stretch_len = 0.5)
#vltest.penup()
#vltest.goto(0,0)
#vltest.hideturtle()








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
	return timeReal

def calcTotalVectorAndVisualize(arraryMR72):
	
	global arrayMR72
	#global arrayMR72Turtle
	#global wn
	global timeNow
	global timeBefore
	now = datetime.now()
	timeNow = long(now.strftime("%H%M%S%f"))
	timeNow = timeFrom00(timeNow)
	
	fileNow = open(currenTimeCreateFile,'a')
	
	newLine = now.strftime("%H:%M:%S:%f")
	fileNow.write(newLine)
	fileNow.write("\r\n")
	
	
	
	#newLine += newLine + "\r\n"
	#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	#sock.sendto(newLine, (UDP_IP, UDP_PORT))
	#print('----------------------------------')
	#strAll =''
	for i in range(256):
		if arrayMR72[i].shownOff == 1:
			
			#print(arrayMR72[i].ID,arrayMR72[i].latX, arrayMR72[i].longY)
			#fileNow.write(arrayMR72[i].ID,arrayMR72[i].latX, arrayMR72[i].longY)
			#fileNow.write("%d %f %f ",arrayMR72[i],arrayMR72[i].latX, arrayMR72[i].longY)
			strID = str(arrayMR72[i].ID)
			strLatX = str(arrayMR72[i].latX)
			strLongY = str(arrayMR72[i].longY)
			strAll = strID + ',' + strLatX + ',' +  strLongY
			
			fileNow.write(strAll)
			fileNow.write("\r\n")
			
			if (timeNow - timeBefore) > 300:
				sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
				sock.sendto(strAll, (UDP_IP, UDP_PORT))
				timeBefore = timeNow
				print  strAll
			
		
			#stdscr.addstr( 100, 100, 'AA')
			#stdscr.refresh()
			#arrayMR72Turtle[i].showturtle()
			#arrayMR72Turtle[i].goto(arrayMR72[i].latX * radarShownScale, 
		#						arrayMR72[i].longY * radarShownScale - screenH/2)
		#	arrayMR72Turtle[i].write(arrayMR72[i].latX,
		#							False,
		#							align = 'center'
		#							)
			
			#reset the marker array
			arrayMR72[i].shownOff = 0
#	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#	sock.sendto(strAll, (UDP_IP, UDP_PORT))

		
	fileNow.write("------\r\n")
	#strAll = strAll + "\r\n------" 
	fileNow.close()
	#if abs(timeNow - timeBefore) > 300:
	#		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	#	sock.sendto(strAll, (UDP_IP, UDP_PORT))
		#print strAll
		#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		#sock.sendto("------", (UDP_IP, UDP_PORT))
	#	timeBefore = timeNow
	
	
	#wn.update()

def processRadar(message):
	global arrayMR72
	#global testGlb
	#testGlb = 1
	#print(testGlb)
	#global arrayMR72
	if message.arbitration_id == 0X61A:
		beginToProcessRadar = True
		
		# beginToProcessRadar will be set to False after calculating the vector and visualize
		if (message.data[0] == 0X01) and (message.data[2] == 0x00) and(message.data[3] == 0x00) and (message.data[4] == 0x00) and (message.data[5] == 0x00) and	(message.data[6] == 0x00) and (message.data[7] == 0x00):
			#print('This is NRA24')
			pass
		else:
			calcTotalVectorAndVisualize(arrayMR72)
		
		
		
		
		
		#print(arrayMR72[1].ID)
		#print('----------------------------------')
		
		
	elif message.arbitration_id == 0X61B:
		
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
		fileNow = open(currenTimeCreateFile,'a')
		now = datetime.now()
		newLine = now.strftime("%H#%M#%S#%f")
		fileNow.write(newLine)
		fileNow.write("\r\n")
		fileNow.write(strAll)
		fileNow.write("\r\n")
		
		
		
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		sock.sendto(strAll, (UDP_IP, UDP_PORT))
		fileNow.close()
		
		#print(arrayMR72[abc.ID].shownOff)
		#print(arrayMR72[abc.ID].latX)
		#print(arrayMR72[abc.ID].latX)
		#print(abc.latX, abc.longY)
		
		#arrayTurtle[abc.ID].goto(abc.latX*10,abc.longY*10 - screenH/2)
		#arrayTurtle[abc.ID].color('red')
		#arrayTurtle[abc.ID].penup()
		#arrayTurtle[abc.ID].dot()
	


def thread_processRadar(message):
	while True:
		message = bus.recv()
		processRadar(message)

#tRadarProcess = Timer(1,processRadar)
#tRadarProcess.start()

#thr1 = threading.Thread(target=thread_processRadar,args=(bus))
#thr1.start()
#thr1.join()

#def closeFileAndStuff():
	
	
	
#r = tk.Tk()
#r.title('Mother of GUI in RTR')
#buttonSaveLogs = tk.Button(r, text='Save new radar logs', width=25, command=r.destroy) 
#buttonSaveLogs.pack() 

"""
def on_press(key):
	print('{0} pressed'.format(key))
def on_release(key):
	print('{0} release'.format(key))
	if key == Key.esc:
		return False
with Listener(on_press=on_press,on_release=on_release) as listener:
	listener.join()
"""
try:
	while True:
		#clientsocket, address = s.accept()
		#print('Connection from me has been established.')

		#wn.update()
		#time.sleep(0.05)	
	
		message = bus.recv()
		processRadar(message)
except KeyboardInterrupt:
	try:
		abc = open(currenTimeCreateFile, "r") # or "a+", whatever you need
	except IOError:
		print "Could not open file! Please close Excel!"
	fileNow.close()
	print('Done.')
