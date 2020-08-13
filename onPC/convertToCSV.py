import socket
import time
import turtle

#import matplotlib.animation as animation
from matplotlib import style
import sys

#from PyQt4.QtWidgets import QApplication

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation
from datetime import datetime


fileName = "15033259906517"
deltaTime = 1000000 #500000 is 500ms
timeInFile = 0
timeBefore = 0

class radarObject:
	
	ID = 0
	IDx = 0
	latX = 0.0
	longY = 0.0
	shownOff = 0

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
#	arrayMR72Turtle.append(turtle.Turtle())
#	arrayMR72Turtle[i].speed(0)
#	arrayMR72Turtle[i].shape('circle')
#	arrayMR72Turtle[i].color('red')
#	arrayMR72Turtle[i].shapesize(stretch_wid = 0.5, stretch_len = 0.5)
#	arrayMR72Turtle[i].penup()
#	arrayMR72Turtle[i].goto(0,0)
#	arrayMR72Turtle[i].hideturtle()
		
tempID = []
tempLatX = []
tempLongY = []
x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
y = [10, 8, 6, 4, 2, 20, 18, 16, 14, 12]


plt.ion()
fig, ax = plt.subplots()

sc = ax.scatter(tempLatX,tempLongY)
plt.xlim(-40,40)
plt.ylim(0,40)
#30000
plt.draw()
now = datetime.now()
timeSytemNow = long(now.strftime("%H%M%S%f"))
beforeTimeCreateFile = timeSytemNow
timeOffset = 0
ft = 0
rf =[]
f = open(fileName, "r")

#for line in f:
#	rf.append(line)
#	print(f)
rf = (f.readlines())
f.close()
#print(rf[2])
lenRF = len(rf)
print(lenRF)
#print((len(rf)))
#i = long(0)
def timeFrom00(timeInInt):
	miSec = timeInInt % 1000000
	timeInInt = timeInInt / 1000000
	sec = timeInInt % 100
	timeInInt = timeInInt / 100
	min = timeInInt % 100
	hour = timeInInt / 100
	timeReal = (hour * 3600 + min * 60 + sec) * 1000000 + miSec
	#print (hour)
	#print (min)
	#print (sec)
	#print (miSec)
	
	return(timeReal)

while i < lenRF:
	i = i + 1;
	#get time now (dummy variable name: "timeSytemNow")
	#now = datetime.now()
	#timeSytemNow = long(now.strftime("%H%M%S%f"))
	#print (rf[i])
	if ':' in rf[i]:
		if ft == 0:
			now = datetime.now()
			timeGet = long(now.strftime("%H%M%S%f"))
			lineInt = rf[i].translate(None, ':')
			timeOffset = long(lineInt)
			
			
			timeOffset = (timeFrom00(timeGet) - timeFrom00(timeOffset)) # time offset = time system - time file
			ft = 1
		else:
			#get time from log file => dummy name "timeInFile"
			print(rf[i])
			lineInt = rf[i].translate(None, ':')
			timeInFileRaw = long(lineInt)
			
			timeInFile = timeFrom00(long(lineInt))
			
			now = datetime.now()
			
			timeSytemNow = timeFrom00(long(now.strftime("%H%M%S%f")))
			#print (timeInFile)
			#print (timeSytemNow)
			#print('+++++')
			
			if (timeSytemNow - timeInFile) < timeOffset:
				while (timeSytemNow - timeInFile) < timeOffset:
					now = datetime.now()
					timeSytemNow = timeFrom00(long(now.strftime("%H%M%S%f")))
					#print('----')
					#print(timeSytemNow - timeInFile)
					#print(timeOffset)
					if (timeSytemNow - timeInFile) > timeOffset:
						break
			else:
				while (timeSytemNow - timeInFile) >= timeOffset:
					i = i + 1;
					if ':' in rf[i]:
						lineInt = rf[i].translate(None, ':')
						timeInFile = timeFrom00(long(lineInt))
					
				#print('bbb')
			#	time.sleep(0.1)
			#	print('aaa')
			#	if (time
			#	print((timeInFile - timeSytemNow))
			#	print(timeOffset)
			#	print('+++++')
			#	print('zzz')
			#	while True:
			#		i = i + 1
			#		
			#		if ':' in rf[i]:
			#			lineInt = rf[i].translate(None, ':')
			#			timeInFile = int(lineInt)
			#			del tempID[:]
			#			del tempLatX[:]
			#			del tempLongY[:]
			#			if abs(timeInFile - timeSytemNow) > timeOffset:
			#				break
			#		if ',' in rf[i]:
			#
			#			#print(line)
			#			abc = rf[i].split(',')
			#			tempID.append(int(abc[0]))
			#			tempLatX.append(float(abc[1]))
			#			tempLongY.append(float(abc[2]))
			#		
			#	print ('zzz')
			
			#	print(timeSytemNow)
			#	print(timeInFile)
			#	print(timeInFile - timeSytemNow)
			#	print(timeOffset)
				
			#	print('-------------')
				
			#now = datetime.now()
			#timeSytemNow = int(now.strftime("%H%M%S%f"))
			
			#print(timeSytemNow)
			#print(timeInFileRaw)
			#print(timeInFile)
			#print(timeSytemNow - timeInFile)
			#print(timeOffset)
				
			#print('-------------')
					#process ploting here
		#now = datetime.now()
		#print(int(rf[i].translate(None, ':')))
		sc.set_offsets(np.c_[tempLatX,tempLongY])
		fig.canvas.draw_idle()
		plt.pause(0.001)
				
				
		del tempID[:]
		del tempLatX[:]
		del tempLongY[:]
				
	if ',' in rf[i]:
			
		#print(line)
		abc = rf[i].split(',')
		if abc[0] == 'H':
			print 'Height: ', int(abc[1])
		else:
			tempID.append(int(abc[0]))
			tempLatX.append(float(abc[1]))
			tempLongY.append(float(abc[2]))
		
		
#with (open(fileName, 'r')) as input:
#	print input
"""	
with (open(fileName, 'r')) as input:
	
	for line in input:
		
		now = datetime.now()
		timeSytemNow = int(now.strftime("%H%M%S%f"))
		
		
		
		if ':' in line:
			if ft == 0:
				now = datetime.now()
				timeGet = int(now.strftime("%H%M%S%f"))
				lineInt = line.translate(None, ':')
				timeOffset = int(lineInt)
				timeOffset = abs(timeGet - timeOffset)
				ft = 1
			else:
				#print(line)
				lineInt = line.translate(None, ':')
				#print(lineInt)
				timeInFile = int(lineInt)
				
				#print(lineInt)
				# doan check nay se an shit neu chay qua ngay moi
				# (23g -> 24g)
				#if (timeSytemNow - beforeTimeCreateFile) >= deltaTime: 
			
				#if (abs(timeInFile - timeBefore)) >= deltaTime: 
				
				#	print(lineInt)
				#	sc.set_offsets(np.c_[tempLatX,tempLongY])
				#	fig.canvas.draw_idle()
				#	#process ploting here
				#	timeBefore = timeInFile
				#	beforeTimeCreateFile = timeSytemNow
				#	plt.pause(0.001)
				while abs(timeInFile - timeSytemNow) > timeOffset:
					now = datetime.now()
					timeSytemNow = int(now.strftime("%H%M%S%f"))
					
					print(timeSytemNow)
					print(timeInFile)
					print(timeInFile - timeSytemNow)
					print(timeOffset)
				
					print('-------------')
					
					#print(lineInt)
					sc.set_offsets(np.c_[tempLatX,tempLongY])
					fig.canvas.draw_idle()
					#process ploting here
					#timeBefore = timeInFile
					#beforeTimeCreateFile = timeSytemNow
					#plt.pause(0.001)
				del tempID[:]
				del tempLatX[:]
				del tempLongY[:]
				
		if ',' in line:
			
			#print(line)
			abc = line.split(',')
			tempID.append(int(abc[0]))
			tempLatX.append(float(abc[1]))
			tempLongY.append(float(abc[2]))
	
	#
	
	
#fileProcess.close()
"""