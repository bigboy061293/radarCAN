import socket
import time
import math
import inspect
import threading
import os
import ctypes
import subprocess, signal
import numpy as np
import can
import cv2
from datetime import datetime
import curses
from curses import wrapper
import scipy
from scipy.cluster.vq import kmeans,vq,whiten


UDP_IP = "192.168.0.111"
UDP_PORT = 2992

RPI_SERIAL = '/dev/ttyS0'
can_interface ='can0'
bus = can.interface.Bus(can_interface, bustype = 'socketcan_native')
rd = np.array([[0,0]])
tappings = 5

tempX = []
tempY = []
blkim = np.zeros((800,400,3),np.uint8)
spc = np.array([[800,400]])


class threadReadInput (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
			while True:
				cm = raw_input()
				if cm == 't':
					print ("Terminate process: ", os.getpid())
					os.kill(os.getpid(), signal.SIGKILL)
				time.sleep(0.01)
class threadReadRadarPolling (threading.Thread):
    def __init__(self, threadID, name):
			threading.Thread.__init__(self)
			self.threadID = threadID
			self.name = name
    def run(self):
			global rd
			while True:
				message = bus.recv()
				if message.arbitration_id == 0X60B:
					y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
					x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
					if x <= -40:
						x = -40
					elif x>= 40:
						x = 40
					if y >= 40:
						y = 40
					elif y < 0:
						y = 0
					#superHugeArray[ int(round((x + 40)*10)), int(round(y*10)) ,0 ] = 1
					#spc
					rd = np.append(rd,[[int(round(x,5) * 10), int(round(y,5) * 10)]],axis =0)
					#print rd
					#blkim[int(x*10) + 400,int(y*10),0] = 255
					#blkim[int(x*10) + 400,int(y*10),1] = 255
					#blkim[int(x*10) + 400,int(y*10),2] = 255
					#blkim[400,400,0] = 255
					#blkim[400,400,1] = 255
					#blkim[400,400,2] = 255
				elif message.arbitration_id == 0X60A:
					#print rd
					#print "-----------------------------------"
					#blkim = np.zeros((800,800,3),np.uint8)
					#print rd[:,0]
					#rd = np.delete(rd,np.s_[0:np.size(rd,0)-1],0)
					pass
			
					
										

class threadInterval (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
				pass
				#global rd
				

				#animation.FuncAnimation(fig, _update_plot, interval = 1)

				#plt.show()
				#while True:
					
					
					#print rd[:,0]
					#cv2.imshow('img',blkim)
					#cv2.waitKey(500)
					#cv2.destroyAllWindows()
					#time.sleep(0.1)
					#pass
					#print rd
					#print "-----------------------------------"
					#rd = np.delete(rd,np.s_[0:np.size(rd,0)-1],0)
					#time.sleep(0.1)
#_threadReadRadarPolling = threadReadRadarPolling(1, "Thread main")
		#_threadInterval = threadInterval(2,"AAA")
		#_threadReadInput = threadReadInput(3,"Read input")
#_threadReadRadarPolling.setDaemon(True)
#_threadReadRadarPolling.start()
		#_threadInterval.start()
		#_threadReadInput.start()

def main(stdscr):
		global rd
		#global superHugeArray
		win = curses.newwin(400,400,0,0)
		curses.initscr()
		stdscr.clear()

		_threadReadRadarPolling = threadReadRadarPolling(1, "Thread main")
		#_threadInterval = threadInterval(2,"AAA")
		#_threadReadInput = threadReadInput(3,"Read input")
		#_threadReadRadarPolling.setDaemon(True)
		_threadReadRadarPolling.start()
		#_threadInterval.start()
		#_threadReadInput.start()
		i =0
		while True:
				#i = i + 1
				stdscr.clear()
				if np.size(rd,0) > 10:
					data = whiten(rd)
					xxx,dis = kmeans(data,10)
					stdscr.addstr(10,10,str(np.size(xxx,0)))

				#stdscr.addstr(10,10,str(size(centroid,0)))
				#stdscr.addstr(str(rd[:,0]))
				#stdscr.addstr(10,10,'+')
				#print 0rd[0,0]
				
						
						#print "yes"
				#for i in range(800):
					#for j in range(400):
						#pass
						#if superHugeArray[i,j,0] is 1:
							#pass
							#stdscr.addstr(j/10, i/10, 'O')
							#superHugeArray[i,j,0] = 0
							
							
							#stdscr.addstr(10,10,str(i))
							#print int(rd[i,0]), int(rd[i,1])
							#stdscr.addstr(int(rd[i,1]), int(rd[i,0]) + 40, 'O')
							#stdscr.addstr(10,40, '+')
						#rd = np.delete(rd,np.s_[0:np.size(rd,0)-1],0)
				time.sleep(0.1)
				stdscr.refresh()


wrapper(main)


