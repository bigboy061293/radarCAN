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

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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
					sock.sendto(str(x) + ',' + str (y), (UDP_IP,UDP_PORT))
				elif message.arbitration_id == 0X60A:
					
					pass
			
					
										

class threadInterval (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
				pass
				
			
				while True:
					sock.sendto('----', (UDP_IP,UDP_PORT))
					time.sleep(0.5)

					pass
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
_threadReadRadarPolling = threadReadRadarPolling(1, "Thread main")
_threadInterval = threadInterval(2,"AAA")
_threadReadInput = threadReadInput(3,"Read input")
#_threadReadRadarPolling.setDaemon(True)
_threadReadRadarPolling.start()
_threadInterval.start()
_threadReadInput.start()


