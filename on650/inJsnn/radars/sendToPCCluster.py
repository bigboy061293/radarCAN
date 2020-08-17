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


MAX_LOOP = 10
DCOST_TH = 0.1
show_animation = True


def kmeans_clustering(rx, ry, nc):
    clusters = Clusters(rx, ry, nc)
    clusters.calc_centroid()

    pre_cost = float("inf")
    for loop in range(MAX_LOOP):
        print("loop:", loop)
        cost = clusters.update_clusters()
        clusters.calc_centroid()

        d_cost = abs(cost - pre_cost)
        if d_cost < DCOST_TH:
            break
        pre_cost = cost

    return clusters


class Clusters:

    def __init__(self, x, y, n_label):
        self.x = x
        self.y = y
        self.n_data = len(self.x)
        self.n_label = n_label
        self.labels = [random.randint(0, n_label - 1)
                       for _ in range(self.n_data)]
        self.center_x = [0.0 for _ in range(n_label)]
        self.center_y = [0.0 for _ in range(n_label)]

    def plot_cluster(self):
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            plt.plot(x, y, ".")

    def calc_centroid(self):
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            n_data = len(x)
            self.center_x[label] = sum(x) / n_data
            self.center_y[label] = sum(y) / n_data

    def update_clusters(self):
        cost = 0.0

        for ip in range(self.n_data):
            px = self.x[ip]
            py = self.y[ip]

            dx = [icx - px for icx in self.center_x]
            dy = [icy - py for icy in self.center_y]

            dist_list = [math.hypot(idx, idy) for (idx, idy) in zip(dx, dy)]
            min_dist = min(dist_list)
            min_id = dist_list.index(min_dist)
            self.labels[ip] = min_id
            cost += min_dist

        return cost

    def _get_labeled_x_y(self, target_label):
        x = [self.x[i] for i, label in enumerate(self.labels) if label == target_label]
        y = [self.y[i] for i, label in enumerate(self.labels) if label == target_label]
        return x, y

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
					#sock.sendto(str(x) + ',' + str (y), (UDP_IP,UDP_PORT))
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
					#sock.sendto('----', (UDP_IP,UDP_PORT))
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


