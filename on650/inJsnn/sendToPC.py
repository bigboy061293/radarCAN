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
from datetime import datetime
UDP_IP = "192.168.0.111"
UDP_PORT = 2992
RPI_SERIAL = '/dev/ttyS0'
can_interface ='can0'
bus = can.interface.Bus(can_interface, bustype = 'socketcan_native')
rd = np.array([0])
print rd
class threadReadRadarPolling (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
        while True:
            message = bus.recv()
            if message.arbitration_id == 0X60B:
                y = (message.data[1] * 32 + (message.data[2] >> 3)) * 0.2 - 500
                x = ((message.data[2]&0X07) * 256 + message.data[3]) * 0.2 - 204.6
                rd = np.append(a,x,1)
                rd = np.append(a,y,2)
                print x,y
            elif message.arbitration_id == 0X60A:
                print "end"
class threadInterval (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
        while True:
            np.delete(rd,1,0)
            np.delete(rd,2,1)
            time.sleep(1)
            pass

time.sleep(0.5)

_threadReadRadarPolling = threadReadRadarPolling(1, "Thread main")
_threadInterval = threadInterval(2,"Main")
_threadReadRadarPolling.start()
_threadInterval.start()