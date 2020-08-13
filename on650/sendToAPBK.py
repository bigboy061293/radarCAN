from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import socket
import time
import math
import inspect
import threading
import os
import ctypes
SIMULATOR_UDP = 'udp:127.0.0.1:14550'
UDP_IP_LOCAL = "127.0.0.1"
UDP_PORT = 2992
UDP_IP_650 = 'tcp:192.168.0.210:20002'

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP_LOCAL, UDP_PORT))

master = mavutil.mavlink_connection(UDP_IP_650, dialect = "ardupilotmega")
mavutil.set_dialect("ardupilotmega")

def connectTo650(master):
    msg = None
    while not msg:
        master.mav.ping_send(
            time.time(), # Unix time
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)




connectTo650(master)


time.sleep(0.5)


mss = mavlink2.MAVLink_distance_sensor_message(
            0, #time
            10, #min distance
            6000, #max distance
            100, # current
            3, #type
            1, #id
            0, #ori
            0#cova
            )

while True:            
    time.sleep(0.1)

    master.mav.send(mss)
