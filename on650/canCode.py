"""

define stuff

"""

thisDroneDatalinkAddress = 'tcp:192.168.0.210:20002'

import can
import time
from pymavlink import mavutil


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
print('----- Starting MAVLINK -----')
master = mavutil.mavlink_connection(thisDroneDatalinkAddress)
master.wait_heartbeat()
print('----- Heartbeat receive -----')



time.sleep(0.1)


def radarType(message):
	if ((((message.arbitration_id >> 8) & 0X0F) == 0X07) and ((message.arbitration_id  & 0X0F) == 0X0C)):
		return 'NR24'
	elif ((((message.arbitration_id >> 8) & 0X0F) == 0X06) and ((message.arbitration_id  & 0X0F) == 0X0B)):
		return 'MR72'
	else:
		return 'NA'
	
def terrainHeight(message):
	return (message.data[2]*256 + message.data[3])
	
while True:
	message = bus.recv()
	#frame = message.split(' ')
	#print (radarType(message))
	#print (type(message.arbitration_id))
	if (radarType(message) == 'NR24'):
		
		print('Down: ', terrainHeight(message))
	elif (radarType(message) == 'MR72'):
		
		print('FW: ', terrainHeight(message))
	
	master.mav.distance_sensor_send(
		0,
		20,
		4000,
		terrainHeight(message),
		3,
		1,
		0,
		2)
	#time.sleep(0.5)
	"""
	print (hex(message.arbitration_id))
	
	if hex(message.arbitration_id) == '0x7ac' :
		print (1)
	else:
		print (2)
	print (message.data[2]*256 + message.data[3])
	"""
