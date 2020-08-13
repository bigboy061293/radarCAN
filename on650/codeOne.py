import time
from pymavlink import mavutil
import pymavlink

print(pymavlink.__doc__)
#master.reboot_autopilot()
def wait_conn(master):
	msg = None
	while not msg:
		master.mav.ping_send(time.time(),0,0,0)
		msg = master.recv_match()
		print(msg)
		time.sleep(0.5)
		print('1')
		
master = mavutil.mavlink_connection('tcp:192.168.0.210:20002')
	
wait_conn(master)
print('2')
while True:
	#try:
	#	print(master.recv_match().to_dict())
	print('3')
	#except:
	#	pass
	#time.sleep(0.1)
	msg = master.recv_match()
	if not msg:
		continue
	print(msg.get_type())
	
	#master.mav.heartbeat_send(1,3,64,0,0,1)
	master.mav.command_long_send(
		master.target_system,
		master.target_component,
		mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
		0,
		1,0,0,0,0,0,0)
	
	#master.mav.request_data_stream(
	#	master.target_system,
	#	master.target_component,
	#	1,
	#	1,
	#	1)
	time.sleep(1)
	
