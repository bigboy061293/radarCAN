#while True:
	
#	wn.update()
#	message = bus.recv()
#	processRadar(message)
	
	
	
	
	#valu = raw_input()
	#if valu == 'a':
	#	print ('aaaaaaaaaaaa')
	#globals()['arrayMR72']
	
	#if message.arbitration_id == 0X60B:
	#	abc = radarObject(message)
	#	print(abc.latX, abc.longY)	 
	
	
	"""
	arrayTurtle[0].goto(0, 0)
	arrayTurtle[0].color('red')
	arrayTurtle[0].penup()
	arrayTurtle[0].dot()
	
	
	time.sleep(2)
	wn.update()
	arrayTurtle[1].goto(10, 10)
	arrayTurtle[1].color('red')
	arrayTurtle[1].penup()
	arrayTurtle[1].dot()
	
	time.sleep(2)
	
	"""
	
	#frame = message.split(' ')
	#print (radarType(message))
	#print (type(message.arbitration_id))
	
	"""
	if (radarType(message) == 'NR24'):
		
		print('Down: ', terrainHeight(message))
	elif (radarType(message) == 'MR72'):
		
		print('FW: ', terrainHeight(message))
	"""
	
	
	"""
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
	
	
	"""
	print (hex(message.arbitration_id))
	
	if hex(message.arbitration_id) == '0x7ac' :
		print (1)
	else:
		print (2)
	print (message.data[2]*256 + message.data[3])
	"""
