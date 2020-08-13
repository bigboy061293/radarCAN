import numpy as np
import time
class linProfile:
	vmax = 0.00
	maxSpeedAt = 0.00
	stopAt = 0.00
	timeStep = 0.00
	
	profileLin =[]
	nameProfile = ''
	def __init__(self, vmax, maxSpeedAt, stopAt, timeStep, namepf):
		self.nameProfile = namepf
		self.vmax = vmax
		self.maxSpeedAt = maxSpeedAt
		self.stopAt = stopAt
		self.timeStep = timeStep
		
		speedUp = np.linspace(0, vmax , maxSpeedAt/timeStep)
		speedDown = np.linspace(vmax, 0, (stopAt - maxSpeedAt)/timeStep)
		#print speedUp
		#print speedDown
		self.profileLin = np.concatenate([speedUp, speedDown])

#a = linProfile(10, 5, 10, 0.1)
#print a.vmax
#print a.profileLin.size
		
#for i in range(a.profileLin.size):
	#print a.profileLin[i]
	#time.sleep(a.timeStep)
