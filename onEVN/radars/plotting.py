import time
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(np.random.random((50,50)))
plt.show(block =False)

data = np.empty((50,50))
data[:,:] = np.nan

#ptlist = np.array(ptlist)
#data[ptlist[:,1].astype('int'), ptlist[:,0].astype('int')] = ptlist[:,2]

a = np.empty((50,50))
a = a * 100
print a



for i in range(100):
	time.sleep(0.01)
	im.set_array(np.random.random((50,50)) * 100)
	#im.set_array(data)
	fig.canvas.show()	

