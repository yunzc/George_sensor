"""
Hopefully the program that integrates the others and navigates George
Hopefully 
@author: Yun 
"""
import serial 
import numpy as np
import time
ser = serial.Serial('/dev/ttyACM0', 115200)

"""
==============================================
Here we start up the sensor array for sensing
==============================================
"""

queue_size = 10 #the queue size for the median filter for the ultrasonic sensor 
ultrasonic_sens = np.zeros((queue_size,3)) #ultrasonic array to medain filter
counter = 0 
datapacket = []
c_prev_data = [0, 0, 0] #calibration use
data = [None, None, None]
threshold = 5
dataLabel = ser.readline().split()[0]
value_stable = False
while dataLabel != 'ultrasonicsensor':
	#wait for initialization 
	dataLabel = ser.readline().split()[0]
print('wait for data to stabilize')
while not value_stable: #wait for values to stabilize before action 
	meas = ser.readline().split()
	if meas[0] == "areal":
		value_stable = True
		for i in range(3):
			if float(meas[i+1]) - c_prev_data[i] > threshold:
				value_stable = False
		c_prev_data = [float(meas[1]),float(meas[2]),float(meas[3])]

"""
Below I will write a simple "see obstacle, turn to side without obstacle" program 
=====================================================
Ultrasonic Rough Navigation 
=====================================================
For incorporation of SLAM, creation of data package is in listener.py 
Not incorporated here due to low confidence with only three range finders and also 
Unclear on what final integration is 
"""
counter = 0
while(1): 
	msg = ser.readline()
	d = msg.split()
	range_meas = []
	if d[0] = 'ultrasonicsensor': 
		if counter == 0:
			for i in range(queue_size):
				for j in range(3):
					ultrasonic_sens[i,j] = float(d[j+1])
		else:
			new = ultrasonic_sens.copy()
			for i in range(queue_size-1):
				new[i+1,:] = ultrasonic_sens[i,:] #These 10 lines are all for the median filter 
			new[0,:] = [float(d[1]),float(d[2]),float(d[3])]
			ultrasonic_sens = new
		range_meas = [np.median(ultrasonic_sens[:,0]), np.median(ultrasonic_sens[:,1]), np.median(ultrasonic_sens[:,2])]
	"""
	now from here on we will say if the middle sensor senses below a certain distance, turn towards the side with the further distance 
	if all sides are close, turn around to find another path 
	"""
	"""
	consequently, we can also use SLAM here and plan a path, which will make for smoother flying but more complicated and more computation 
	and also higher chance of failure (basically what I would do would be to generate datapackets and feed it to graphslam every so often 
	to determine our location and landmarks, and then a path can be planned to reach the destination. BUT for now  hard to know what and where
	this "destination" is 
	"""
	