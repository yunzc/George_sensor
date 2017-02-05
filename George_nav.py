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
while not value_stable: #wait for value to stabilize before action 
	meas = ser.readline().split()
	if meas[0] == "areal":
		value_stable = True
		for i in range(2):
			if abs(float(meas[i+1])) > threshold:
				value_stable = False


"""
Below I will write a simple "Vector Field Histogram obstacle avoidance" program 
=====================================================
Ultrasonic Rough Navigation 
=====================================================
For incorporation of SLAM, creation of data package is in listener.py 
Not incorporated here due to low confidence with only three range finders and also 
Unclear on what final integration is 
"""
##############################################
#some vector field histogram parameters
##############################################
d_max = 400 #max ultrasonic sense range 
#want a - b*d_max = 0
a = 800 #this effects the weight difference obstacle distance makes 
b = a/d_max
c = 1 #certainty value of each detection, just say 1 
sensor_fov = 15. #ultrasonic sensor sensing angle is 15 degs according to specs 
sensor_angs = [-44.5, 0 , 44.5]
nav_angle = (sensor_angs[-1] + sensor_fov/2.)*2
sect_angle = 104/3.
num_sects = nav_angle/sect_angle #navigation 
###############################################

counter = 0
while(1): 
	msg = ser.readline()
	d = msg.split()
	range_meas = []
	if d[0] == 'ultrasonicsensor': 
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
	 	#Now we construct a Polar Vector Field Histogram from the range_meas 
	 	#The twist I will put on vector histogram is that I will restrict it's navigation within the 120 degrees in front of it 
	 	#if density is uniform and above a threshold, stop and turn 
	 	#also I will use 5 degree partitions, so there will be 9 sectors 
	 	vf_vectors = [] #histogram vector, consist of lists of [angle, magnitude]
	 	for i in range(3): #there are 3 sensors
	 		d = range_meas[i]
	 		magnitude = (c**2)*(a - b*d)
	 		anglerange = [(sensor_angs[i] - sensor_fov/2 + j) for j in range(int(sensor_fov))]
	 		for ang in anglerange: 
	 			vf_vectors.append([ang, magnitude])
	 	sect_density = [] #sectors of 5 degrees each total of 24 sectors 
	 	#currently thinking about navigating the turning using the IMU
	 	ang_n = 0
	 	for n in range(int(num_sects)):
	 		mass_density = 0
	 		if ang_n < 45:
		 		while vf_vectors[ang_n][0] < -nav_angle/2. + sect_angle*(n+1):
		 			mass_density += vf_vectors[ang_n][1]
		 			ang_n += 1
		 			if ang_n == 45:
		 				break
		 		sect_density.append(mass_density)
	 	dir_index = sect_density.index(min(sect_density))
	 	print(sensor_angs[dir_index])
	 	"""
	 	after this listen to imu yaw and then turn until yaw exceeds the sensor_angs[dir_index]
	 	"""




	