# -*- coding: utf-8 -*-
"""
Created on Sat Feb 04 20:58:20 2017
listening to the arduino messages
@author: Yun
"""
import serial 
import numpy as np
import time
ser = serial.Serial('/dev/ttyACM0', 115200)

queue_size = 10
ultrasonic_sens = np.zeros((10,3)) #ultrasonic array to medain filter
counter = 0 
datapacket = []
c_prev_data = [0, 0, 0] #calibration use
data = [None, None, None]
threshold = 0.05
dataLabel = ser.readline().split()[0]
value_stable = False
while dataLabel != 'ultrasonicsensor':
	#wait for initialization 
	dataLabel = ser.readline().split()[0]
print('wait for data to stabilize')
while not value_stable:
	meas = ser.readline().split()
	if meas[0] == "ypr":
		value_stable = True
		for i in range(3):
			if float(meas[i+1]) - c_prev_data[i] > threshold:
				value_stable = False
		c_prev_data = [float(meas[1]),float(meas[2]),float(meas[3])]

while counter < 10:  
	print data
	if None not in data:
		datapacket.append(data)
		counter += 1
		data = [None, None, None]
	msg = ser.readline()
	d = msg.split() #extract data 
	if d[0] == 'ultrasonicsensor':
		if counter == 0:
			for i in range(queue_size):
				for j in range(3):
					ultrasonic_sens[i,j] = float(d[j+1])
		else:
			new = ultrasonic_sens.copy()
			for i in range(queue_size-1):
				new[i+1,:] = ultrasonic_sens[i,:]
			new[0,:] = [float(d[1]),float(d[2]),float(d[3])]
			ultrasonic_sens = new
		data[2] = [np.median(ultrasonic_sens[:,0]), np.median(ultrasonic_sens[:,1]), np.median(ultrasonic_sens[:,2])]
	elif d[0] == "areal":
		data[0] =  [float(d[1]),float(d[2]),float(d[3])]
	elif d[0] == "ypr":
		data[1] = [float(d[1]),float(d[2]),float(d[3])]
print(datapacket)

