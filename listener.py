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
data = [None, None, None]
threshold = 10
dataLabel = ser.readline().split()[0]
value_stable = False
while dataLabel != 'ultrasonicsensor':
	#wait for initialization 
	dataLabel = ser.readline().split()[0]
print('wait for data to stabilize')
while not value_stable:
	meas = ser.readline().split()
	if meas[0] == "areal":
		value_stable = True
		for i in range(2):
			if abs(float(meas[i+1])) > threshold:
				value_stable = False

while counter < 20:
	if None not in data:
		print(data)
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
		data[0] =  [float(d[1])/8192*980,float(d[2])/8192*980,float(d[3])/8192*980] #unit in cm/sec
	elif d[0] == "ypr":
		data[1] = [float(d[1]),float(d[2]),float(d[3])]
print(datapacket)

