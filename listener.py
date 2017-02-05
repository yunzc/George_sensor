# -*- coding: utf-8 -*-
"""
Created on Sat Feb 04 20:58:20 2017
listening to the arduino messages
@author: Yun
"""
import serial 
import numpy as np
ser = serial.Serial('/dev/ttyACM0', 115200)

queue_size = 10
ultrasonic_sens = np.zeros((10,3)) #ultrasonic array to medain filter
counter = 0 
datapacket = []
data = [None, None, None]
while counter < 10: 
    if None not in data: 
        datapacket.append(data)
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
            data[2] = [np.median(new[:,0]), np.median(new[:,1]), np.median(new[:,2])]
            counter += 1
    elif d[0] == "areal":
        data[0] =  [float(d[1]),float(d[2]),float(d[3])]
    elif d[1] == "ypr":
        data[1] == [float(d[1]),float(d[2]),float(d[3])]

