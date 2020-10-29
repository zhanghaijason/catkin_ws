#!/usr/bin/env/ python
import matplotlib
import matplotlib.pyplot as plt
import csv

time = []
x = []
yaw = []
yaw_fit = []
with open('angle_log.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        time.append(float(row[0]))
        x.append(float(row[1]))
        yaw.append(float(row[2]))
	yaw_fit.append(float(row[3]))
plt1 = plt.figure(1)
plt.plot(time,x, label='y')
plt.xlabel('time (s)')
plt.ylabel('pixel distance')


plt2 = plt.figure(2)
plt.plot(yaw, label = 'yaw_estimation')
plt.plot(yaw_fit, label = 'yaw_kalman')
plt.xlabel('time (s)')
plt.ylabel('yaw (degree)')

#plt.title('Interesting Graph\nCheck it out')
plt.legend()
plt.show()
