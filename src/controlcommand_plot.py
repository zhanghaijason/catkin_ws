#!/usr/bin/env/ python
import matplotlib
import matplotlib.pyplot as plt
import csv

time = []
phi = []
theta = []
i = 0
with open('control_log.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        time.append(row[0])
        phi.append(float(row[1])*180/3.1415)
	theta.append(float(row[2])*180/3.1415)
plt.plot(time,phi, label='phi')
plt.plot(time,theta, label='theta')
plt.xlabel('time (s)')
plt.ylabel('control angle sent')


#plt.title('Interesting Graph\nCheck it out')
plt.legend()
plt.show()
