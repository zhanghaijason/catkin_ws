#!/usr/bin/env/ python
import matplotlib
import matplotlib.pyplot as plt
import csv

time = []
x = []
y = []
z = []
vz = []
az = []
with open('fb_log.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        time.append(float(row[0]))
        x.append(float(row[1]))
        y.append(float(row[2]))
        z.append(float(row[3]))
	vz.append(float(row[4]))
	az.append(float(row[5]))

print("Max velocity is" , max(vz))
print("Max upward acceleration is ", min(az[:-300]))

plt1 = plt.figure(1)
plt.plot(time,x, label='x')
plt.ylim(2.5, 4)
plt.xlabel('time (s)')
plt.ylabel('feedback (m)')
plt.legend()

plt2 = plt.figure(2)
plt.plot(time,y, label = 'y')
plt.ylim(-0.2, 0.2)
plt.xlabel('time (s)')
plt.ylabel('feedback (m)')
plt.legend()

plt3 = plt.figure(3)
plt.plot(time,z, label = 'z')
#plt.ylim(-1.1, -0.7)
plt.xlabel('time (s)')
plt.ylabel('feedback (m)')



plt4 = plt.figure(4)
plt.plot(time,vz, label = 'v_z')
#plt.xlim(0,28)
plt.xlabel('time (s)')
plt.ylabel('Velocity (m)')


plt5 = plt.figure(5)
plt.plot(time,az, label = 'a_z')
plt.xlabel('time (s)')
plt.ylabel('Acceleration (m)')
plt.legend()
plt.show()


