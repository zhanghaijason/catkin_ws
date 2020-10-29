#!/usr/bin/env/ python
import matplotlib
import matplotlib.pyplot as plt
import csv

time = []
ttc_ref = []
ttc = []
with open('ttc.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        time.append(float(row[0]))
        ttc_ref.append(float(row[1]))
        ttc.append(float(row[2]))

plt.plot(time,ttc_ref, label='ttc_ref')
plt.plot(time,ttc, label = 'ttc')
plt.xlabel('time (s)')
plt.ylabel('TTC (s)')

#plt.title('Interesting Graph\nCheck it out')
plt.legend()
plt.show()
