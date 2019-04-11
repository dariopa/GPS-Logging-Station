import numpy as np
import matplotlib as mpl
from matplotlib.pyplot import imshow
import matplotlib.pyplot as plt
import mpl_toolkits
from mpl_toolkits.mplot3d import Axes3D

filename = 'POS.pos'
measInterval = 1 # seconds!

# PARSING DATA
with open(filename, 'r') as filehandle:  
    date = []
    time = []
    latitude = []
    longitude = []
    height = []
    quality = []
    for line in filehandle:
        if line[0] != '%':
            values = line.split()
            date.append(values[0])
            time.append(values[1])
            latitude.append(float(values[2]))
            longitude.append(float(values[3]))
            height.append(float(values[4]))
            quality.append(int(values[5]))

fixCounter = 0

for i in range(0,len(quality)):
    if quality[i] == 1:
        fixCounter += 1
FixFloatRate = float(fixCounter)/float(len(quality))*100

timeCounter = 0
for i in range(0,len(quality)):
    if quality[i] != 1:
        timeCounter += 1
    else:
        print('Stopped after ' + str(i) + ' sec')
        break
timeToFirstFix = timeCounter * measInterval


# GENERATE PLOTS
color = []
for i in range(0, len(date)):
    if quality[i] == 1:
        color.append('g')
    elif quality[i] == 2:
        color.append('b')
    else:
        color.append('r')


fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
ax.plot(xs=longitude, ys=latitude, zs=height, zdir='z', c='k', linewidth=0.5, label='Position in Space')
ax.scatter(xs=longitude, ys=latitude, zs=height, zdir='z', c=color)
plt.title('3D Representation \nFIXED: %1.1f%%' % FixFloatRate + ' after ' + str(timeToFirstFix) + ' sec')
plt.show()

plt.figure(2)
plt.plot(longitude, latitude, 'k')
plt.scatter(longitude, latitude, c = color)
plt.title('2D Representation (long/lat-plane) \nFIXED: %1.1f%%' % FixFloatRate + ' after ' + str(timeToFirstFix) + ' sec')
plt.show()
# plt.savefig()

plt.figure(3)
plt.title('Time-Quality Plot \nFIXED: %1.1f%%' % FixFloatRate + ' after ' + str(timeToFirstFix) + ' sec')
plt.xlabel('Time (sec)')
# plt.xticks(rotation=90)
plt.ylabel('Fix vs. Float')
plt.plot(np.arange(1, 1+len(date)), quality, 'k')
plt.scatter(np.arange(1, 1+len(date)), quality, c = color)
plt.show()