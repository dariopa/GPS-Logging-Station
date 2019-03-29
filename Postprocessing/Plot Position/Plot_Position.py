import numpy as np
import matplotlib as mpl
from matplotlib.pyplot import imshow
import matplotlib.pyplot as plt
import mpl_toolkits
from mpl_toolkits.mplot3d import Axes3D

filename = 'POS.pos'

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

# GENERATE PLOTS
color = []
for i in range(0, len(date)):
    if quality[i] == 1:
        color.append('green')
    elif quality[i] == 2:
        color.append('blue')
    else:
        color.append('red')

plt.figure(1)
plt.scatter(longitude, latitude, c = color)

plt.show()
'''
fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
ax.plot(longitude, latitude, height, label='Position in Space', color = color)
plt.show()
'''

