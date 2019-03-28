'''Visualises the data file for cs410 camera calibration assignment
To run: %run LoadCalibData.py
'''
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

data = np.loadtxt('trajectory.txt')

fig = plt.figure()
ax = fig.gca(projection="3d")
ax.plot(data[:,1], data[:,2], data[:,3],'r.')

plt.show()

