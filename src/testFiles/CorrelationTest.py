import numpy as np
from matplotlib import pylab as  plt
from math import *


#a,b = [0,1,2,3,4,5,6,0,0,0,0,0],[0,0,0,3,4,5,6,7,8,0,0,0]

Fs = 8000
f = 100
sample = 8000
x = np.arange(sample)
t = np.arange(500)

y = np.zeros(sample)
y[1000:1500] = np.sin(2 * np.pi * f * t / Fs)

z = np.zeros(sample)
z[5000:5500] = np.sin(2 * np.pi * f * t / Fs)


plt.figure()
plt.plot(x,y)
plt.plot(x,z)
plt.figure()
plt.plot(np.correlate(y,z, "full"), "red")
plt.plot(np.correlate(y,z, "same"))
plt.show()
