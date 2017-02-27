from matplotlib.pylab import *
#import matplotlib.pyplot as plt
import numpy as np
import lib.audio as au
import scipy

import scipy.io.wavfile as wave

# sig = au.PingToNpArray("sound/ping_3m.wav")
rate, data = wave.read("sound/ping_3m.wav")
figure(1)
plot(data[:,0])
figure(2)
plot(data[:,1])
show()