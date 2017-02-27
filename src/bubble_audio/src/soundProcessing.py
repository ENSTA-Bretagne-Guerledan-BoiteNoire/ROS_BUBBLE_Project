import scipy
import matplotlib.pylab as plt
import numpy as np
from numpy import pi
# import lib.audio as audio
from scipy.signal import butter, lfilter
import scipy.io.wavfile as wav



def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a


def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

def detection():
    pass

if __name__=="__main__":
    # audio.PingToNpArray()
    # butter_bandpass_filter(data, 14500, 15500, 44100, order=5)
    rate, data=wav.read('ping_3m.wav')
    filtred=butter_bandpass_filter(data.T[1], 10000, 12000, 44100, order=5)
    # print filtred.size

    plt.figure(1)
    plt.plot(filtred)
    plt.show()
