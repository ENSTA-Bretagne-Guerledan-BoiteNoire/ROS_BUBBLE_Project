import scipy.io.wavfile as sciwave
import scipy.signal as signal
import pylab
import numpy as np
import math
import time
import pyaudio
import wave

from scipy.signal import butter, lfilter


def recPing(WAVE_OUTPUT_FILENAME): #enregistre sous le nom "son.wav" 1 seconde de l'entree micro
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 2
    RATE = 44100
    RECORD_SECONDS = 5

    p = pyaudio.PyAudio()

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    print("* recording")

    frames = []

    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("* done recording")

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()


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


def filter_spectrograms(x,fs,name='signal',show=False):
    if show:
        # Plot of the initial spectrogram of the signal
        pylab.figure()
        f, t, Sxx = signal.spectrogram(x, fs, nfft=2048)
        pylab.pcolormesh(t, f, np.log(Sxx))
        pylab.ylabel('Frequency [Hz]')
        pylab.xlabel('Time [sec]')
        pylab.title(name+' : spectrogramme logarithmique du signal initial')

    # Filter definition
    lowcut = 14000.0
    highcut = 16000.0
    lowcut = 2000.0
    highcut = 4000.0

    if show:
        # Plot the frequency response for an order.
        pylab.figure()
        pylab.clf()
        b, a = butter_bandpass(lowcut, highcut, fs, order=3)
        w, h = signal.freqz(b, a, worN=2000)
        pylab.plot((fs * 0.5 / np.pi) * w, abs(h), label="order = %d" % 3)
        pylab.xlabel('Frequency (Hz)')
        pylab.ylabel('Gain')
        pylab.grid(True)
        pylab.legend(loc='best')
        pylab.title(name + ' : representation du filtre')

    # Filter signal
    x_f = butter_bandpass_filter(x, lowcut, highcut, fs, order=3)

    if show:
        # Plot filtered signal
        pylab.figure()
        pylab.plot(x_f)
        pylab.title(name + ' : filtered temporal signal')

        # Plot spectrogram of filtered signal
        pylab.figure()
        f, t, Sxx = signal.spectrogram(x_f, fs, nfft=2048)
        pylab.pcolormesh(t, f, Sxx)
        pylab.ylabel('Frequency [Hz]')
        pylab.xlabel('Time [sec]')
        pylab.title(name + ' : spectrogram of filtered signal')

    return x_f


#retourne angle entre la droite formee par les 2 hydrophones et la boite noire (90 = boite noire en face)
def calculAngle(file,d,vSonEau,freqEch):
    seuil = 0
    rate, sound = sciwave.read(file)
    #print(rate)

    channel1_f = filter_spectrograms(sound[:,0],rate,'Channel 1')
    channel2_f = filter_spectrograms(sound[:,1],rate,'Channel 2')

    xa_1 = signal.hilbert(channel1_f)
    AI_1 = abs(xa_1)
    #AI_1 = np.clip(AI_1,seuil,np.inf);

    xa_2 = signal.hilbert(channel2_f)
    AI_2 = abs(xa_2)
    #AI_2 = np.clip(AI_2,seuil,np.inf);

    AI1_extended = AI_1[AI_1.argmax()-10000:AI_1.argmax()+1+10000]-4.1
    AI2_cible = (AI_2[AI_1.argmax()-10000:AI_1.argmax()+1+5000]-4.1)
    diff_corr = np.correlate(AI1_extended,AI2_cible)

    pylab.figure()
    pylab.plot(AI1_extended)
    pylab.plot(AI2_cible)
    pylab.title('signal AI1 et AI2 tronques')

    pylab.figure()
    pylab.plot(diff_corr)
    pylab.title('signal correle entre AI1 et AI2 tronque')


    #print('np.max(diff_corr)',np.max(diff_corr))
    #print('diff_corr.argmax()',diff_corr.argmax())
    deltaT=diff_corr.argmax()
    print('deltaT :', deltaT / freqEch )
    print('cosangle :', (deltaT / freqEch * vSonEau) / d)
    if ((deltaT / freqEch * vSonEau) / d)<=1 and ((deltaT / freqEch * vSonEau) / d)>-1:
        print('angle en deg :', math.acos((deltaT / freqEch * vSonEau) / d) * 180 / np.pi)
    else:
        print('domain exception : cos(angle) > 1 ou <-1')
    #print('length:',len(AI_1))


if __name__=='__main__':
    distanceHydro=0.025; #distance entre les 2 hydrophones
    vSonEau=1500;
    #freqEch=96000.0;
    freqEch=44100.0
    t=0;
    #while t<5:   #TODO boucle se fait 5 fois : coder pour le faire tant que l'utilisateur n'appuie pas sur echap
        #filename = time.strftime("%d%b%Y_%H%M%S.wav") #titre du fichier = date
        #recPing(filename)                             #enregistrement
        #calculAngle(filename, distanceHydro, vSonEau, freqEch)  #calcul angle
    calculAngle('ping_3m.wav', distanceHydro, vSonEau, freqEch)       #test calcul angle pour un fichier preenregistre avant
    #    t=t+1 #iteration boucle


# cos(distHyrophone/distanceTrouvee)

# Affichage des plot
pylab.show()