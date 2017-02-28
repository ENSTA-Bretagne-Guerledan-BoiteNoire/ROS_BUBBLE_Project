import wave
import math
import numpy as np
import pyaudio
import sys
import time
import pylab
from scipy.signal import butter, lfilter
import matplotlib.pyplot as plt
from scipy import fft, arange, ifft
from scipy.io import wavfile


global TheoricPing
TheoricPing = np.zeros(44032)
pingLen = 0.015
fEch = 44100
pingFreq = 15000
nEch = int(pingLen*fEch)

for i in range(nEch):
        TheoricPing[i] = int(128.0 + 127.5*math.sin(2.0*math.pi*pingFreq*i/fEch))



def genPing():      #genere un fichier .wav de 1 seconde, commencant par 15ms de 15khz puis silence.
    filename = "ping.wav"
    wav = wave.open(filename, 'w')

    nChanel = 1 #1 = Mono, 2= Stereo
    nOctet = 1
    fEch = 44100

    pingFreq = 15000
    level = 1
    pingLen = 0.015
    pingRate = 0.5

    nEch = int(pingLen*fEch)
    nZeros = int(pingRate*fEch)-nEch

    params = (nChanel, nOctet, fEch, nEch+nZeros, 'NONE', 'not compressed')
    wav.setparams(params)

    amp = 127.5*level

    for i in range(nEch):
        val = wave.struct.pack('B', int(128.0 + amp*math.sin(2.0*math.pi*pingFreq*i/fEch)))
        wav.writeframes(val)

    for i in range(nZeros):
        val = wave.struct.pack('B', 128)
        wav.writeframes(val)
    wav.close()

def PingToTxt():  #transforme le fichier "son.wav" en fichier .txt contenant les niveaux des different canaux
    wav = wave.open("son.wav", 'r')
    test = open("datas.txt", 'w')
    n = wav.getnframes()
    for i in range(n):
        val = wav.readframes(1)
        test.write(str(wave.struct.unpack('B', val[1])[0])+ str(wave.struct.unpack('B', val[0])[0]) + " " + str(wave.struct.unpack('B', val[3])[0])+ str(wave.struct.unpack('B', val[2])[0]) + "\n")
    wav.close()
    test.close()

def PingToNpArray(file_input):  #transforme un fichier "son.wav" en tableau numpy[canal][echantillon][caractere](2 cannaux, 2 caracteres)
    wav = wave.open(file_input, 'r')
    n = wav.getnframes()
    PingTab = np.zeros([2, n, 2])
    for i in range(n):
        val = wav.readframes(1)
        PingTab[0][i][0] = int(str(wave.struct.unpack('B', val[1])[0]))
        PingTab[0][i][1] = int(str(wave.struct.unpack('B', val[0])[0]))
        PingTab[1][i][0] = int(str(wave.struct.unpack('B', val[3])[0]))
        PingTab[1][i][1] = int(str(wave.struct.unpack('B', val[2])[0]))
    wav.close()
    return PingTab

def NpArrayToWAV(nparray, wavefile): #transforme un tableau numpy[canal][echantillon][carractere] en fichier .wav
    #genere un fichier .wav de 1 seconde, commencant par 15ms de 15khz puis silence.
    wav = wave.open(wavefile, 'w')

    nChanel = 2 #1 = Mono, 2= Stereo
    nOctet = 1
    fEch = 44100

    params = (nChanel, nOctet, fEch, len(nparray[0]), 'NONE', 'not compressed')
    wav.setparams(params)

    for i in range(len(nparray[0])):
        lval = wave.struct.pack('B', nparray[0][i][0]) + wave.struct.pack('B', nparray[0][i][1])
        rval = wave.struct.pack('B', nparray[1][i][0]) + wave.struct.pack('B', nparray[1][i][1])
        wav.writeframes(lval + rval)
    wav.close()

def getCorrel(): #renvoi la fonction d'intercorelation (np.array) entre le ping et le canal droit, le ping et le canal gauche, et l'autocorellation du ping
    global TheoricPing
    hydroArray = PingToNpArray()
    pingArray = TheoricPing

    t0 = time.time()
    corel1 = np.correlate(pingArray, hydroArray[0], "full")
    t1 = time.time()
    etime = t1-t0
    print "execution Time :", etime

    corel2 = np.correlate(pingArray, hydroArray[1], "full")
    corel12 = np.correlate(hydroArray[0], hydroArray[1], "full")
    corelPing = np.correlate(pingArray, pingArray, "full")
    return corel1, corel2, corel12, corelPing

def play1Ping(): #emet un ping (le fichier ping.wav doit etre moins long qu'une seconde)
    CHUNK = 1024

    wf = wave.open("ping.wav", 'rb')

    p = pyaudio.PyAudio()

    stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)

    data = wf.readframes(CHUNK)

    while data != '':
        stream.write(data)
        data = wf.readframes(CHUNK)

    stream.stop_stream()
    stream.close()

    p.terminate()

def recPing(WAVE_OUTPUT_FILENAME = "son.wav"): #enregistre sous le nom "son.wav" 1 seconde de l'entree micro
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 2
    RATE = 44100
    RECORD_SECONDS = 1

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


def amp():  #amplifie un fichier texte contenant les donnees audio (sous forme d'entiers) et ecrit un fichier "son2.wav"
    PingToTxt()
    filename = "son2.wav"
    wav = wave.open(filename, 'w')
    datas = open("datas.txt", 'r')
    nChanel = 2 #1 = Mono, 2= Stereo
    nOctet = 1
    fEch = 44100

    pingFreq = 15000
    level = 1
    pingLen = 0.015
    pingRate = 1

    nEch = int(pingLen*fEch)
    nZeros = int(pingRate*fEch)-nEch

    params = (nChanel, nOctet, fEch, nEch+nZeros, 'NONE', 'not compressed')
    wav.setparams(params)

    amp = 127.5*level

    try:
        while 1:
            v = datas.readline()
            v = v.split()
            v1 = (int(v[0])-128)*2
            v2 = (int(v[1])-128)*2
            v1 = min(max(-127, v1), 127)
            v2 = min(max(-127, v2), 127)
            val1 = wave.struct.pack('B', int(128.0 + v1))
            val2 = wave.struct.pack('B', int(128.0 + v2))
            wav.writeframes(val1 + val2)

    except IndexError:
        datas.close()
        wav.close()

def AmpNpArray(nparray):  #amplifie un son sous forme de tableu numpy[canal][echantillon][caractere] et ecrit un fichier "amplifiedSound.wav"
    resArray = np.multiply(nparray, 2)
    resArray = np.minimum(np.maximum(-127, resArray), 127)
    resArray = np.add(resArray, 128)
    NpArrayToWAV(resArray, "amplifiedSound.wav")

def playPing():   #emet en continu un ping toute les secondes (precis)
    genPing()
    while 1:
        t0 = time.time()
        play1Ping()
        t1 = time.time()
        t = t1-t0
        sleepTime = max(1-t, 0)
        time.sleep(sleepTime)

#Plot un fichier wav, signalName sous la forme 'name.wav'
def plotWAV(signalName):

    fs, data = wavfile.read(signalName)  # load the data
    print fs
    l = data.T[0]  # this is a two channel soundtrack, I get the first track
    r = data.T[1]

    plt.figure(1)
    plt.subplot(211)
    plt.plot(l, 'r')
    plt.subplot(212)
    plt.plot(r,'b')

    n = len(l)  # lungime semnal
    k = arange(n)
    T = n / fs
    frq = k / T  # two sides frequency range
    frq = frq[range(n / 2)]  # one side frequency range

    L = fft(l) / n  # fft computing and normalization
    L = L[range(n / 2)]
    #L = fft(l)  # liste de complexes. type de c: class 'numpy.complex128# '
    R = fft(r)  # liste de complexes. type de c: class 'numpy.complex128

    R = fft(r) / n  # fft computing and normalization
    R = R[range(n / 2)]

    #d = len(L) // 2  # on n'a besoin que de la moitie de la liste fft

    plt.figure(2)
    plt.subplot(211)
    plt.plot(frq,abs(L), 'r')  # affichage
    pylab.ylim([0, 0.3])
    plt.subplot(212)
    plt.plot(abs(R), 'b')  # affichage
    pylab.ylim([0, 0.3])
    plt.show()

if __name__ == "__main__":
    playPing()
    b, a =butter_bandpass(14500, 15500, fs, order=5)

