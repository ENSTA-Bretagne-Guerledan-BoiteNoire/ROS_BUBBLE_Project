import scipy.io.wavfile as sciwave
from scipy.fftpack import fft
import math
import numpy as np
import RecPing
import time
import pylab
import soundProcessing as sp


def process():

    # ========= (1) ENREGISTREMENT ==========
    #filename = "toto.wav"
    filename = time.strftime("%d%b%Y_%H%M%S.wav")  # titre du fichier = date
    RecPing.recPing(filename)  # enregistrement

    # ========= (2) RECEIVED SIGNAL =========

    Fs, a = sciwave.read(filename)
    #a = np.array(a)
    a = a*10
    a_l= sp.butter_bandpass_filter(a[:,0], 4000, 6000, Fs, order=5)
    a_r= sp.butter_bandpass_filter(a[:,1], 4000, 6000, Fs, order=5)
    a = np.array([a_l[1000:],a_r[1000:]])
    a = a.transpose()
    a = a / (2.0**15 - 1.0)     # Normalisation des donnees (valeurs sur 16 bits)
    
    ####### Parametres modifiables ###

    M = 1;                       # Number of sources
    N = 2;                       # Number of microphones(on doit avoir M <= N - 1)
    dist = .10;                  # Distance between adjacent microphones(en m)
    #c = 346.287;               # Speed of sound in air(en m/s)
    c = 1500;                    # Speed of sound in water
    f = 5000;                   # Signal frequency(en Hz)

    lfft = 1024 * 1;             # Number of data points for FFT in a snapshot
    K = math.floor(np.size(a[:,0]) / lfft); # Number of frequency snapshots(multiple of lfft)
    L = int(K * lfft);                # Number of data snapshots recorded by receiver
    y = a[0:L,:];                # Truncated signal(contient nbre entier de snapshots)

    T = 1 / Fs;
    rx=np.linspace(-(N-1)/2.0,(N-1)/2.0,N).reshape(N,1); # Assume uniform linear array (on peut avoir elevation si on fait array rectangulaire avec min 3 hydrophones)
    ry=np.zeros((N,1),float);
    rz=np.zeros((N,1),float);
    r = np.concatenate((rx,ry,rz),axis=1); # Assume uniform linear array

    ## Forme complexe du signal recu
    df = float(Fs)/float(lfft)/1.0;                 # frequency grid size
    F = np.arange(0,Fs/1,df);
    X=np.zeros((int(lfft),int(N),int(K)),dtype='complex');
    for ih in range (0,int(N)):
        for iv in range (0,int(K)):
            pos=iv*lfft;
            tmp=y[pos:pos+lfft,ih];
            X[:,ih,iv]=fft(tmp);



    # Find the closest frequency from the discrete frequency vector resulted from FFT
    values=abs(F-f);
    mf=np.amin(values);
    mi=np.argmin(values)
    f0=F[mi];

    x0=np.zeros((int(N), int(K)),dtype='complex');
    for ih in range (0,int(N)):
        for iv in range (0,int(K)):
            x0[ih,iv]=X[mi,ih,iv]; # signal complexe

    # Sample covariance matrix
    Rxx = np.dot(x0,x0.conj().T)/L;

    # Search directions
    AzSearch = np.linspace(0,180,181); # Azimuth values to search
    ElSearch = np.zeros(AzSearch.shape); # Simple 1D example


    # Corresponding points on array manifold to search
    kSearch = math.pi*np.array([np.cos(np.radians(AzSearch))*np.cos(np.radians(ElSearch)), np.sin(np.radians(AzSearch))*np.cos(np.radians(ElSearch)), np.sin(np.radians(ElSearch))]);
    ASearch = np.exp(-1j*np.dot(r,kSearch));

    #################################################################################################################
    ## Capon

    # Capon spectrum
    Zc=[]
    for i in range (0,np.size(AzSearch)):
        Z=(ASearch[:,i].conj().T).dot(np.linalg.inv(Rxx)).dot(ASearch[:,i]);
        Zc.append(abs(1/Z));
    Zc = 10*np.log10(Zc/max(Zc))

    ##################################################################################################################
    ## MUSIC

    # Eigendecompose
    D,E = np.linalg.eig(Rxx);

    lambd = np.sort(D); # Vector of sorted eigenvalues
    idx = np.argsort(D); # Index of sorted eigenvalues


    E = E[:,idx]; # Sort eigenvalues accordingly
    En = E[:,0:np.size(idx)-M]; # Noise eigenvectors (ASSUMPTION: M IS KNOWN)


    # MUSIC spectrum
    Zm=[]
    for i in range (0,np.size(AzSearch)):
        Z = (ASearch[:, i].conj().T).dot(En).dot(En.conj().T).dot(ASearch[:, i]);
        Zm.append(abs(1/Z));

    Zm = 10*np.log10(Zm/max(Zm))


    # Angle calculation
    Zcmin=np.amin(Zc);
    Zcmax=np.amax(Zc);
    Zmi=np.argmax(Zm);
    #Zmmin=np.amin(Zm)
    #if (abs(Zmmin-Zcmin)<2):
       
    # Amplitude detection Capon pour savoir si on a une fausse detection
    if (abs(Zcmax-Zcmin)<5):
        angle=-180
        print("Source not detected")
    # Position pic MUSIC pour determiner localisation source detectee
    else:
        angle = AzSearch[Zmi];
        print("Angle  :", angle )

    # Plot spectrum
    #pylab.figure()
    #pylab.plot(AzSearch,Zc)
    #pylab.plot(AzSearch,Zm)
    #pylab.show()

    return angle


if __name__=="__main__":
    process()
