import numpy as np
import matplotlib.pyplot as plt
import sys
from numpy.fft import rfft

# plot rms vs time

def calc_rms(accel_values, filter_size):
    filter = np.zeros(filter_size)
    filter_ind = 0

    rms_out = np.array([])

    for i, value in enumerate(accel_values):
        
        filter[filter_ind] = value # 0, {0,1}, {0, 1, 2}, {0, 1..., 8}
        filter_ind = (filter_ind + 1) % filter_size

        # filter = accel_values[i: (i+filter_size)]
        rms_out =  np.append(rms_out, np.sqrt(np.mean(np.square(filter))))

    return rms_out

if __name__ == '__main__':
    filename = sys.argv[1]

    with open(filename, 'r') as f:
        lines = f.readlines()
        lines = [float(line) -1.0 for line in lines]


    sample_rate = 200 # Hz
    t = np.arange(len(lines)*(1.0/sample_rate), step=1.0/sample_rate)

    rms4 = calc_rms(lines, 4)
    rms8 = calc_rms(lines, 8)
    rms64 = calc_rms(lines, 64)
    rms256 = calc_rms(lines, 256)
    rmsAll = calc_rms(lines, len(lines))

    X = rfft(rms8)
    X[0] = 0
    X[1] = 0
    freqHz = np.fft.fftfreq(len(rms8)) * sample_rate
    freqHz = np.append(0, freqHz)
    plt.plot(freqHz[:len(freqHz)//2], np.abs(X))
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")


    # plt.plot( lines, label="raw acceleration")
    # plt.plot( rms4, label="rms4")
    # plt.plot( rms8,   label="rms8")
    # plt.plot( rms64,  label="rms64")
    # plt.plot( rms256, label="rms256")
    # plt.plot( rmsAll, label="rmsAll")

    plt.legend()

    plt.savefig(sys.argv[2])
    