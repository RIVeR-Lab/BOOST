import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

DATA_FILE = 'teleplot_2023-2-8_13-20-accel-5min.csv'
FS = 100  # Sample frequency [Hz]
MEAS_DUR_SEC = 5*60  # (6 hrs) Seconds to record data for
TS = 1.0 / FS  # Sample period [s]

# Load into arrays, convert units
dataArr = np.genfromtxt(DATA_FILE, delimiter=',', usecols=)
ax = dataArr[:, 14] * 9.80665  # m/s/s
ay = dataArr[:, 15] * 9.80665
az = dataArr[:, 16] * 9.80665
# gx = dataArr[:, 3] * (180 / np.pi)  # deg/s
# gy = dataArr[:, 4] * (180 / np.pi)
# gz = dataArr[:, 5] * (180 / np.pi)

N = len(ax)  # Number of elements

# Compute FFTs
freqBins = np.linspace(0, FS / 2, N // 2)  # Freq. labels [Hz]
fax = np.fft.fft(ax)  # FFT of accel. data
# fgx = np.fft.fft(gx)  # FFT of gyro data

# Plot x-accel. FFT
plt.figure()
plt.plot(freqBins, (2 / N) * np.abs(fax[:N // 2]))
plt.title('FFT of X-Accelerometer Data')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude')
plt.grid()

# Plot x-gyro. FFT
plt.figure()
plt.plot(freqBins, (2 / N) * np.abs(fgx[:N // 2]))
plt.title('FFT of X-Gyroscope Data')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude')
plt.grid()
plt.show()