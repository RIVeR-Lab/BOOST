import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
from datetime import datetime
import os

DATA_FILE = os.path.dirname(__file__) + "/20230308161053-analog-data.csv"
FS = 100  # Sample frequency [Hz]
MEAS_DUR_SEC = 5*60  # (6 hrs) Seconds to record data for
TS = 1.0 / FS  # Sample period [s]

# Load into arrays, convert units
dataArr = np.genfromtxt(DATA_FILE, delimiter=',')
dataArr = dataArr[1:]  # Remove header row
time = dataArr[:, 0]  # ms
ax = dataArr[:, 2]
ay = dataArr[:, 3]
az = dataArr[:, 4]
# remove the first 10 rows
time = time[10:]
ax = ax[10:]
ay = ay[10:]
az = az[10:]
# gx = dataArr[:, 3] * (180 / np.pi)  # deg/s
# gy = dataArr[:, 4] * (180 / np.pi)
# gz = dataArr[:, 5] * (180 / np.pi)

N = len(ax)  # Number of elements

print(dataArr)

# Plot x-accel. time series
plt.figure()
plt.plot(time, ax, label='ax')
plt.plot(time, ay, label='ay')
plt.plot(time, az, label='az')
plt.title('Raw Accel Data')
plt.xlabel('Time (ms)')
plt.ylabel('Acceleration (m/s^2)')
plt.legend()
plt.grid()
# plt.show()

# Compute FFTs
freqBins = np.linspace(0, FS / 2, N // 2)  # Freq. labels [Hz]
fax = np.fft.fft(ax)
fay = np.fft.fft(ay)
faz = np.fft.fft(az)
# fgx = np.fft.fft(gx)  # FFT of gyro data

# Plot x-accel. FFT
plt.figure()
plt.plot(freqBins, (2 / N) * np.abs(fax[:N // 2]), label='ax')
plt.plot(freqBins, (2 / N) * np.abs(fay[:N // 2]), label='ay')
plt.plot(freqBins, (2 / N) * np.abs(faz[:N // 2]), label='az')
plt.title('FFT of Accel Data')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude')
plt.legend()
plt.grid()

# Plot x-gyro. FFT
# plt.figure()
# plt.plot(freqBins, (2 / N) * np.abs(fgx[:N // 2]))
# plt.title('FFT of X-Gyroscope Data')
# plt.xlabel('Frequency [Hz]')
# plt.ylabel('Amplitude')
# plt.grid()
# plt.show()


########## Compute Power Spectral Density (PSD) ##########

# Conversion factor: m/s/s -> ug (micro G's)
accel2ug = 1e6 / 9.80665

# Compute PSD via Welch algorithm
freqax, psdax = signal.welch(ax, FS, nperseg=1024, scaling='density')  # ax
freqay, psday = signal.welch(ay, FS, nperseg=1024, scaling='density')  # ay
freqaz, psdaz = signal.welch(az, FS, nperseg=1024, scaling='density')  # az

# freqgx, psdgx = signal.welch(gx, FS, nperseg=1024, scaling='density')  # gx
# freqgy, psdgy = signal.welch(gy, FS, nperseg=1024, scaling='density')  # gy
# freqgz, psdgz = signal.welch(gz, FS, nperseg=1024, scaling='density')  # gz

# Convert to [ug / sqrt(Hz)]
psdax = np.sqrt(psdax) * accel2ug
psday = np.sqrt(psday) * accel2ug
psdaz = np.sqrt(psdaz) * accel2ug / 9.80665

# psdgx = np.sqrt(psdgx)
# psdgy = np.sqrt(psdgy)
# psdgz = np.sqrt(psdgz)

# Compute noise spectral densities
ndax = np.mean(psdax)
nday = np.mean(psday)
ndaz = np.mean(psdaz)
print('AX Noise Density: %f ug/sqrt(Hz)' % (ndax))
print('AY Noise Density: %f ug/sqrt(Hz)' % (nday))
print('AZ Noise Density: %f ug/sqrt(Hz)' % (ndaz))

# ndgx = np.mean(psdgx)
# ndgy = np.mean(psdgy)
# ndgz = np.mean(psdgz)
# print('GX Noise Density: %f dps/sqrt(Hz)' % (ndgx))
# print('GY Noise Density: %f dps/sqrt(Hz)' % (ndgy))
# print('GZ Noise Density: %f dps/sqrt(Hz)' % (ndgz))


# Plot accel. data
plt.figure()
plt.plot(freqax, psdax, label='ax')
plt.plot(freqay, psday, label='ay')
plt.plot(freqaz, psdaz, label='az')
plt.title('Accelerometer Power Spectral Density')
plt.xlabel('Frequency [Hz]')
plt.ylabel(r'Spectral Density  $\mu g / \sqrt{Hz}$')
plt.legend()
plt.grid()

# Plot gyro data
# plt.figure()
# plt.plot(freqgx, psdgx, label='gx')
# plt.plot(freqgy, psdgy, label='gy')
# plt.plot(freqgz, psdgz, label='gz')
# plt.title('Gyro Noise Spectral Density')
# plt.xlabel('Frequency [Hz]')
# plt.ylabel(r'Spectral Density  $dps / \sqrt{Hz}$')
# plt.legend()
# plt.grid()
plt.show()