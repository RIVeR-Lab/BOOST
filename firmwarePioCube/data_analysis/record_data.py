import serial
import csv

serial_port = "COM12"
baud = 57600
fileName="analog-data.csv"
data_labels_to_read = ['>AcclX(m/s^2)', '>AcclY(m/s^2)', '>AcclZ(m/s^2)']

# Open port and csv
ser = serial.Serial(serial_port, baud)
print("Connected to STM32 port:" + serial_port)
file = open(fileName, "a")
print("Created csv file")

# Read one set of data samples
def read_one_sample(data_labels):
  read_one_set = False
  # Set of data labels we want to read in one data sampling
  data_sample = [0] * len(data_labels)
  # Indicates which specific sample have been read and which still need to be read.
  data_sample_read = [0] * len(data_labels)
  while sum(data_sample_read) != len(data_sample_read):
    getData=ser.readline()
    dataString = getData.decode('utf-8')
    data=dataString[0:][:-2]
    reading = data.split(":")
    try:
      i = data_labels.index(reading[0])
    # if i != -1:
      data_sample[i] = reading[1]
      data_sample_read[i] = 1
    except:
      nop = 1
  return data_sample

FS = 100.0 # sample freq Hz
TS = 1.0 / FS # sample period
time_to_collect_sec = 5.0 # 5*60
n_samples = time_to_collect_sec / TS # how many samples to collect
print_labels = False
line = 0 #start at 0 because our header is 0 (not real data)
sensor_data = [] #store data

print(n_samples)

# collect the raw data
while line <= n_samples:
  sample = read_one_sample(data_labels_to_read)
  print(sample)
  sensor_data.append(sample)
  line = line+1

print(sensor_data)

  