import serial
import csv
import time
import numpy as np
import pandas as pd

serial_port = "COM12"
baud = 57600
fileName="analog-data.csv"
data_labels_to_read = ['>time(ms)', '>AcclX(m/s^2)', '>AcclY(m/s^2)', '>AcclZ(m/s^2)']

# Open port and csv
ser = serial.Serial(serial_port, baud)
print("Connected to STM32 port:" + serial_port)

FS = 100.0 # sample freq Hz
TS = 1.0 / FS # sample period
time_to_collect_sec = 5.0 # 5*60
n_samples = time_to_collect_sec / TS # how many samples to collect
print_labels = False
line = 0 #start at 0 because our header is 0 (not real data)
raw_serial_lines = []

# collect the raw data
start_time = time.time()
while time.time() - start_time < time_to_collect_sec:
  getData=ser.readline()
  dataString = getData.decode('utf-8')
  data=dataString[0:][:-2]
  reading = data.split(":")
  raw_serial_lines.append(reading)
  print(reading)

# Read one set of data samples
def parse_data(data_labels, data):
  lineN = 0
  sensor_data = []
  # Set of data labels we want to read in one data sampling
  data_sample = [0] * len(data_labels)  
  # Indicates which specific sample have been read and which still need to be read.
  data_sample_read = [0] * len(data_labels)

  while lineN < len(data):
    # Check if we have collecting 1 whole sample yet
    if sum(data_sample_read) == len(data_sample_read):
      sensor_data.append(data_sample)
      data_sample_read = [0] * len(data_labels)
      data_sample = [0] * len(data_labels)  

    # Parse next line
    line = data[lineN]
    try:
      i = data_labels.index(line[0])
      data_sample[i] = line[1]
      data_sample_read[i] = 1
    except:
      nop = 1
    lineN = lineN + 1
  print(sensor_data)
  return sensor_data

# Parse the data
parsed_sensor_data = parse_data(data_labels_to_read, raw_serial_lines)
parsed_sensor_data_np = np.array(parsed_sensor_data)
print(parsed_sensor_data)
print("Number of samples collected: " + str(len(parsed_sensor_data)))

# Write to csv
DF = pd.DataFrame(parsed_sensor_data)
with open(fileName, "w") as file:
  DF.to_csv(file)