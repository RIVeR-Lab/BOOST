import serial
import time

USB_PORT = "COM9"
GRBL_BPS = 115200
SERIAL_CONNECTION = serial.Serial(USB_PORT, GRBL_BPS, timeout=1)

# GCode commands
mm_mode = "G21 "
inch_mode = "G20 "
absolute_mode = "G90 "
relative_mode = "G91 "

def connect():
  """Connect to the grbl controller"""
  print("Connecting to GRBL...")
  # Wake up grbl
  SERIAL_CONNECTION.write(str.encode("\r\n\r\n"))
  # Wait for grbl to initialize and flush startup text in serial input
  time.sleep(2)
  SERIAL_CONNECTION.flushInput()
  print("Connected to GRBL")

def send_grbl_gcode_cmd(cmd: str):
  """Send a gcode command to the grbl controller"""
  # Send g-code block
  SERIAL_CONNECTION.write(cmd.encode() + str.encode('\n'))
  # Wait for response with carriage return
  grbl_out = SERIAL_CONNECTION.readline()
  print("Sent command: " + cmd)
  print("grbl response: " + grbl_out.strip().decode("utf-8"))
  

def repl():
  while(1):  
    # Get input
    val = input(">")
    print(val)
    # Send input
    ser.write(bytes("$J=G91 G20 X0.5 F10", 'utf-8'))
    # Get output
    while(line != ""):
      line = ser.readline().decode('utf-8')   # read a '\n' terminated line
      print(line)

def home_z():
  """Home the z axis"""
  print("Homing Z axis")
  send_grbl_gcode_cmd("$J=G91 G21 Z-304.8 F2000")

def home_x():
  """Home the x axis"""
  print("Homing X axis")
  send_grbl_gcode_cmd("$J=G91 G21 X-20 F2000")

def home_y():
  """Home the y axis"""
  print("Homing Y axis")
  send_grbl_gcode_cmd("$J=G91 G21 Y-20 F2000")
  send_grbl_gcode_cmd("$J=G91 G21 Y-5 100")

def ESTOP():
  """Emergency stop. Kills all motion controlled by GRBL"""
  print("Emergency stop!!")
  send_grbl_gcode_cmd("!")


connect()
# send_grbl_gcode_cmd("$J=G91 G20 X0.5 F10")
# home_z()
# home_x()
home_y()
time.sleep(2)
# ESTOP()
