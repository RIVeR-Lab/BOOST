import serial
import time

USB_PORT = "COM9"
GRBL_BPS = 115200
SERIAL_CONNECTION = serial.Serial(USB_PORT, GRBL_BPS, timeout=1)

# GCode commands
mm_mode = "G21"
inch_mode = "G20"
absolute_mode = "G90"
relative_mode = "G91"

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
  print(grbl_out.strip().decode("utf-8"))
  print("Sent command")

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

connect()
send_grbl_gcode_cmd("$J=G91 G20 X0.5 F10")