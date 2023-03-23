import serial
import time
from time import sleep

USB_PORT = "COM9"
GRBL_BPS = 115200
SERIAL_CONNECTION = serial.Serial(USB_PORT, GRBL_BPS, timeout=1)

# State, these are indexed when looking from back of hubbot
is_batt_slot_populated = [False, False, True]
batt_slot_abs_loc_cm = [0, 4.75, 9.75]

# GCode commands
cm_mode = "G21 "
inch_mode = "G20 "
absolute_mode = "G90 "
relative_mode = "G91 "

# GRBL commands
grbl_home = "$H"
grbl_jog = "$J="  # eg: $J=G91 G21 X0.5 F10

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

def send_grbl_gcode_cmd_bytes(cmd: bytes):
  """Send a gcode command to the grbl controller"""
  # Send g-code block
  SERIAL_CONNECTION.write(cmd + str.encode('\n'))
  # Wait for response with carriage return
  grbl_out = SERIAL_CONNECTION.readline()
  print("Sent command: " + str(cmd))
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

def home_z_lift_arms():
  """Home the z axis"""
  print("Homing Z axis")
  # send_grbl_gcode_cmd("$J=G91 G21 Z-304.8 F2000")
  # send_grbl_gcode_cmd("G91 G21 G1 Z30.48 F2000")
  send_grbl_gcode_cmd("G91 G21 G1 Z-304.8 F2000")

def home_x_batt_mover():
  """Home the x axis"""
  print("Homing X axis")
  send_grbl_gcode_cmd("G91 G21 G1 X-5 F2000")
  send_grbl_gcode_cmd("G91 G21 G1 X22 F2000")

# Battery indexer
def home_y_batt_indexer():
  """Home the y axis"""
  print("Homing Y axis")
  send_grbl_gcode_cmd("G91 G21 G1 Y17 F2000")
  send_grbl_gcode_cmd("G91 G21 G1 Y-17 F2000")
  # Slower derease speed so that we increase torque in case we have tension in belts near end.
  send_grbl_gcode_cmd("G91 G21 G1 Y-1 F10")

def ESTOP():
  """Emergency stop. Kills all motion controlled by GRBL"""
  print("Emergency stop!!")
  # send_grbl_gcode_cmd("!")
  # send_grbl_gcode_cmd("0x18")
  send_grbl_gcode_cmd_bytes(b"\x18")
  SERIAL_CONNECTION.flushInput()

def unESTOP():
  send_grbl_gcode_cmd("$H")
  send_grbl_gcode_cmd("$X")

# Lift minibot to ensure alignment
def lift_minibot_check_alignment():
  """Lift minibot to ensure alignment"""
  print("Lifting minibot to check alignment")
  send_grbl_gcode_cmd("G91 G21 G1 Z-304.8 F2000")

# Once minibot is aligned, swap battery
def swap_battery():
  """Swap battery"""
  print("Swapping battery...")
  # First move batt pusher to above minibot
  # It should already be there from homing

  # Find first empty batt slow and move indexer to empty slot
  # empty_slot = -1
  # for i in range(0, len(is_batt_slot_populated)):
  #   if not is_batt_slot_populated:
  #     empty_slot = i
  #     break
  # if empty_slot == -1:
  #   print("No empty battery slots!")
  #   exit(1)
  
  # Move indexer to empty slot
  send_grbl_gcode_cmd("G90 G21 G1 Y4.75 F100")

  # Lift minibot up
  send_grbl_gcode_cmd("G91 G21 G1 Z304.8 F2000")
  time.sleep(5)
  # send_grbl_gcode_cmd("G90 G21 G1 Y10 F100")
  # send_grbl_gcode_cmd("G90 G21 G1 Y22 F10")
  # send_grbl_gcode_cmd("G90 G21 G1 Y10 F10")

  
def move_lift_arms_to_minibot():
  """Move lift arms down to minibot"""
  print("Moving lift arms to minibot...")
  send_grbl_gcode_cmd("G90 G21 G1 Z27 F2000")

if __name__ == "__main__":
  connect()
  while(1):
    cmd = input(">")
    if cmd == "exit":
      exit(0)
    elif cmd == "homez":
      home_z_lift_arms()
    elif cmd == "homex":
      home_x_batt_mover()
    elif cmd == "homey":
      home_y_batt_indexer()
    elif cmd == "sethome":
      send_grbl_gcode_cmd("G28.1")
    elif cmd == "swap":
      swap_battery()
    elif cmd == "liftdown":
      # move_lift_arms_to_minibot()
      send_grbl_gcode_cmd("G91 G21 G1 Z200 F2000")
    elif cmd == "liftup":
      send_grbl_gcode_cmd("G91 G21 G1 Z-200 F400")
    elif cmd == "s":
      ESTOP()
    elif cmd == "r":
      unESTOP()
    elif cmd == "slot1":
      send_grbl_gcode_cmd("G90 G21 G1 Y0 F100")
    elif cmd == "slot2":
      send_grbl_gcode_cmd("G90 G21 G1 Y4.75 F100")
    elif cmd == "slot3":
      send_grbl_gcode_cmd("G90 G21 G1 Y9.75 F100")
    elif cmd == "pushin":
      send_grbl_gcode_cmd("G90 G21 G1 X-11 F200")
    elif cmd == "pushout":
      send_grbl_gcode_cmd("G90 G21 G1 X0 F200")
    elif cmd == "repl":
      repl()
    else:
      print("Unknown command")
  
  # send_grbl_gcode_cmd("$J=G91 G20 X0.5 F10")
  # home_z_lift_arms()
  # home_x_batt_mover()
  # home_y_batt_indexer()
  # time.sleep(2)
  # move_lift_arms_to_minibot()

  # swap_battery()
  
