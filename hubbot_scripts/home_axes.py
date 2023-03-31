import serial
import time
from time import sleep

'''
Resources
GRBL Error Codes: https://docs.sainsmart.com/article/dm4jbd5jce-grbl-error-codes
GRBL Commands: https://github.com/gnea/grbl/blob/master/doc/markdown/commands.md
GRBL Settings: https://github.com/gnea/grbl/blob/master/doc/markdown/settings.md
GRBL Wiki: https://github.com/grbl/grbl/wiki
GCODE Commands: https://howtomechatronics.com/tutorials/g-code-explained-list-of-most-important-g-code-commands/#G01_Linear_Interpolation
'''
from serial.tools import list_ports

grbl_port_VID = 6790
grbl_port_PID = 29987
USB_PORT = ""
GRBL_BPS = 115200
SERIAL_CONNECTION = None

# State, these are indexed when looking from back of hubbot
is_batt_slot_populated = {"0": False, "1": True, "2": True}
is_homed_dict = {"x": False, "y": False, "z": False}

# Absolute location of things all relative to the homed position
indexer_abs_loc_cm = {"slot0": 0, "slot1": 4.75, "slot2": 9.75}
pusher_abs_loc_cm = {"above_minibot": -0.5, "in_indexer": -11}
'''
liftdown: Where the minibot can dock
liftup: Where the minibot is lifted to and where battery can be swapped at
'''
lift_abs_loc_cm = {"liftdown": 0, "liftup": -105}

# Feed rates
indexer_feed_rate = "F200"
pusher_feed_rate = "F200"
lift_feed_rate = "F1000"

# Axis Invert Direction
x_axis_invert = 0   # pusher
y_axis_invert = 1   # indexer
z_axis_invert = 0

# GCode commands
cm_mode = "G21"
inch_mode = "G20"
absolute_mode = "G90"
relative_mode = "G91"
G01_linear_interpolation = "G01"
indexer_axis = "Y"
pusher_axis = "X"
lift_axis = "Z"

# GRBL commands
# https://github.com/gnea/grbl/blob/master/doc/markdown/commands.md
grbl_home_cmd = "$H"
grbl_jog_cmd = "$J="  # eg: $J=G91 G21 X0.5 F10
grbl_run_homing_cycle_cmd = "$H"
grbl_status_report_cmd = "?"

# GRBL Settings
grbl_enable_homing_setting_cmd = "$22=1"
grbl_grbl_disable_homing_setting_cmd = "$22=0"
grbl_enable_soft_limits_setting_cmd = "$20=1"
grbl_disable_soft_limits_setting_cmd = "$20=0"
grbl_status_report_buffer_data_cmd = "$10=2"
grbl_dir_port_invert_mask = "$3="


# GRBL RealTime commands
grbl_soft_reset_cmd = b"\x18"
grbl_feed_hold_cmd = "!"

def get_com_port():
    global USB_PORT
    device_list = list_ports.comports()
    for device in device_list:
        print(device)
        print(device.vid)
        print(device.pid)
        if (device.vid != None or device.pid != None):
            if (device.vid == grbl_port_VID and
                device.pid == grbl_port_PID):
                port = device.device
                print("GRBL found on port:" + port)
                break
            port = None
    USB_PORT = port
            
def connect():
    global SERIAL_CONNECTION
    """Connect to the grbl controller"""
    print("Connecting to GRBL...")
    SERIAL_CONNECTION = serial.Serial(USB_PORT, GRBL_BPS, timeout=1)
    # Wake up grbl
    SERIAL_CONNECTION.write(str.encode("\r\n\r\n"))
    # Wait for grbl to initialize and flush startup text in serial input
    time.sleep(2)
    SERIAL_CONNECTION.flushInput()
    print("Connected to GRBL")

def send_grbl_gcode_cmd(cmd: str, wait_for_response: bool = True):
    """Send a gcode command to the grbl controller"""
    # Send g-code block
    SERIAL_CONNECTION.write(cmd.encode() + str.encode('\n'))
    if wait_for_response:
        # Wait for response with carriage return
        grbl_out = SERIAL_CONNECTION.readline()
        print("grbl response: " + grbl_out.strip().decode("utf-8"))
    print("Sent command: " + str(cmd))


def send_grbl_gcode_cmd_bytes(cmd: bytes, wait_for_response: bool = True):
    """Send a gcode command to the grbl controller"""
    # Send g-code block
    SERIAL_CONNECTION.write(cmd + str.encode('\n'))
    if wait_for_response:
        # Wait for response with carriage return
        grbl_out = SERIAL_CONNECTION.readline()
        print("grbl response: " + grbl_out.strip().decode("utf-8"))
    print("Sent command: " + str(cmd))


def repl():
    while (1):
        # Get input
        val = input(">")
        print(val)
        # Send input
        ser.write(bytes("$J=G91 G20 X0.5 F10", 'utf-8'))
        # Get output
        while (line != ""):
            line = ser.readline().decode('utf-8')   # read a '\n' terminated line
            print(line)

def set_grbl_positive_directions():
    grbl_direction_port_invert_mask = z_axis_invert << 2 | y_axis_invert << 1 | x_axis_invert
    send_grbl_gcode_cmd(grbl_dir_port_invert_mask + str(grbl_direction_port_invert_mask))

def home_x_pusher():
    """Home the x axis"""
    print("Homing X axis")
    send_grbl_gcode_cmd("G91 G21 G1 X-5 F2000")
    send_grbl_gcode_cmd("G91 G21 G1 X22 F2000")
    is_homed_dict["x"] = True


def home_y_batt_indexer():
    """Home the y axis"""
    print("Homing Y axis")
    send_grbl_gcode_cmd("G91 G21 G1 Y10 F500")
    send_grbl_gcode_cmd("G91 G21 G1 Y-17 F500")
    # Slower derease speed so that we increase torque in case we have tension in belts near end.
    send_grbl_gcode_cmd("G91 G21 G1 Y-1 F10")
    is_homed_dict["y"] = True


def home_z_lift_arms():
    """Home the z axis"""
    print("Homing Z axis")
    # send_grbl_gcode_cmd("$J=G91 G21 Z-304.8 F2000")
    # send_grbl_gcode_cmd("G91 G21 G1 Z30.48 F2000")
    # send_grbl_gcode_cmd("G91 G21 G1 Z304.8 F2000")
    send_grbl_gcode_cmd(grbl_enable_homing_setting_cmd)
    send_grbl_gcode_cmd(grbl_run_homing_cycle_cmd)
    send_grbl_gcode_cmd(grbl_disable_soft_limits_setting_cmd)
    unESTOP()
    unESTOP()
    unESTOP()
    is_homed_dict["z"] = True


def blocking_wait_until_axes_stop_moving():
    """Wait until all axes stop moving"""
    SERIAL_CONNECTION.flushInput()
    sleep(0.5)  # Wait for previous command to send
    while (are_motors_moving()):
        sleep(0.5)
        continue
    SERIAL_CONNECTION.flushInput()


def set_home():
    """Set the home position"""
    send_grbl_gcode_cmd("G92 X0 Y0")


def home_all_axes():
    home_z_lift_arms()
    blocking_wait_until_axes_stop_moving()
    home_y_batt_indexer()
    blocking_wait_until_axes_stop_moving()
    set_home()
    sleep(1)
    move_indexer_to_first_empty_slot()
    home_x_pusher()
    blocking_wait_until_axes_stop_moving()
    assert (are_all_axes_homed() == True)
    set_home()
    


def move_x_pusher(loc: str):
    """Move the x axis to the specified location"""
    assert (loc in pusher_abs_loc_cm.keys())
    send_grbl_gcode_cmd(absolute_mode + " " + cm_mode + " " + G01_linear_interpolation +
                        " " + pusher_axis + str(pusher_abs_loc_cm[loc]) + " " + pusher_feed_rate)


def move_y_indexer(loc: str):
    """Move the y axis to the specified location"""
    assert (loc in indexer_abs_loc_cm.keys())
    send_grbl_gcode_cmd(absolute_mode + " " + cm_mode + " " + G01_linear_interpolation +
                        " " + indexer_axis + str(indexer_abs_loc_cm[loc]) + " " + batt_indexer_feed_rate)


def move_z_lift_arms(loc: str):
    """Move the z axis to the specified location"""
    assert (loc in lift_abs_loc_cm.keys())
    send_grbl_gcode_cmd(absolute_mode + " " + cm_mode + " " + G01_linear_interpolation +
                        " " + lift_axis + str(lift_abs_loc_cm[loc]) + " " + lift_feed_rate)


def get_first_empty_batt_slot() -> str:
    """Get the first empty battery slot"""
    for i in range(0, len(is_batt_slot_populated)):
        if is_batt_slot_populated[str(i)] == False:
            return str(i)
    return None


def move_indexer_to_first_empty_slot() -> str:
    """Move the indexer to the first empty slot
    Return the first empty slot.
    """
    empty_slot = get_first_empty_batt_slot()
    if empty_slot != None:
        move_indexer_to_slot(int(empty_slot))
    else:
        print("No empty slots!!")
    return empty_slot
        
def get_first_non_empty_batt_slot() -> str:
    """Get the first non-empty battery slot"""
    for i in range(0, len(is_batt_slot_populated)):
        if is_batt_slot_populated[str(i)] == True:
            return str(i)
    return None

def move_indexer_to_new_battery() -> str:
    """Move the indexer to a battery slot with with a battery with the highest charge
    Return the new battery slot.
    """
    # For now get first non empty slot
    non_empty_slot = get_first_non_empty_batt_slot()
    if non_empty_slot != None:    
        move_indexer_to_slot(int(non_empty_slot))
    else:
        print("No batteries available!!")
    return non_empty_slot


def move_indexer_to_slot(slot: int):
    """Move the indexer to the specified slot"""
    assert (slot >= 0 and slot < len(is_batt_slot_populated))
    # Move to the slot
    send_grbl_gcode_cmd(absolute_mode + " " + cm_mode + " " + G01_linear_interpolation +
                        " " + indexer_axis + str(indexer_abs_loc_cm["slot" + str(slot)]) + " " + indexer_feed_rate)
    # Mark the slot as populated
    is_batt_slot_populated[slot] = True


def are_all_axes_homed() -> bool:
    """Check if all axes are homed"""
    return is_homed_dict["x"] and is_homed_dict["y"] and is_homed_dict["z"]


def ESTOP():
    """Emergency stop. Kills all motion controlled by GRBL"""
    print("Emergency stop!!")
    # send_grbl_gcode_cmd("!")
    # this is ctrl-x (soft-reset)
    send_grbl_gcode_cmd_bytes(grbl_soft_reset_cmd)
    SERIAL_CONNECTION.flushInput()


def unESTOP():
    send_grbl_gcode_cmd("$X")
    send_grbl_gcode_cmd("$X")


def lift_minibot_check_alignment():
    """Lift minibot to ensure alignment"""
    print("Lifting minibot to check alignment")
    send_grbl_gcode_cmd("G91 G21 G1 Z-304.8 F2000")

# Once minibot is aligned, swap battery


def swap_battery():
    """Swap a single battery from minibot to hub and then hub to minibot"""
    print("Swapping battery...")
    if not are_all_axes_homed():
        print("Homing all axes")
        home_all_axes()

    move_x_pusher(loc="above_minibot")

    # Take out old battery from minibot into hub
    old_batt_slot = move_indexer_to_first_empty_slot()
    move_z_lift_arms(loc="liftup")
    move_x_pusher(loc="in_indexer")

    # Put new batteyr from hub into minibot
    new_batt_slot = move_indexer_to_new_battery()
    move_x_pusher(loc="above_minibot")
    move_z_lift_arms(loc="liftdown")
    is_batt_slot_populated[old_batt_slot] = True
    is_batt_slot_populated[new_batt_slot] = False


def are_motors_moving() -> bool:
    """Check if motors are moving"""
    status = get_status()
    # Checks if feed rates are 0
    if "FS:0,0" in status:
        return False
    return True


def move_lift_arms_to_minibot():
    """Move lift arms down to minibot"""
    print("Moving lift arms to minibot...")
    send_grbl_gcode_cmd("G90 G21 G1 Z27 F2000")


def get_status() -> str:
    """Get status from GRBL"""
    status = ""
    line = ""
    send_grbl_gcode_cmd(grbl_status_report_buffer_data_cmd)
    send_grbl_gcode_cmd(grbl_status_report_cmd, False)
    # Read all lines until it's done
    while ("ok" not in status):
        line = SERIAL_CONNECTION.readline().decode("utf-8")
        status += line
        print(line)
    return status


if __name__ == "__main__":
    get_com_port()
    connect()
    set_grbl_positive_directions()
    # Disable soft limits
    send_grbl_gcode_cmd(grbl_disable_soft_limits_setting_cmd)
    # exit(0)
    while (1):
        cmd_line = input(">")
        cmds = cmd_line.split()
        cmd = cmds.pop(0)
        if cmd == "exit":
            exit(0)
        elif cmd == "homez":
            home_z_lift_arms()
            set_home()
        elif cmd == "homex":
            home_x_pusher()
            set_home()
        elif cmd == "homey":
            home_y_batt_indexer()
            set_home()
        elif cmd == "homeall":
            home_all_axes()
        elif cmd == "sethome":
            set_home()
        elif cmd == "swap":
            swap_battery()
        elif cmd == "liftdownraw":
            # move_lift_arms_to_minibot()
            send_grbl_gcode_cmd("G91 G21 G1 Z200 F2000")
        elif cmd == "liftupraw":
            send_grbl_gcode_cmd("G91 G21 G1 Z-200 F400")
        elif cmd == "liftdown":
            move_z_lift_arms(loc="liftdown")
        elif cmd == "liftup":
            move_z_lift_arms(loc="liftup")
            # send_grbl_gcode_cmd("G90 G21 G1 Z-140 F400")
        elif cmd == "s":  # Stop
            ESTOP()
        elif cmd == "r":  # Resume
            unESTOP()
        elif cmd == "slot0":
            move_indexer_to_slot(0)
        elif cmd == "slot1":
            move_indexer_to_slot(1)
        elif cmd == "slot2":
            move_indexer_to_slot(2)
        elif cmd == "pushin":
            move_x_pusher(loc="in_indexer")
        elif cmd == "pushout":
            move_x_pusher(loc="above_minibot")
        elif cmd == "softreset":
            send_grbl_gcode_cmd(grbl_soft_reset_cmd)
        elif cmd == "repl":
            repl()
        elif cmd == "stat":
            get_status()
        else:
            send_grbl_gcode_cmd(cmd_line)

    # send_grbl_gcode_cmd("$J=G91 G20 X0.5 F10")
    # home_z_lift_arms()
    # home_x_batt_mover()
    # home_y_batt_indexer()
    # time.sleep(2)
    # move_lift_arms_to_minibot()

    # swap_battery()
