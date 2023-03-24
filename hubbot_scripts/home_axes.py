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

USB_PORT = "COM9"
GRBL_BPS = 115200
SERIAL_CONNECTION = serial.Serial(USB_PORT, GRBL_BPS, timeout=1)

# State, these are indexed when looking from back of hubbot
is_batt_slot_populated = {"0": False, "1": True, "2": True}
batt_slot_abs_loc_cm = [0, 4.75, 9.75]
is_homed = {"x": False, "y": False, "z": False}

# GCode commands
cm_mode = "G21"
inch_mode = "G20"
absolute_mode = "G90"
relative_mode = "G91"
indexer_feed_rate = "F100"
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


# GRBL RealTime commands
grbl_soft_reset_cmd = b"\x18"
grbl_feed_hold_cmd = "!"


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


def home_x_pusher():
    """Home the x axis"""
    print("Homing X axis")
    send_grbl_gcode_cmd("G91 G21 G1 X-5 F2000")
    send_grbl_gcode_cmd("G91 G21 G1 X22 F2000")
    is_homed["x"] = True

# Battery indexer


def home_y_batt_indexer():
    """Home the y axis"""
    print("Homing Y axis")
    send_grbl_gcode_cmd("G91 G21 G1 Y17 F500")
    send_grbl_gcode_cmd("G91 G21 G1 Y-17 F500")
    # Slower derease speed so that we increase torque in case we have tension in belts near end.
    send_grbl_gcode_cmd("G91 G21 G1 Y-1 F10")
    is_homed["y"] = True


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
    is_homed["z"] = True


def blocking_wait_until_axes_stop_moving():
    """Wait until all axes stop moving"""
    while (are_motors_moving()):
        sleep(0.5)
        continue


def home_all_axes():
    home_z_lift_arms()
    blocking_wait_until_axes_stop_moving()
    home_x_pusher()
    blocking_wait_until_axes_stop_moving()
    home_y_batt_indexer()
    blocking_wait_until_axes_stop_moving()
    assert (are_all_axes_homed() == True)
    move_indexer_to_first_empty_slot()


def get_first_empty_batt_slot() -> str:
    """Get the first empty battery slot"""
    for i in range(0, len(is_batt_slot_populated)):
        if is_batt_slot_populated[str(i)] == False:
            return str(i)
    return None


def move_indexer_to_first_empty_slot():
    """Move the indexer to the first empty slot"""
    empty_slot = get_first_empty_batt_slot()
    if empty_slot != None:
        move_indexer_to_slot(int(empty_slot))
    else:
        print("No empty slots!!")


def move_indexer_to_slot(slot: int):
    """Move the indexer to the specified slot"""
    assert (slot >= 0 and slot < len(is_batt_slot_populated))
    assert (is_batt_slot_populated[str(slot)] == False)
    # Move to the slot
    send_grbl_gcode_cmd("G91 G21 G1 Y" + str(slot * 25.4) + " F500")
    send_grbl_gcode_cmd("G90 G21 G1 Y0 F100")
    send_grbl_gcode_cmd(absolute_mode + " " + cm_mode + " " + G01_linear_interpolation +
                        " " + indexer_axis + batt_slot_abs_loc_cm[slot] + " " + indexer_feed_rate)
    # Mark the slot as populated
    is_batt_slot_populated[slot] = True


def are_all_axes_homed() -> bool:
    """Check if all axes are homed"""
    return is_homed["x"] and is_homed["y"] and is_homed["z"]


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
    send_grbl_gcode_cmd(grbl_status_report_buffer_data_cmd)
    send_grbl_gcode_cmd(grbl_status_report_cmd)
    # Read all lines until it's done
    while (1):
        line = SERIAL_CONNECTION.readline().decode("utf-8")
        status += line
        print(line)
        if "ok" not in line:
            break
    return status


if __name__ == "__main__":
    connect()
    # Disable soft limits
    send_grbl_gcode_cmd(grbl_disable_soft_limits_setting_cmd)
    while (1):
        cmd = input(">")
        cmds = cmd.split()
        cmd = cmds.pop(0)
        if cmd == "exit":
            exit(0)
        elif cmd == "homez":
            home_z_lift_arms()
        elif cmd == "homex":
            home_x_pusher()
        elif cmd == "homey":
            home_y_batt_indexer()
        elif cmd == "homeall":
            home_all_axes()
        elif cmd == "sethome":
            send_grbl_gcode_cmd("G92 X0 Y0")
        elif cmd == "swap":
            swap_battery()
        elif cmd == "liftdownraw":
            # move_lift_arms_to_minibot()
            send_grbl_gcode_cmd("G91 G21 G1 Z200 F2000")
        elif cmd == "liftupraw":
            send_grbl_gcode_cmd("G91 G21 G1 Z-200 F400")
        elif cmd == "liftdown":
            send_grbl_gcode_cmd("G90 G21 G1 Z0 F2000")
        elif cmd == "liftup":
            send_grbl_gcode_cmd("G90 G21 G1 Z-140 F400")
        elif cmd == "s":  # Stop
            ESTOP()
        elif cmd == "r":  # Resume
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
            send_grbl_gcode_cmd("G90 G21 G1 X-0.5 F200")
        elif cmd == "softreset":
            send_grbl_gcode_cmd(grbl_soft_reset_cmd)
        elif cmd == "repl":
            repl()
        elif cmd == "stat":
            get_status()
        else:
            send_grbl_gcode_cmd(cmd)

    # send_grbl_gcode_cmd("$J=G91 G20 X0.5 F10")
    # home_z_lift_arms()
    # home_x_batt_mover()
    # home_y_batt_indexer()
    # time.sleep(2)
    # move_lift_arms_to_minibot()

    # swap_battery()
