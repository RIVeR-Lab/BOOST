# Must be run not in a venv. Jetson.GPIO doesn't install in a venv for some reason.
# Valid pins: https://maker.pro/nvidia-jetson/tutorial/how-to-use-gpio-pins-on-jetson-nano-developer-kit
# Jetson.GPIO is here in venv:

import Jetson.GPIO as GPIO
import serial
import time
from time import sleep
from serial.tools import list_ports
import re

# ############################### PIN DEFS ###############################

# Board pins (i.e. the numbers that are on the Jetson itself)
# All active low relays
boost_0_en_pin = 7
boost_1_en_pin = 11
boost_2_en_pin = 12
boost_en_pins = [boost_0_en_pin, boost_1_en_pin, boost_2_en_pin]

# Toggle low then high.
charge_start_pin = 13

wing_power_en_pin = 15

# Continuity pins
# Active low (i.e. if=low, then battery is connected.)
cont0_pin = 21
cont1_pin = 33
cont2_pin = 31
continuity_pins = [cont0_pin, cont1_pin, cont2_pin]
wing_continuity = 33


nucleo_port_VID = 1155
nucleo_port_PID = 14155
USB_PORT = ""
NUCLEO_BPS = 57600
SERIAL_CONNECTION = None

# ############################### END PIN DEFS ###############################

# ############################### STATE ###############################

# Example input data: >batt0Cont=0,batt1Cont=1,batt2Cont=1,wingCont=1


class NucRXSerDataPkt:
    # 0=continous=batt detected
    BATT_NOT_DETECTED = 0
    BATT_DETECTED = 1
    battConts = []

    def __init__(self, batt0Cont: int, batt1Cont: int, batt2Cont: int, wingCont: int):
        self.battConts = [batt0Cont, batt1Cont, batt2Cont]
        self.wingCont = wingCont
    
    def getBattCont(self, slot: int) -> int:
        return self.battConts[slot]


last_nucleo_data_reading = NucRXSerDataPkt(
    NucRXSerDataPkt.BATT_NOT_DETECTED, NucRXSerDataPkt.BATT_NOT_DETECTED, NucRXSerDataPkt.BATT_NOT_DETECTED, NucRXSerDataPkt.BATT_NOT_DETECTED)

# ############################### END STATE ###############################


print(GPIO.JETSON_INFO)
print(GPIO.VERSION)


def get_com_port():
    global USB_PORT
    device_list = list_ports.comports()
    port = None
    for device in device_list:
        print(device)
        print(device.vid)
        print(device.pid)
        if (device.vid != None or device.pid != None):
            if (device.vid == nucleo_port_VID and
                    device.pid == nucleo_port_PID):
                port = device.device
                print("NUCLEO found on port:" + port)
                break
            port = None
    USB_PORT = port


def connect():
    global SERIAL_CONNECTION
    """Connect to the NUCLEO controller"""
    print("Connecting to NUCLEO...")
    SERIAL_CONNECTION = serial.Serial(USB_PORT, NUCLEO_BPS, timeout=1)
    # Wake up NUCLEO
    SERIAL_CONNECTION.write(str.encode("\r\n\r\n"))
    # Wait for NUCLEO to initialize and flush startup text in serial input
    time.sleep(2)
    SERIAL_CONNECTION.flushInput()
    print("Connected to NUCLEO")


def setup():
    print("Setting up pins...")
    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)

    # Outputs
    GPIO.setup(boost_0_en_pin, GPIO.OUT)
    GPIO.setup(boost_1_en_pin, GPIO.OUT)
    GPIO.setup(boost_2_en_pin, GPIO.OUT)
    GPIO.setup(charge_start_pin, GPIO.OUT)
    GPIO.setup(wing_power_en_pin, GPIO.OUT)

    # Start all HIGH
    GPIO.output(boost_0_en_pin, GPIO.HIGH)
    GPIO.output(boost_1_en_pin, GPIO.HIGH)
    GPIO.output(boost_2_en_pin, GPIO.HIGH)
    GPIO.output(charge_start_pin, GPIO.HIGH)
    GPIO.output(wing_power_en_pin, GPIO.HIGH)

    # Setup NUCLEO connection
    get_com_port()
    connect()


def is_batt_detected(slot: int) -> bool:
    '''
    Checks if the given slot has a battery in it by checking continuity.
    '''
    # Active low. Continuity when LOW.
    # First get a new status packet from Nucleo
    data: NucRXSerDataPkt = get_nucleo_serial_data_blocking()
    if data.getBattCont(slot) == NucRXSerDataPkt.BATT_DETECTED:
        return False
    else:
        return True

def is_wing_cont_detected() -> bool:
    # Active low. Continuity when LOW.
    # First get a new status packet from Nucleo
    data: NucRXSerDataPkt = get_nucleo_serial_data_blocking()
    if data.wingCont == NucRXSerDataPkt.BATT_DETECTED:
        return False
    else:
        return True


def enable_charging(slot: int, force: bool = False):
    print("Starting charging on slot " + str(slot) + "...")
    # Check that the battery is there
    if not is_batt_detected(slot):
        print("BATTERY NOT DETECTED in slot " +
              str(slot) + ". CANNOT start charging!!")
    else:
        # Enable the Boost converter
        GPIO.output(boost_en_pins[slot], GPIO.LOW)
        print("Boost enabled.")
        sleep(1)
        GPIO.output(charge_start_pin, GPIO.LOW)
        print("Start button low.")
        sleep(1)
        GPIO.output(charge_start_pin, GPIO.HIGH)
        print("Start button high.")
        print("Enabled charging on slot " + str(slot))


def disable_charging(slot: int):
    print("Disabling charging on slot " + str(slot) + "...")
    GPIO.output(boost_en_pins[slot], GPIO.HIGH)
    print("Disabled charging on slot " + str(slot))


def ESTOP():
    exit(0)


def get_nucleo_serial_data_blocking() -> NucRXSerDataPkt:
    '''
    Will block here until we get the next packet of serial data from the NUCLEO.
    Data from the NUCLEO should all be sent in a single comma-separated line.
    '''
    global last_nucleo_data_reading
    start = -1
    # Flush the input buffer since the NUCLEO will have sent a bunch of data and
    # filled the Jetson's serial input buffer with a bunch of now old data.
    SERIAL_CONNECTION.flushInput() 
    while(start == -1):
        line = SERIAL_CONNECTION.readline().decode("utf-8")
        start = line.find(">")
        print(line)
    last_nucleo_data_reading = parse_nucleo_raw_serial_data(line)
    print(last_nucleo_data_reading)
    return last_nucleo_data_reading

def get_value(keyVal: str) -> int:
  valStart = keyVal.find("=")
  if valStart != -1:
    return int(keyVal[valStart+1])
  else:
    return None

def parse_nucleo_raw_serial_data(raw_data: str) -> NucRXSerDataPkt:
    # Remove whitespace 
    raw_data = re.sub(r"\s+", "", raw_data)
    # Split data up
    data = raw_data[1:].split(",")
    # Extract the values
    return NucRXSerDataPkt(get_value(data[0]), get_value(data[1]), get_value(data[2]), get_value(data[3]))


def test_input():
    test_pin = 21  # 26 #24 #21 #22
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(test_pin, GPIO.IN)
    print("Starting REPL...")
    while (1):
        print("BattCont: " + str(GPIO.input(test_pin)))
        sleep(0.1)


def repl():
    global last_nucleo_data_reading
    # while (1):
    #     data = get_nucleo_serial_data_blocking()
    #     print(data.battConts)
    #     print(data.wingCont)

    while (1):
        cmd_line = input(">")
        cmds = cmd_line.split()
        cmd = cmds.pop(0)
        if cmd == "e":
            ESTOP()
        elif cmd == "c0":
            enable_charging(0)
        elif cmd == "c1":
            enable_charging(1)
        elif cmd == "c2":
            enable_charging(2)
        elif cmd == "s0":
            disable_charging(0)
        elif cmd == "s1":
            disable_charging(1)
        elif cmd == "s2":
            disable_charging(2)
        elif cmd == "d0":
            print("BattCont: " + str(is_batt_detected(0)))
        elif cmd == "d1":
            print("BattCont: " + str(is_batt_detected(1)))
        elif cmd == "d2":
            print("BattCont: " + str(is_batt_detected(2)))
        elif cmd == "d3":
            print("WingCont: " + str(is_wing_cont_detected()))
        else:
            print("Command not found: " + cmd)


if __name__ == "__main__":
    try:
        setup()
        repl()
    except KeyboardInterrupt:
        print("Keyboard Interrupts")
    finally:
        GPIO.cleanup()  # this ensures a clean exit


def crap():
    try:
        while True:
            time.sleep(1)
            # print(GPIO.output(output_pin, LOW))

            # Toggle the output every second
            print("Outputting {} to pin {}".format(curr_value, output_pin))
            GPIO.output(output_pin, curr_value)
            curr_value ^= GPIO.HIGH
    finally:
        GPIO.cleanup()
