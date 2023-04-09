# Must be run not in a venv. Jetson.GPIO doesn't install in a venv for some reason.
# Valid pins: https://maker.pro/nvidia-jetson/tutorial/how-to-use-gpio-pins-on-jetson-nano-developer-kit
# Jetson.GPIO is here in venv:

import Jetson.GPIO as GPIO
import serial
import time
from time import sleep
from serial.tools import list_ports
import re
import sys
import os
from typing import List

# Board pins (i.e. the numbers that are on the Jetson itself)
# All active low relays
boost_0_en_pin = 7
boost_1_en_pin = 24
boost_2_en_pin = 12
boost_en_pins = [boost_0_en_pin, boost_1_en_pin, boost_2_en_pin]

# Toggle low then high.
charge_start_pin = 13
wing_power_en_pin = 15


class MCURXSerDataPkt:
    # 0=continous=batt detected
    BATT_NOT_DETECTED = 0
    BATT_DETECTED = 1
    battConts = []
    wingCont = 0

    battVoltsMvIndices = {'batt0': 0, 'batt1': 1, 'batt1': 2, 'hubbatt': 3}
    battVoltsMv = []

    def __init__(self, batt0Cont=0, batt1Cont=0, batt2Cont=0, wingCont=0, batt0VoltMv=0,
                 batt1VoltMv=0, batt2VoltMv=0, hubBattVoltMv=0):
        self.battConts = [batt0Cont, batt1Cont, batt2Cont]
        self.wingCont = wingCont
        self.battVoltsMv = [batt0VoltMv,
                            batt1VoltMv, batt2VoltMv, hubBattVoltMv]

    def getBattCont(self, slot: int) -> int:
        return self.battConts[slot]

    def getWingCont(self, slot: int) -> int:
        return self.wingCont

    def getBattVoltMv(self, slot: int) -> float:
        print(self.battVoltsMv[slot])
        return float(self.battVoltsMv[slot])

    def getBattVoltV(self, slot: int) -> float:
        print(self.battVoltsMv[slot])
        return float((self.battVoltsMv[slot] * 1000.0))


class MCUController:
    '''
    For controlling communication with the MCU
    Data must be sent over serial in this format:
    Example input data from MCU: >batt0Cont=0,batt1Cont=1,batt2Cont=1,wingCont=1
    '''
    mcu_port_VID = 1027
    mcu_port_PID = 24577
    USB_PORT = ""
    MCU_BPS = 115200
    SERIAL_CONNECTION: serial.Serial = None

    def __init__(self):
        self.USB_PORT = self.__get_com_port()
        print(self.USB_PORT)
        self.SERIAL_CONNECTION = self.__connect(self.USB_PORT, self.MCU_BPS)

    def get_mcu_serial_data_blocking(self) -> MCURXSerDataPkt:
        '''
        Will block here until we get the next packet of serial data from the MCU.
        Data from the MCU should all be sent in a single comma-separated line.
        '''
        mcu_rx_pkt = MCURXSerDataPkt()
        start = -1
        # Flush the input buffer since the MCU will have sent a bunch of data and
        # filled the Jetson's serial input buffer with a bunch of now old data.
        self.SERIAL_CONNECTION.flushInput()
        print("BLOCKING UNTIL SERIAL DATA RECEIVED FROM MCU")
        while (start == -1):
            line = self.SERIAL_CONNECTION.readline().decode("utf-8")
            start = line.find(">")
            print("Serial data from MCU:\n" + line)
        mcu_rx_pkt = self.__parse_mcu_raw_serial_data(line)
        print(mcu_rx_pkt)
        return mcu_rx_pkt

    def __get_com_port(self) -> str:
        print("Getting COM PORT")
        device_list = list_ports.comports()
        port = None
        print("Here are all detected COM Ports:")
        for device in device_list:
            print(device)
            print(device.vid)
            print(device.pid)
        print("===========================================\n\n")
        for device in device_list:
            if (device.vid != None or device.pid != None):
                if (device.vid == self.mcu_port_VID and
                        device.pid == self.mcu_port_PID):
                    port = device.device
                    print("MCU found on port:" + port)
                    break
                port = None
        if port == None:
            print("ERROR: MCU not found!!!!")
        return port

    def __connect(self, port: str, baud: int) -> serial.Serial:
        """Connect to the MCU controller"""
        print("Connecting to MCU...")
        tempSerConnection = serial.Serial(port,
                                          baudrate=baud,
                                          parity=serial.PARITY_NONE,
                                          stopbits=serial.STOPBITS_ONE,
                                          timeout=1)
        print(tempSerConnection)
        # Wake up MCU
        tempSerConnection.write(str.encode("\r\n\r\n"))
        # Wait for MCU to initialize and flush startup text in serial input
        time.sleep(2)
        tempSerConnection.flushInput()
        print("Connected to MCU")
        return tempSerConnection

    def __get_value_from_input_data_int(self, keyVal: str) -> int:
        data = keyVal.split("=")
        if len(data) != 2:
            print("ERROR getting key and value from MCU serial input data: " + keyVal)
            return None
        return int(data[1])

    def __get_value_from_input_data_float(self, keyVal: str) -> float:
        data = keyVal.split("=")
        if len(data) != 2:
            print("ERROR getting key and value from MCU serial input data: " + keyVal)
            return None
        return float(data[1])

    @classmethod
    def remove_all_occurences(self, arr: List[str], toRemove: str):
        for i in range(0, len(arr)):
            arr[i] = arr[i].replace(toRemove, '')

    def __parse_mcu_raw_serial_data(self, raw_data: str) -> MCURXSerDataPkt:
        # Remove whitespace
        raw_data = re.sub(r"\s+", "", raw_data)
        # Split data up
        data = raw_data[1:].split(",")
        # Get rid of any null characters
        self.remove_all_occurences(data, '\x00')

        print(data)
        # Extract the values
        return MCURXSerDataPkt(batt0Cont=self.__get_value_from_input_data_int(data[0]),
                               batt1Cont=self.__get_value_from_input_data_int(
                                   data[1]),
                               batt2Cont=self.__get_value_from_input_data_int(
                                   data[2]),
                               wingCont=self.__get_value_from_input_data_int(
                                   data[3]),
                               batt0VoltMv=self.__get_value_from_input_data_float(
                                   data[4]),
                               batt1VoltMv=self.__get_value_from_input_data_float(
                                   data[5]),
                               batt2VoltMv=self.__get_value_from_input_data_float(
                                   data[6]),
                               hubBattVoltMv=self.__get_value_from_input_data_float(
                                   data[7]),
                               )

def test_remove_all_occurences():
    test = ["hey\x00heyasd\x00asd\x00", "aldasdheyasldjashey"]
    expectedTest = ["\x00asd\x00asd\x00", "aldasdasldjas"]
    MCUController.remove_all_occurences(test, 'hey')
    assert(test == expectedTest)
    test = ["hey\x00heyasd\x00asd\x00", "aldasdheyasldjashey"]
    expectedTest = ["heyheyasdasd", "aldasdheyasldjashey"]
    MCUController.remove_all_occurences(test, '\x00')
    assert(test == expectedTest)

class ChargeController:
    mcu: MCUController
    FULL_6S_BATT_VOLTAGE_MV = 25.2 * 1000.0
    NUM_BATT_SLOTS = 3
    # Battery is charging state
    is_charging = []

    def __init__(self):
        print("Initializing ChargeController...")
        print(GPIO.JETSON_INFO)
        print(GPIO.VERSION)
        self.mcu = MCUController()
        self.is_charging = [False, False, False]
        self.__init_pins()
        self.disable_all_charging()
        print("Initialized ChargeController SUCCESSFULLY")

    def __init_pins(self):
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

        # pin = 24
        # GPIO.setup(pin, GPIO.OUT)
        # while(1):
        #     print("loop...")
        #     GPIO.output(pin, GPIO.HIGH)
        #     sleep(1)
        #     # print(GPIO.input(pin))
        #     # sleep(1)
        #     GPIO.output(pin, GPIO.LOW)
        #     # sleep(1)
        #     # print(GPIO.input(pin))
        #     sleep(1)

    def loopHook(self):
        data: MCURXSerDataPkt = self.mcu.get_mcu_serial_data_blocking()

        # Enable charging if battery detected in slot.
        for i in range(0, self.NUM_BATT_SLOTS):
            if self.is_batt_detected(i, data):
                if self.get_batt_voltage_mv(i, data) < (self.FULL_6S_BATT_VOLTAGE_MV - (0.1 * 1000.0)) \
                        and not self.is_charging[i]:
                    self.enable_charging(i, True)
            else:
                self.disable_charging(i)

    def enable_charging(self, slot: int, force: bool = False):
        print("Starting charging on slot " + str(slot) + "...")
        # Check that the battery is there
        if not force and not is_batt_detected(slot):
            print("BATTERY NOT DETECTED in slot " +
                  str(slot) + ". CANNOT start charging!!")
        else:
            sleep(1)
            # Enable the Boost converter
            GPIO.output(boost_en_pins[slot], GPIO.LOW)
            print("Boost enabled.")
            sleep(1)
            GPIO.output(charge_start_pin, GPIO.LOW)
            print("Start button low.")
            sleep(1)
            GPIO.output(charge_start_pin, GPIO.HIGH)
            self.is_charging[slot] = True
            print("Start button high.")
            print("Enabled charging on slot " + str(slot))

    def enable_wing_power(self, enable = True, force: bool = False) -> bool:
        # Check that the minibot is there
        if not force and not self.is_wing_cont_detected():
            print("MINIBOT NOT DETECTED in dock. CANNOT transfer power!!")
            return False
        else:
            if enable:
                print("Enabling wing power...")
                sleep(1)
                GPIO.output(wing_power_en_pin, GPIO.LOW)
                print("Wing Power Enable Pin button low.")
                print("Enabled Wing Power")
            else:
                GPIO.output(wing_power_en_pin, GPIO.HIGH)
                print("Wing Power Enable Pin button high.")
                print("Disabled Wing Power")
        return True

    def disable_all_charging(self):
        for i in range(0, self.NUM_BATT_SLOTS):
            self.disable_charging(i)

    def disable_charging(self, slot: int):
        print("Disabling charging on slot " + str(slot) + "...")
        GPIO.output(boost_en_pins[slot], GPIO.HIGH)
        self.is_charging[slot] = False
        print("Disabled charging on slot " + str(slot))

    def is_batt_detected(self, slot: int, data: MCURXSerDataPkt = None) -> bool:
        '''
        Checks if the given slot has a battery in it by checking continuity.
        '''
        # Active low. Continuity when LOW.
        if data == None:
            data = self.mcu.get_mcu_serial_data_blocking()
        if data.getBattCont(slot) == MCURXSerDataPkt.BATT_DETECTED:
            return False
        else:
            return True

    
    def blockPrint(self):
        sys.stdout = open(os.devnull, 'w')

    def enablePrint(self):
        sys.stdout = sys.__stdout__

    def is_wing_cont_detected(self, data: MCURXSerDataPkt = None) -> bool:
        # Active low. Continuity when LOW.
        if data == None:
            self.blockPrint()
            data = self.mcu.get_mcu_serial_data_blocking()
            data = self.mcu.get_mcu_serial_data_blocking()
            self.enablePrint()
        if data.wingCont == MCURXSerDataPkt.BATT_DETECTED:
            return False
        else:
            return True

    def get_batt_voltage_mv(self, slot: int, data: MCURXSerDataPkt = None) -> float:
        if data == None:
            data = self.mcu.get_mcu_serial_data_blocking()
        # If the battery is not detected based on continuity then return 0.0V
        if (slot <= 2):
            if not self.is_batt_detected(slot):
                return 0.0
        return float(data.getBattVoltMv(slot))

    def ESTOP(self):
        exit(0)


def repl():
    cont = ChargeController()

    while (1):
        cmd_line = input(">")
        cmds = cmd_line.split()
        cmd = cmds.pop(0)
        if cmd == "e":
            ESTOP()
        elif cmd == "c0":
            cont.enable_charging(0)
        elif cmd == "c1":
            cont.enable_charging(1)
        elif cmd == "c2":
            cont.enable_charging(2)
        elif cmd == "s0":
            cont.disable_charging(0)
        elif cmd == "s1":
            cont.disable_charging(1)
        elif cmd == "s2":
            cont.disable_charging(2)
        elif cmd == "d0":
            print("BattCont: " + str(cont.is_batt_detected(0)))
        elif cmd == "d1":
            print("BattCont: " + str(cont.is_batt_detected(1)))
        elif cmd == "d2":
            print("BattCont: " + str(cont.is_batt_detected(2)))
        elif cmd == "d3":
            while(1):
                print("WingCont: " + str(cont.is_wing_cont_detected()))
        elif cmd == "v0":
            print("Batt0Voltage: " + str(cont.get_batt_voltage_mv(0)))
        elif cmd == "v1":
            print("Batt0Voltage: " + str(cont.get_batt_voltage_mv(1)))
        elif cmd == "v2":
            print("Batt0Voltage: " + str(cont.get_batt_voltage_mv(2)))
        elif cmd == "v3":
            print("Batt0Voltage: " + str(cont.get_batt_voltage_mv(3)))
        elif cmd == "enablewingpow":
            cont.enable_wing_power(True, True)
        elif cmd == "disablewingpow":
            cont.enable_wing_power(False, True)
        else:
            print("Command not found: " + cmd)


def loop_mode():
    controller = ChargeController()

    controller_last_loop_time_ms = time.time() * 1000.0
    while (1):
        time_now_ms = time.time() * 1000.0

        if (time_now_ms - controller_last_loop_time_ms) > 1000:
            controller_last_loop_time_ms = time_now_ms
            controller.loopHook()


def repl_mode():
    repl()

def test_MCUController():
    t = MCUController()

if __name__ == "__main__":
    try:
        repl_mode()
        # loop_mode()
    except KeyboardInterrupt:
        print("Keyboard Interrupts")
    finally:
        GPIO.cleanup()  # this ensures a clean exit
