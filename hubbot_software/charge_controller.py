# Must be run not in a venv. Jetson.GPIO doesn't install in a venv for some reason.
# Valid pins: https://maker.pro/nvidia-jetson/tutorial/how-to-use-gpio-pins-on-jetson-nano-developer-kit
# Jetson.GPIO is here in venv:

import Jetson.GPIO as GPIO
import time
from time import sleep

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


# ############################### END PIN DEFS ###############################

print(GPIO.JETSON_INFO)
print(GPIO.VERSION)

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

    # Inputs
    GPIO.setup(cont0_pin, GPIO.IN)
    GPIO.setup(cont1_pin, GPIO.IN)
    GPIO.setup(cont2_pin, GPIO.IN)
    GPIO.setup(wing_continuity, GPIO.IN)

def is_batt_detected(slot: int) -> bool:
    '''
    Checks if the given slot has a battery in it by checking continuity.
    '''
    # Active lwow. Continuity hen LOW.
    # print(GPIO.input(continuity_pins[slot]))
    if GPIO.input(continuity_pins[slot]) == GPIO.HIGH:
        return False
    else:
        return True

def enable_charging(slot: int, force: bool = False):
    print("Starting charging on slot " + str(slot) + "...")
    # Check that the battery is there
    if not is_batt_detected(slot):
        print("BATTERY NOT DETECTED in slot " + str(slot) + ". CANNOT start charging!!")
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

def repl():
    test_pin = 21 # 26 #24 #21 #22
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(test_pin, GPIO.IN)
    print("Starting REPL...")
    while(1):
        print("BattCont: " + str(GPIO.input(test_pin)))
        sleep(0.1)

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
            while(1):
                print("BattCont: " + str(is_batt_detected(0)))
                sleep(0.1)
        elif cmd == "d1":
            print("BattCont: " + str(is_batt_detected(1)))
        elif cmd == "d2":
            print("BattCont: " + str(is_batt_detected(2)))
        else:
            print("Command not found: " + cmd) 

if __name__ == "__main__":
    try:  
        setup()
        repl()
    except KeyboardInterrupt:  
        print("Keyboard Interrupts")
    finally:  
        GPIO.cleanup() # this ensures a clean exit  

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