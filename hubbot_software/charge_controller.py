# Must be run not in a venv. Jetson.GPIO doesn't install in a venv for some reason.
# Jetson.GPIO is here in venv:

import Jetson.GPIO as GPIO
import time

# Board pins (i.e. the numbers that are on the Jetson itself)
output_pin = 18

print(GPIO.JETSON_INFO)
print(GPIO.VERSION)

# Pin Setup:
GPIO.setmode(GPIO.BOARD)  # BCM pin-numbering scheme from Raspberry Pi
# set pin as an output pin with optional initial state of HIGH
# GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(output_pin, GPIO.IN)

print("Starting demo now! Press CTRL+C to exit")
curr_value = GPIO.HIGH
try:
    while True:
        time.sleep(1)
        print(GPIO.input(output_pin))

        # Toggle the output every second
        # print("Outputting {} to pin {}".format(curr_value, output_pin))
        # GPIO.output(output_pin, curr_value)
        # curr_value ^= GPIO.HIGH
finally:
    GPIO.cleanup()


GPIO.setup(channel, GPIO.OUT, initial=GPIO.HIGH)


GPIO.cleanup()
