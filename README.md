# swarm_crawler
* Making swarms crawl

## What does what

`/hubbot_software/`
* The software running on the NVidia Jetson Nano on the HubBot
* Includes the main ROS2 node running on the HubBot in `hubbot_software\ros2_ws\src\py_hubbot_main_controller_node\py_hubbot_main_controller_node`

`/minibot_hubbot_firmware/charge_controller_firmware/`
* Code running on the Arduino Nano on the HubBot that controls the LEDs and reads battery module continuity and voltage and sends to Jetson Nano over UART.

`/minibot_hubbot_firmware/firmwarePioCube/`
* Code running on the STM32F466RE NUCLEO on our Custom PCB.
* This board is identical on the 2x minibots and on the hubbot.
* Controls IMU, GPS, motor control, encoders, battery voltage.