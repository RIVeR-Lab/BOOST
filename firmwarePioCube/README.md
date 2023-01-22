# XP PSU Programmer
This code/project is for our InkBit manufacturing device used to program our 3-phase XP PSUs to their appropriate settings/configuration.
## How to use the Programmer
1. This assumes the programmer is all assembled.
1. Plug the cable into the power supply you want to program and into the external port on the PSU programmer.
1. Ensure that the voltage shown on the programmer's display is the voltage you want to program the PSU to.
1. Press the button on the front of the programmer.
    1. It will display what it is doing as it is programming/configuring the PSU.

## Setup for Developing
### Abbreviated Setup For Developing
1. Install VScode.
1. Install extension "PlatformIO IDE".
1. Install STM32CubeProgrammer which will install necessary USB drivers for STLink.
    1. https://www.st.com/en/development-tools/stm32cubeprog.html
1. In VScode, open folder "\inkbit\Tools\psu_benchtop_programmer\"
1. Plug MicroUSB into NUCLEO_L432KC board and into PC.
1. In VScode, click upload button in the lower menu bar.
### Setup For Developing With More Info
* If you're lazy, just follow the numbered steps. The bullets are extra info.
1. Install VScode.
1. Clone our inkbit repo.
1. Switch to your branch.
    * In the repo make a new branch if you haven't already corresponding to a ticket (make the ticket if you haven't), switch to the branch.
1. In VScode, install the "PlatformIO IDE" extension.
    * In VScode. Go to extensions in left sidebar. Search "PlatformIO" and install it. Reload your window.
    * On Windows, this will install PlatformIO's core in "/c/Users/[yourUserName]/.platformio". You don't need to do anything with this .platformio folder, but it is good to know where all the packages are stored for debugging.
    * PlatformIO (aka PIO) is a well known, well used, actively developed, and well tested extension that turns VScode (and just about any other IDE or code editor) into a full embedded IDE supporting lots of platforms. It manages dependencies and all the tooling for you.
    * Everything in this project is Arduiuno based. i.e. it is the same thing as using Arduiuno IDE except easier and better and can do more things imo.
1. In VScode, go to File->Open Folder, browse in the InkBit repo for "\inkbit\Tools\psu_benchtop_programmer\psu_benchtop_programmer_arduino_v2"
    * You should be opening a folder that has a platformio.ini file in it. This is how PlatformIO knows it is a PlatformIO project.
1. Plug a MicroUSB cable into the NUCLEO_L432KC board and into PC.
1. Click the upload button in the lower menu bar to both build and upload to target.
    * This will install all needed dependencies.
    * This will also autodetect the NUCLEO_L432KC board and upload firmware to it.
![](readme_images/Screenshot%202022-12-22%20160645.jpg)

## Setup For Debugging
1. NUCLEO boards have an STLink on them already.
1. Plug MicroUSB into NUCLEO board and into PC.
1. Click the debugging button with "PIO Debug".
![](readme_images/Screenshot%202022-12-23%20110452.jpg)
1. Only 5 breakpoints at a time.
    * Use Jlink if you want more.
1. I personally prefer using Eclipse for debugging since it has better code "Go To" behaviour and better parsing of variables. Google "PlatformIO for Eclipse" if you want to do this.

## Setup For Programming Only
1. Plug the NUCLEO_L432KC board into your computer via microUSB.
1. The board should appear in File Explorer as a drive with name "NODE_L432KC".
1. Drag and drop the latest firmware.bin file into this driver/folder and it will program the board.