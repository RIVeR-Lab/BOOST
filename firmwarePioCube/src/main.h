#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>  // This include order matters for Arduino's INPUT macro not getting overwritten.
// #include "rosHandler.h"
// #include "RealMain.h"
// #include <HardwareSerial.h>

//
#include "utils/log.h"
//
#include "pins.h"

// #include "ros.h"
// #include "std_msgs"

// #include <micro_ros_arduino.h>

// #include "HardwareSerial.h"
// #include "adc.h"
// #include "pwm.h"

// HardwareSerial Console;
#ifdef NUCLEO_F767ZI_CUSTOM
#define Console Serial3
#endif
#ifdef NUCLEO_F446RE_CUSTOM
// extern HardwareSerial serial2;
// HardwareSerial mySerial4;
#define Console Serial2
#endif


#endif /* MAIN_H */