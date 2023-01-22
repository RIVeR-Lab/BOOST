#ifndef _SRC_PINS_H
#define _SRC_PINS_H

#include "Arduino.h"

static const uint32_t NOPIN = 0xFFFFFFFF;

// Inhibit Line
static const uint32_t INHIBIT_PIN = NOPIN;

#if SEEED_XIAO
#error "SEEED"
#define Console  Serial1

// JUMPER
enum VoltageToPinMapping_t : uint32_t {
  DIP_24V_PIN = A0,
  DIP_48V_PIN = A1,
  DIP_200V_PIN = A2,
  DIP_NONE_PIN = NOPIN,
};

// BUTTON
static const uint8_t PROGRAM_BTN_PIN = A7;

// ePaper
static const uint32_t EPD_READY = A3;
static const uint32_t EPD_RESET = A4;
static const uint32_t EPD_DC = A5;
static const uint32_t EPD_CS = A7;
// static const uint32_t EPD_MOSI = A10;
// static const uint32_t EPD_SCK = A8;
#elif NUCLEO_L432KC
extern HardwareSerial SerialUSB;
#define Console SerialUSB

// JUMPER
enum VoltageToPinMapping_t : uint32_t {
  vpmDIP_24V_PIN = PA8,  //D9
  vpmDIP_48V_PIN = PA11, //D10
  vpmDIP_200V_PIN = PB5, //D11
  vpmDIP_NONE_PIN = NOPIN,
  vpmUNKNOWN_VOLTAGE = NOPIN,
};

// BUTTON
static const uint8_t PROGRAM_BTN_PIN = PB1; //D6

#endif


#endif // _SRC_PINS_H