#ifndef _TXB0104PWR_H_
#define _TXB0104PWR_H_

#include "Arduino.h"
#include "utils/log.h"
#include "utils/macros.h"

// A 5V to 3v3 level shifter
class TXB0104PWR {
public:
  TXB0104PWR(uint32_t _enablePin) : enablePin(_enablePin) {}

  bool enable() {
    bool success = true;
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);
    return success;
  }

  bool disable() {
    bool success = true;
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW);
    return success;
  }

  bool init() {
    INIT_HEADER
    pinMode(enablePin, OUTPUT);
    success = success && enable();
    INIT_FOOTER
  }

private:
  uint32_t enablePin = NOPIN;
};

#endif // _TXB0104PWR_H_
