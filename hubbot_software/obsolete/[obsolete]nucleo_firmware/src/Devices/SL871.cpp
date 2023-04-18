#include "SL871.h"

bool SL871::reset() {
  bool success = true;
  LOGEVENT("Sending SL871 reset signal.");
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  delay(100);
  // SL871 reset pin should not be driven HIGH.
  // It has internal pullup.
  pinMode(resetPin, INPUT_FLOATING);
  LOGEVENT("SL871 reset signal sent.");
  return success;
}