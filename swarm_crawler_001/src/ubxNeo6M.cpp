
/**
 * ubxNeo6M.cpp
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#include "ubxNeo6M.h"

ubxNeo6M::ubxNeo6M(uartBase &_uart) : uartPort(_uart) {}
ubxNeo6M::~ubxNeo6M() {}
bool ubxNeo6M::readIn() {
  bool success = true;
  uint8_t inChar;

  int stat = uartPort.poll_read(inChar);
  // If char arrived, add to buffer.
  if (stat == 0) {
    memcpy(&rxBuffer[rxBufferI++], &inChar, 1);
    // If buffer is full, print it out
    if (rxBufferI >= rxBufferLen) {
      rxBufferI = 0;
      printk("%s", rxBuffer);
    }
  }

  success = success && (stat == 0 || stat == -1);
  return success;
}