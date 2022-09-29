
/**
 * ubxNeo6M.cpp
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#include "ubxNeo6M.h"
LOG_MODULE_DECLARE(gps, LOG_LEVEL_NONE);

ubxNeo6M::ubxNeo6M(uartBase &_uart) : uartPort(_uart) {}
ubxNeo6M::~ubxNeo6M() {}
int ubxNeo6M::readIn() {
  bool success = true;
  unsigned char inChar;

  int stat = uartPort.poll_read(inChar);
  // If char arrived, add to buffer and keep reading until no more characters to read.
  while (stat == 0) {
    stat = uartPort.poll_read(inChar);

    // Add character to the encoder
    gpsEncoder.encode(inChar);
    // memcpy(&rxBuffer[rxBufferI++], &inChar, 1);
    // // If buffer is full, print it out
    // if (rxBufferI >= rxBufferLen) {
    //   rxBufferI = 0;
    //   LOG_DBG("\n%s\r\n", log_strdup(rxBuffer));
    // }
  }

  success = success && (stat == 0 || stat == -1);
  return stat;
}