/**
 * ubxNeo6M.h
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#ifndef SRC_UBXNEO6M_H
#define SRC_UBXNEO6M_H
#include "uartBase.h"
#include <string.h>
#include "zephyr.h"
#include "logging/log.h"


class ubxNeo6M {
public:
  ubxNeo6M(uartBase &_uart);
  ~ubxNeo6M();
  bool readIn();

private:
    uartBase uartPort;
    static const uint32_t rxBufferLen = 512;
    char rxBuffer[rxBufferLen] = { 0 };
    uint32_t rxBufferI = 0;
};

#endif // SRC_UBXNEO6M_H
