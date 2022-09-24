/**
 * uartBase.h
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#ifndef SRC_UARTBASE_H
#define SRC_UARTBASE_H
#include <device.h>
#include <zephyr.h>
#include <drivers/uart.h>

class uartBase {
public:
  uartBase(const struct device &_uartDev);
  ~uartBase();
  int poll_read(unsigned char &p_char);

private:
  const struct device &uartDev;
};

#endif // SRC_UARTBASE_H
