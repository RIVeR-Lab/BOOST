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
  bool initialize();
  int poll_read(unsigned char &p_char);
  bool read(unsigned char &p_char);
  // bool write(unsigned char p_char);

private:
  static const int rxFifoSize = 512;
  static const int txFifoSize = 512;
  uint8_t rxFifo[rxFifoSize];
  uint8_t txFifo[txFifoSize];
  const struct device &uartDev;
  static void uart_fifo_callback(const struct device *dev, void *uartInstance);
};

#endif // SRC_UARTBASE_H
