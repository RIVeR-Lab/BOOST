#ifndef _SL871_H_
#define _SL871_H_

#include "Arduino.h"
#include "HardwareSerial.h"
#include "utils/log.h"

class SL871 {
public:
  /**
   * @param _rxPin: RX pin of SL871
   * @param _txPin: TX pin of SL871
   */
  SL871(HardwareSerial &_uart, uint32_t _sl871RxPin, uint32_t _sl871TxPin,
        uint32_t _resetPin, uint32_t _1ppsPin, uint32_t _forceOnNPin)
      : uart(_uart), sl871RxPin(_sl871RxPin), sl871TxPin(_sl871TxPin),
        resetPin(_resetPin), ppsPin(_1ppsPin), forceOnNPin(_forceOnNPin) {}
  ~SL871() {}

  bool init() {
    LOGEVENT("Intializing SL871.");
    uart.end();
    uart.setRx(sl871TxPin);
    uart.setTx(sl871RxPin);
    uart.begin(DEFAULT_BAUDRATE);
    pinMode(resetPin, INPUT_FLOATING);
    reset();
    LOGEVENT("SL871 initialized.");
    return true;
  }

  bool reset() {
    bool success = true;
    LOGEVENT("Sending SL871 reset signal.");
    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, LOW);
    delay(100);
    // SL871 reset pin should not be driven HIGH.
    // It has internal pullup.
    digitalWrite(resetPin, INPUT_FLOATING);
    LOGEVENT("SL871 reset signal sent.");
    return success;
  }
  // bool send(uint8_t *data, uint32_t len);

  /**
   * @return number of bytes received, -1 if error.
  */
  int recv(uint8_t *data, uint32_t len) {
    int ret = -1;
    uint32_t i = 0;
    while (i < len && uart.available()) {
      int temp = uart.read();
      // No data left.
      if (temp == -1) {
        ret = -1;
        break;
      } else {
        data[i] = (uint8_t)temp;
        i++;
        ret = i;
      }
    }
    // LOGEVENT("SL871 received %d bytes.", i);
    LOGEVENT("uart.available() = %d", uart.available());
    return ret;
  }

private:
  static constexpr unsigned long DEFAULT_BAUDRATE = 9600;

  HardwareSerial &uart;
  const uint32_t sl871RxPin = NOPIN;
  const uint32_t sl871TxPin = NOPIN;
  const uint32_t resetPin = NOPIN;
  const uint32_t ppsPin = NOPIN;
  const uint32_t forceOnNPin = NOPIN;
  
};

#endif // _SL871_H_