#ifndef _SL871_H_
#define _SL871_H_

#include "Arduino.h"
#include "HardwareSerial.h"
#include "data/GpsDatagram.h"
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
  static constexpr float GPS_HORIZONTAL_ACCURACY_METERS = 2.5;
  static constexpr float GLONASS_HORIZONTAL_ACCURACY_METERS = 2.6;
  static constexpr float BEIDOU_HORIZONTAL_ACCURACY_METERS = 10.2;

  bool init() {
    LOGEVENT("Intializing SL871 rxPin=%d, txPin=%d.", sl871RxPin, sl871TxPin);
    uart.end();
    uart.setRx(sl871TxPin);
    uart.setTx(sl871RxPin);
    uart.begin(DEFAULT_BAUDRATE);
    pinMode(resetPin, INPUT_FLOATING);
    reset();
    LOGEVENT("SL871 initialized.");
    return true;
  }

  bool reset();

  HardwareSerial &uart;

private:
  static constexpr unsigned long DEFAULT_BAUDRATE = 9600;

  const uint32_t sl871RxPin = NOPIN;
  const uint32_t sl871TxPin = NOPIN;
  const uint32_t resetPin = NOPIN;
  const uint32_t ppsPin = NOPIN;
  const uint32_t forceOnNPin = NOPIN;
};

#endif // _SL871_H_