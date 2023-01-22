/*
 * I2cBase.h
 *
 *  Created on: 1-/24/2022
 *      Author: David Antaki
 */

#ifndef SRC_MANTAHW_I2CBASE_H_
#define SRC_MANTAHW_I2CBASE_H_

#include "Arduino.h"
#include "Wire.h"
#include "MantaCommon/src/utils/log.h"

#define Sleep(x) delay(x)

class I2cBase {
 public:
  I2cBase(TwoWire &_port, uint32_t busFreq)
      : initialized(false),
        port(_port),
        timeout(100),
        sclkFreq(busFreq) {
  }

  virtual ~I2cBase() {
  }

  bool initialize();
  bool deinitialize();


  // write data to an IIC device that has internal addresses
  virtual bool write(uint32_t IICAddress, uint32_t regAddress,
                     uint32_t addrBytes, const uint8_t* pData, size_t dataSize,
                     uint32_t maxWait = 0);

  // read data from an IIC device that has internal addresses
  // will return 0 (no characters) if an error occured
  // otherwise returns number of chars
  virtual uint32_t read(uint32_t IICAddress, uint32_t regAddress,
                        uint32_t addrBytes, uint8_t* pBuffer, size_t xferSize,
                        uint32_t maxWait = 0, bool useRepeatedStart = false, bool isBlockRead = false);

  virtual bool hwReset(bool verbose = true) {
    uint8_t err = port.endTransmission(true); // Release bus
    port.clearWriteError();
    if (err != 0){
      LOGERROR("I2C reset failed with Wire.endTransmission() err:%d", err);
      return false;
    }
    return true;
  }

  float getUSecPerByte() const {
    return uSecPerByte;
  }

  void setUSecPerByte(float uSec);

  // virtual const char* getName();

 protected:
 bool initialized;
  TwoWire &port;
  static const uint32_t kBUFFERSIZE = 256;
  // Bits per byte ssumes worst case I2c 8 data, 1 start, 1 ack, 1 stop
  // a long stream of data would be 9 (just the ack/nak) with start and stop at ends but most of our
  // transactions are short
  static constexpr float kBitsPerBYTE = 11.0f;
  static const uint32_t kMinWaitTime = 5;   // must be >=2 (freeRTOS rounds wait times down)
  static const uint32_t kWriteTime = 10;
  static const uint32_t kReadTime = 10;
  uint32_t timeout;  // used for both mutex (& I2C bus timeout for ps version of I2C)
  float uSecPerByte;  // used to calculate read timeouts for fpga version of I2C

  uint32_t sclkFreq;  // save for re-init
  volatile uint32_t maxWriteMicros;
  volatile uint32_t maxReadMicros;

  virtual uint32_t getMaxTransferSize() {
    static const uint32_t kMAXTRANSFERSIZE = kBUFFERSIZE - 4;
    return (kMAXTRANSFERSIZE);
  }

  I2cBase(I2cBase const &) = delete;
  void operator=(I2cBase const &x) = delete;

  uint32_t calcMaxTimeoutMillis() {
    return kMinWaitTime
        + std::lround((getMaxTransferSize() * uSecPerByte) / 1000.0f);  // ~11 bit times per byte
  }

};

#endif /* SRC_MANTAHW_I2CBASE_H_ */
