/*
 * I2cBase.h
 *
 *  Created on: 10/24/2022
 *      Author: David Antaki
 */

#include "I2cBase.h"

bool I2cBase::initialize() {
  bool success = true;
  LOGEVENT("Initializing I2C...");
  port.setTimeout(timeout);
  port.setClock(sclkFreq);
  port.begin();
  // success = success && hwReset();

  if (!success) {
    initialized = false;
    LOGERROR("I2C failed to initialize.");
  } else{
    initialized = true;
    LOGEVENT("I2C initialized successfully.");
  }
  return success;
}

bool I2cBase::deinitialize() {
  bool success = true;
  LOGEVENT("De-initializing I2C...");
  port.endTransmission();
  port.end();

  if (!success) {
    initialized = false;
    LOGERROR("I2C failed to de-initialize.");
  } else{
    LOGEVENT("De-initialized I2C successfully.");
  }
  return success;
}

bool I2cBase::write(uint32_t IICAddress, uint32_t regAddress,
                    uint32_t addrBytes, const uint8_t *pData, size_t dataSize,
                    uint32_t maxWait) {
  bool success = true;
  uint8_t WriteBuffer[kBUFFERSIZE]{};

  if (addrBytes > sizeof(regAddress)) {
    LOGERROR(
        "Invalid address byte count device:%d, regaddr:%d addrSize:%d",
        IICAddress, regAddress, addrBytes);
    success = false;
  }
  if (dataSize > getMaxTransferSize()) {
    LOGERROR("Invalid write size device:%d, regaddr:%d dataSize: %d/%d",
                  IICAddress, regAddress, dataSize, getMaxTransferSize());
    success = false;
  }

  if (success) {
    // Add register address bytes.
    size_t writeIndex = 0;
    if (addrBytes > 0) {
      for (int ci = addrBytes - 1; ci >= 0; ci--) {
        WriteBuffer[writeIndex] = (uint8_t)(regAddress >> 8 * ci);
        writeIndex++;
      }
    }
    // Add the data.
    for (size_t ci = 0; ci < dataSize; ci++) {
      WriteBuffer[writeIndex] = pData[ci];
      writeIndex++;
    }

    // Send the buffer.
    uint32_t numchars = 0;
    port.beginTransmission(static_cast<uint8_t>(IICAddress));
    numchars = port.write(WriteBuffer, writeIndex);
    uint8_t err = port.endTransmission(true);

    if (numchars != writeIndex && err != 0) {
      success = false;
      LOGERROR("Write failed I2C addr: %d(0x%x) reg:%d(0x%x) bytesSent: %d "
               "bytesExpectedSent: %d err: %d",
               IICAddress, IICAddress, regAddress, regAddress, numchars,
               writeIndex, err);
      hwReset(false);
    }
  }

  return success;
}

uint32_t I2cBase::read(uint32_t IICAddress, uint32_t regAddress,
                       uint32_t addrBytes, uint8_t *pBuffer, size_t xferSize,
                       uint32_t maxWait, bool useRepeatedStart,
                       bool isBlockRead) {
  int32_t numchars = 0;
  uint8_t WriteBuffer[kBUFFERSIZE]{};

  if (addrBytes > sizeof(regAddress)) {
    LOGERROR("Invalid address byte count device:%d, regaddr:%d addrSize:%d",
             IICAddress, regAddress, addrBytes);
    numchars = -1;
  }
  if (xferSize > getMaxTransferSize() && !isBlockRead) {
    LOGERROR("Invalid non-block read transfer size device:%d, regaddr:%d "
             "dataSize: %d/%d",
             IICAddress, regAddress, xferSize, getMaxTransferSize());
    numchars = -1;
  }
  if (numchars != -1) {
    // write register address with restart enabled, followed by the data
    size_t writeIndex = 0;
    for (int ci = addrBytes - 1; ci >= 0; ci--) {
      WriteBuffer[writeIndex] = (uint8_t)(regAddress >> 8 * ci);
      writeIndex++;
    }
    port.beginTransmission(static_cast<uint8_t>(IICAddress));
    numchars = port.write(WriteBuffer, writeIndex);
    uint8_t err = port.endTransmission(false); // Don't send stop bit

    // Read data back
    if (numchars != 0 || err != 0) {
      port.requestFrom(IICAddress, xferSize, false); // Don't send stop bit
      numchars = port.readBytes(pBuffer, xferSize);
    }

    if (numchars == 0 || err != 0) {
      LOGERROR("Write failed I2C addr: %d(0x%x) reg:%d(0x%x) bytesSent: %d "
               "bytesExpectedSent: %d err: %d",
               IICAddress, IICAddress, regAddress, regAddress, numchars,
               writeIndex, err);
      hwReset(false);
    }
  }
  return numchars;
}

void I2cBase::setUSecPerByte(float uSec) {
  uSecPerByte = uSec;
  timeout = calcMaxTimeoutMillis();
}
