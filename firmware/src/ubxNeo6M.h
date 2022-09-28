/**
 * ubxNeo6M.h
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#ifndef SRC_UBXNEO6M_H
#define SRC_UBXNEO6M_H
#include "3rd_party/TinyGPSPlus.h"
#include "logging/log.h"
#include "uartBase.h"
#include "zephyr.h"
#include <string.h>

class ubxNeo6M {
public:
  ubxNeo6M(uartBase &_uart);
  ~ubxNeo6M();
  bool readIn();
  TinyGPSLocation getLocation() { return gpsEncoder.location; }
  TinyGPSDate getDate() { return gpsEncoder.date; }
  TinyGPSTime getTime() { return gpsEncoder.time; }
  TinyGPSSpeed getSpeed() { return gpsEncoder.speed; }
  TinyGPSCourse getCourse() { return gpsEncoder.course; }
  TinyGPSAltitude getAltitude() { return gpsEncoder.altitude; }
  TinyGPSInteger getSatellites() { return gpsEncoder.satellites; }
  TinyGPSHDOP getHdop() { return gpsEncoder.hdop; }
  uint32_t charsProcessed()   const { return gpsEncoder.charsProcessed(); }
  uint32_t sentencesWithFix() const { return gpsEncoder.sentencesWithFix(); }
  uint32_t failedChecksum()   const { return gpsEncoder.failedChecksum(); }
  uint32_t passedChecksum()   const { return gpsEncoder.passedChecksum(); }

private:
  TinyGPSPlus gpsEncoder;
  uartBase uartPort;
  static const uint32_t rxBufferLen = 512;
  char rxBuffer[rxBufferLen] = {0};
  uint32_t rxBufferI = 0;
};

#endif // SRC_UBXNEO6M_H
