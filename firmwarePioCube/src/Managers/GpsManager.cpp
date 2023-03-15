#include "GpsManager.h"

bool GpsManager::init() {
  bool success = true;
  success = success && gps.init();
  return success;
}

bool GpsManager::loopHook() {
  bool success = true;
  success = success && readInGpsData();
  success = success && processGpsData();
  return true;
}

bool GpsManager::logLoopHook() {
  // char buf[256];
  // LOGEVENT(buf);

  return true;
}

bool GpsManager::readInGpsData() {
  bool success = true;

  int bytes = gps.uart.available();
  while (bytes > 0) {
    if (bytes >= 255) {
      LOGERROR("UART3 BUFFER IS FULL! DATA MAY HAVE BEEN LOST!!!");
    }
    char tempInByte = (char)gps.uart.read();
    gpsDataBuffer.push(tempInByte);
    bytes--;
  }
  return success;
}

/**
 */
bool GpsManager::processGpsData() {
  bool success = true;
  // Add character to the decoder
  while (!gpsDataBuffer.isEmpty()) {
    char t{};
    success = success && gpsDataBuffer.pop(t);
#if GPS_DEBUG
    Console.print(t);
#endif
    gpsDataNMEADecoder.encode(t);
  }

  // Update the last good GPS reading if the checksum incremented
  if (lastPassedChecksumCount < gpsDataNMEADecoder.passedChecksum()) {

    // LOGEVENT(
    //     "NEW GPS location: lat:%f:lng:%f:age:%d:isValid:%d:isUpdated:%d",
    //     gpsDataNMEADecoder.location.lat(), gpsDataNMEADecoder.location.lng(),
    //     gpsDataNMEADecoder.location.age(),
    //     gpsDataNMEADecoder.location.isValid(),
    //     gpsDataNMEADecoder.location.isUpdated());
    // LOGEVENT("NEW GPS hdop: hdop:%f:age:%d", gpsDataNMEADecoder.hdop.hdop(),
    //          gpsDataNMEADecoder.hdop.age());

    lastPassedChecksumCount = gpsDataNMEADecoder.passedChecksum();
    const GpsDatagram newLastGoodReading = {
        gpsDataNMEADecoder.location,
        gpsDataNMEADecoder.date,
        gpsDataNMEADecoder.time,
        gpsDataNMEADecoder.speed,
        gpsDataNMEADecoder.course,
        gpsDataNMEADecoder.altitude,
        gpsDataNMEADecoder.satellites,
        gpsDataNMEADecoder.hdop,
        gpsDataNMEADecoder.charsProcessed(),
        gpsDataNMEADecoder.sentencesWithFix(),
        gpsDataNMEADecoder.failedChecksum(),
        gpsDataNMEADecoder.passedChecksum()};
    memcpy(&lastGoodGpsReading, &newLastGoodReading, sizeof(GpsDatagram));
  }

  return success;
}