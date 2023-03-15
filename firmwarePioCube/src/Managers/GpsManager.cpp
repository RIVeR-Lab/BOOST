#include "GpsManager.h"
#include "RealMain.h"

bool GpsManager::init() {
  bool success = true;
  success = success && gps.init();
  return success;
}

bool GpsManager::loopHook() {
  bool success = true;
  success = success && readInGpsData();
  success = success && processGpsData();

  // Check if we lost GPA signal/comms and try to re-init it.
  if(!checkLastGoodGpsReading()) {
    gps.init();
  }
  return true;
}

bool GpsManager::logLoopHook() {
#if ENABLE_GPS_LOGLOOP
  LOGINFO("Current GPS location: lat:%f:lng:%f:age:%d:isValid:%d:isUpdated:%d",
          lastGoodGpsReading.location.lat(), lastGoodGpsReading.location.lng(),
          lastGoodGpsReading.location.age(),
          lastGoodGpsReading.location.isValid(),
          lastGoodGpsReading.location.isUpdated());
  LOGINFO("Current GPS hdop: hdop:%f:age:%d", lastGoodGpsReading.hdop.hdop(),
          lastGoodGpsReading.hdop.age());
  LOGINFO("Current GPS date: yr:%d:mo:%d:day:%d:age:%d",
          lastGoodGpsReading.date.year(), lastGoodGpsReading.date.month(),
          lastGoodGpsReading.date.day(), lastGoodGpsReading.date.age());
  LOGINFO("Current GPS time: hr:%d:min:%d:sec:%d:age:%d",
          lastGoodGpsReading.time.hour(), lastGoodGpsReading.time.minute(),
          lastGoodGpsReading.time.second(), lastGoodGpsReading.time.age());
  LOGINFO("Current GPS speed: mph:%f:mps:%f:knots:%f:kmph:%f:age:%d",
          lastGoodGpsReading.speed.mph(), lastGoodGpsReading.speed.mps(),
          lastGoodGpsReading.speed.knots(), lastGoodGpsReading.speed.kmph(),
          lastGoodGpsReading.speed.age());
  LOGINFO("Current GPS course: deg:%f:age:%d");
  LOGINFO(
      "Current GPS altitude: meters:%f:mi:%f:km:%f:ft:%f:age:%d",
      lastGoodGpsReading.altitude.meters(), lastGoodGpsReading.altitude.miles(),
      lastGoodGpsReading.altitude.kilometers(),
      lastGoodGpsReading.altitude.feet(), lastGoodGpsReading.altitude.age());
  LOGINFO("Current GPS satellites: sat:%d:age:%d",
          lastGoodGpsReading.satellites.value(),
          lastGoodGpsReading.satellites.age());
  LOGINFO("Current GPS status: "
          "charsProcesses:%d:sentencesWithFix:%d:failedChecksum:%d:"
          "passedChecksum:%d",
          lastGoodGpsReading.charsProcessed,
          lastGoodGpsReading.sentencesWithFix,
          lastGoodGpsReading.failedChecksum, lastGoodGpsReading.passedChecksum);
#endif
  return true;
}

// Checks if the time the GPS was last updated is within range.
bool GpsManager::checkLastGoodGpsReading() {
  bool success = true;
  if (millis() - lastGoodGpsReadingTime > GPS_MAX_AGE_MS) {
    LOGERROR("!!!!GPS DATA IS TOO OLD!!!!!");
    success = false;
  }
  return success;
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
 * @brief Process the GPS data in the internal buffer.
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

  // Check if the failed checksum incremented
  if (lastFailedChecksumCount < gpsDataNMEADecoder.failedChecksum()) {
    LOGERROR("GPS Checksum Failed!");
  }

  // Update the last good GPS reading if the checksum incremented
  if (lastPassedChecksumCount < gpsDataNMEADecoder.passedChecksum()) {
    LOGEVENT("New GPS Data Received.");

    lastGoodGpsReadingTime = millis();

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

  lastPassedChecksumCount = gpsDataNMEADecoder.passedChecksum();
  lastFailedChecksumCount = gpsDataNMEADecoder.failedChecksum();

  return success;
}