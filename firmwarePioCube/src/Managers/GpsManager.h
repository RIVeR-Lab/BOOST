#ifndef _GPSMANAGER_H_
#define _GPSMANAGER_H_

#include "3rd_party/TinyGPS/TinyGPSPlus.h"
#include "Arduino.h"
#include "FakeThread.h"
#include "SL871.h"
#include "config.h"
#include "data/GpsDatagram.h"
#include "data/RingBuffer.h"
#include "sensor_msgs/NavSatFix.h"
#include "utils.h"
#include "utils/log.h"
#include <ros/time.h>
#include "utils/macros.h"

#define GPS_BUF_SIZE 512

class RealMain;

class GpsManager : public FakeThread {
public:
  GpsManager(SL871 &_gps)
      : FakeThread(LOOP_DELAY_MS, LOG_LOOP_DELAY_MS), gps(_gps) {}
  ~GpsManager() {}

  bool init() override;
  GpsDatagram getLastGoodGpsReading() { return lastGoodGpsReading; }
  sensor_msgs::NavSatFix toRosMsg();
  bool isGpsFixed() {
    return lastGoodGpsReading.satellites.value() >= MIN_NUM_SATS_FOR_FIX;
  }

private:
  static constexpr uint32_t LOOP_DELAY_MS = 500;
  static constexpr uint32_t LOG_LOOP_DELAY_MS = 1000;

  static constexpr uint32_t MIN_NUM_SATS_FOR_FIX = 4;

  SL871 &gps;
  RingBuffer<char, GPS_BUF_SIZE> gpsDataBuffer;

  bool loopHook() override;
  bool logLoopHook() override;
  bool readInGpsData();
  bool processGpsData();
  bool checkLastGoodGpsReading();
  std::array<float, 9>
  positionCovarianceDiag(float hdop, float idealHorizontalMeasurementError);

  GpsDatagram lastGoodGpsReading;
  TinyGPSPlus gpsDataNMEADecoder;
  // Statistics
  uint32_t lastGoodGpsReadingTime = 0;
  uint32_t lastPassedChecksumCount = 0;
  uint32_t lastFailedChecksumCount = 0;
  static constexpr uint32_t GPS_MAX_AGE_MS = 30000;
};

#endif // _GPSMANAGER_H_