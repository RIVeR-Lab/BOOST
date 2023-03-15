/**
 * GpsDatagram.h
 *
 * Author: David Antaki
 * Date: 10/5/2022
 */

#ifndef EXPORT_GPS_DATAGRAM_H
#define EXPORT_GPS_DATAGRAM_H

#include "3rd_party/TinyGPS/TinyGPSPlus.h"

struct GpsDatagram {
  GpsDatagram() {}
  GpsDatagram(TinyGPSLocation _location, TinyGPSDate _date, TinyGPSTime _time,
              TinyGPSSpeed _speed, TinyGPSCourse _course,
              TinyGPSAltitude _altitude, TinyGPSInteger _satellites,
              TinyGPSHDOP _hdop, uint32_t _charsProcessed,
              uint32_t _sentencesWithFix, uint32_t _failedChecksum,
              uint32_t _passedChecksum)
      : location(_location), date(_date), time(_time), speed(_speed),
        course(_course), altitude(_altitude), satellites(_satellites),
        hdop(_hdop), charsProcessed(_charsProcessed),
        sentencesWithFix(_sentencesWithFix), failedChecksum(_failedChecksum),
        passedChecksum(_passedChecksum) {}
  TinyGPSLocation location;
  TinyGPSDate date;
  TinyGPSTime time;
  TinyGPSSpeed speed;
  TinyGPSCourse course;
  TinyGPSAltitude altitude;
  TinyGPSInteger satellites;
  TinyGPSHDOP hdop;
  uint32_t charsProcessed;
  uint32_t sentencesWithFix;
  uint32_t failedChecksum;
  uint32_t passedChecksum;
};
static const uint32_t gpsDatagramSize = sizeof(GpsDatagram);

#endif // EXPORT_GPS_DATAGRAM_H