/**
 * gpsDatagram.h
 *
 * Author: David Antaki
 * Date: 10/5/2022
 */

#ifndef EXPORT_GPS_DATAGRAM_H
#define EXPORT_GPS_DATAGRAM_H

#include "3rd_party/TinyGPSPlus.h"

struct gpsDatagram {
  TinyGPSLocation location;
  TinyGPSDate date;
  TinyGPSTime time;
  TinyGPSSpeed speed;
  TinyGPSCourse course;
  TinyGPSAltitude altitude;
  TinyGPSInteger satellites;
  TinyGPSHDOP hdop;
};
static const uint32_t gpsDatagramSize = sizeof(gpsDatagram);

#endif // EXPORT_GPS_DATAGRAM_H