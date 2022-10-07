/**
 * ubxNeo6M.h
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#ifndef SRC_UBXNEO6M_H
#define SRC_UBXNEO6M_H
#include "3rd_party/TinyGPSPlus.h"
#include "export/gpsDatagram.h"
#include "logging/log.h"
#include "uartBase.h"
#include "zephyr.h"
#include <string.h>
#include <stdio.h>


class ubxNeo6M {
public:
  ubxNeo6M(uartBase &_uart);
  ~ubxNeo6M();
  bool initialize();
  int readIn();
  TinyGPSLocation getLocation() { return gpsEncoder.location; }
  TinyGPSDate getDate() { return gpsEncoder.date; }
  TinyGPSTime getTime() { return gpsEncoder.time; }
  TinyGPSSpeed getSpeed() { return gpsEncoder.speed; }
  TinyGPSCourse getCourse() { return gpsEncoder.course; }
  TinyGPSAltitude getAltitude() { return gpsEncoder.altitude; }
  TinyGPSInteger getSatellites() { return gpsEncoder.satellites; }
  TinyGPSHDOP getHdop() { return gpsEncoder.hdop; }
  uint32_t charsProcessed() const { return gpsEncoder.charsProcessed(); }
  uint32_t sentencesWithFix() const { return gpsEncoder.sentencesWithFix(); }
  uint32_t failedChecksum() const { return gpsEncoder.failedChecksum(); }
  uint32_t passedChecksum() const { return gpsEncoder.passedChecksum(); }

  const gpsDatagram *getLastGoodReading() { return lastGoodReading; }
  bool checkProbOfChecksumFail();

    static void printFloat(float val, bool valid, int len, int prec) {
    if (!valid) {
      while (len-- > 1)
        printk("*");
      printk(" ");
    } else {
      // char sz[32] = "*****************";
      // sprintf(sz, "%f", val);
      printk("%.5f", val);
      int vi = abs((int)val);
      int flen = prec + (val < 0.0 ? 2 : 1); // . and -
      flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
      for (int i = flen; i < len; ++i)
        printk(" ");
    }
  }

  static void printInt(unsigned long val, bool valid, int len) {
    char sz[32] = "*****************";
    if (valid)
      sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i = strlen(sz); i < len; ++i)
      sz[i] = ' ';
    if (len > 0)
      sz[len - 1] = ' ';
    printk("%s", sz);
  }

  static void printDateTime(TinyGPSDate d, TinyGPSTime t) {
    if (!d.isValid()) {
      printk("********** ");
    } else {
      char sz[32];
      sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
      printk("%s", sz);
    }

    if (!t.isValid()) {
      printk("******** ");
    } else {
      char sz[32];
      sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
      printk("%s", sz);
    }
  }

  static void printStr(bool valid, const char *str) {
    if (valid) {
      printk("%s", str);
    } else {
      printk("***");
    }
    // int slen = strlen(str);
    // for (int i=0; i<len; ++i)
    //   if(i<slen){
    //     printk("%c", str[i]);
    //   }else{
    //     printk(" ");
    //   }
  }

private:
  static const bool USE_INTERRUPT_UART = false;
  TinyGPSPlus gpsEncoder;
  uartBase uartPort;
  static const uint32_t rxBufferLen = 512;
  char rxBuffer[rxBufferLen] = {0};
  uint32_t rxBufferI = 0;
  uint32_t lastPassedChecksumCount = 0;
  gpsDatagram *lastGoodReading;
   // The max allowable probability of checksum's failing before we log error.
   // There is 1 checksums per GPS sentence.
  static constexpr double maxProbOfChecksumFail = 0.05;
  static const int64_t checkProbOfChecksumFailRateMs = 2000;
  int64_t lastCheckProbOfChecksumFailMs = 0;
};

#endif // SRC_UBXNEO6M_H
