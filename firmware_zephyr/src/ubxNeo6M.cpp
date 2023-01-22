
/**
 * ubxNeo6M.cpp
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#include "ubxNeo6M.h"
LOG_MODULE_DECLARE(gps, LOG_LEVEL_DBG);

ubxNeo6M::ubxNeo6M(uartBase &_uart) : uartPort(_uart) {}
ubxNeo6M::~ubxNeo6M() {}

bool ubxNeo6M::initialize() {
  bool success = true;
  if (USE_INTERRUPT_UART) {
    success = success && uartPort.initializeIrq();
  }

  return success;
}

// Probability of Checksum fail=failedChecksums/passedChecksums.
// There is 1 checksums per GPS sentence.
// Return true if the ratio of P(checksumFail)>0.05
bool ubxNeo6M::checkProbOfChecksumFail(){
  // Only check every set period.
  if((k_uptime_get() - lastCheckProbOfChecksumFailMs)> checkProbOfChecksumFailRateMs){
    lastCheckProbOfChecksumFailMs = k_uptime_get();
    double probOfChecksumFail = static_cast<double>(gpsEncoder.failedChecksum())/static_cast<double>(gpsEncoder.passedChecksum());
    if(probOfChecksumFail > maxProbOfChecksumFail){
      LOG_ERR("The ratio of GPS sentence failedChecksums:passedChecksums=%.2f>%.2f", probOfChecksumFail*100, maxProbOfChecksumFail*100);
      return true;
    }
  }
  return false;
}

/**
 * Return the following
 * 0 – If a character arrived.
 * -1 – If no character was available to read (i.e. the UART input buffer was
 * empty). -ENOSYS – If the operation is not implemented. -EBUSY – If async
 * reception was enabled using uart_rx_enable
 */
int ubxNeo6M::readIn() {
  unsigned char inChar;

  int stat = uartPort.poll_read(inChar);
  // If char arrived, add to buffer and keep reading until no more characters to
  // read.
  while (stat == 0) {
    stat = uartPort.poll_read(inChar);
    // Add character to the encoder
    gpsEncoder.encode(inChar);
  }

  // Update the last good GPS reading if the checksum incremented
  if (lastPassedChecksumCount < gpsEncoder.passedChecksum()) {
    lastPassedChecksumCount = gpsEncoder.passedChecksum();
    const gpsDatagram newLastGoodReading = {
        gpsEncoder.location,       gpsEncoder.date,
        gpsEncoder.time,           gpsEncoder.speed,
        gpsEncoder.course,         gpsEncoder.altitude,
        gpsEncoder.satellites,     gpsEncoder.hdop,
        gpsEncoder.charsProcessed(), gpsEncoder.sentencesWithFix(),
        gpsEncoder.failedChecksum(), gpsEncoder.passedChecksum()};
    memcpy(lastGoodReading, &newLastGoodReading, sizeof(gpsDatagram));
  }

  return stat;
}