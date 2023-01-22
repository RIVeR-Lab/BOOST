/*
 * Logging.h
 *
 *  Created on: Apr 18, 2019
 *      Author: whzen
 */

#ifndef SRC_BOARD2BOARD_LOGGING_H_
#define SRC_BOARD2BOARD_LOGGING_H_
// #include "Manta.h"
// #include <Board2Board/B2BDataSendInterface.h>
// #include "utils/LogDelta.h"
#include <array>
#include <MantaExport/LogSpecs.h>
// #include "utils/FRTimer.h"

class B2BDataSendInterface;

class Logging {
public:
  Logging(void *con, bool udpOn = true, bool remoteOn = false) {}

  // provide a way to globally request everyone to log
  class LogSignal {
  public:
    LogSignal() {}

  private:
    static volatile bool logNow;
    bool logDone = false;
  };

  // combine the delta checking & logging
  template <class T> class DeltaLogger {
  public:
    DeltaLogger(void *sub, Logging &log, const T &init) {}

    DeltaLogger(void *sub, Logging &log, float init) {}

  private:
    Logging &logger;
  };

  // combine the delta checking & logging for case where 'value' is array of
  // pointers
  template <class T, size_t S> class DeltaLoggerPtr {
  public:
    DeltaLoggerPtr(Manta::LogSpecs::Subject sub, Logging &log, float init) {}

    DeltaLoggerPtr(Manta::LogSpecs::Subject sub, Logging &log, std::array<T, S> init) {}

    bool sendData(bool force = false) { return true; }
    bool setValPtr(size_t idx, const T &val) { return true; }

    bool setEpsilon(const T &val) { return true; }

    // return true if it is time to log (honors global request)
    bool logTime() { return true; }

    void setTimeIncrement(uint32_t inc) {}
  };
};

#endif /* SRC_BOARD2BOARD_LOGGING_H_ */
