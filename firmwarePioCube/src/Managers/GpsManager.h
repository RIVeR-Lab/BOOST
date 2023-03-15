#ifndef _GPSMANAGER_H_
#define _GPSMANAGER_H_

#include "Arduino.h"
#include "FakeThread.h"
#include "utils.h"
#include "utils/log.h"
#include "SL871.h"

class GpsManager : public FakeThread {
public:
  GpsManager(SL871 &_gps)
      : FakeThread(LOOP_DELAY_MS, LOG_LOOP_DELAY_MS), gps(_gps) {}
  ~GpsManager() {}

  bool init() override;

private:
  SL871 &gps;
  // We don't need to loop fast because we set motor speed when a new twist
  // message comes in.
  static constexpr uint32_t LOOP_DELAY_MS = 1;
  static constexpr uint32_t LOG_LOOP_DELAY_MS = LOOP_DELAY_MS;

  bool loopHook() override;
  bool logLoopHook() override;

};

#endif // _GPSMANAGER_H_