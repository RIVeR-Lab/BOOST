#ifndef _IMU_H_
#define _IMU_H_

#include "Encoder.h"
#include "FakeThread.h"
#include "TXB0104PWR.h"
#include "utils/log.h"
#include "utils/macros.h"

class OdometryManager : public FakeThread {
public:
  OdometryManager(Encoder &left, Encoder &right, TXB0104PWR &_lvlShifter)
      : FakeThread(LOOP_DELAY_MS, LOG_LOOP_DELAY_MS), leftQuadEnc(left), rightQuadEnc(right),
        lvlShifter(_lvlShifter) {}

  bool init() {
    INIT_HEADER
    // Enable encoder level shifter.
    lvlShifter.init();
    lvlShifter.enable();
    INIT_FOOTER
  }

  bool loopHook() override;
  bool logLoopHook() override;
  

private:
  static constexpr uint32_t LOOP_DELAY_MS = 500;
  static constexpr uint32_t LOG_LOOP_DELAY_MS = LOOP_DELAY_MS;
  Encoder &leftQuadEnc;
  Encoder &rightQuadEnc;
  TXB0104PWR &lvlShifter;

  void printOdomData();
};

#endif // _IMU_H_