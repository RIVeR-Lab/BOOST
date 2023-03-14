#ifndef _IMU_H_
#define _IMU_H_

#include "Encoder.h"
#include "FakeThread.h"
#include "utils/log.h"

class OdometryManager : public FakeThread {
public:
  OdometryManager(Encoder left, Encoder right)
      : FakeThread(LOOP_DELAY_MS), leftEncoder(left), rightEncoder(right) {}

  bool init() {
    bool success = true;
    LOGEVENT("Encoder initiated SUCCESSFULLY");
    return success;
  }

  bool loopHook() override;

private:
  static constexpr uint32_t LOOP_DELAY_MS = 500;
  Encoder leftEncoder;
  Encoder rightEncoder;
};

#endif // _IMU_H_