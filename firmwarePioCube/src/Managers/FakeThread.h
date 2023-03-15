#ifndef _FAKE_THREAD_H_
#define _FAKE_THREAD_H_

#include "utils/log.h"
#include <stdint.h>

class FakeThread {
public:
  FakeThread(uint32_t loopDelay, uint32_t logLoopDelay)
      : LOOP_DELAY_MS(loopDelay), LOG_LOOP_DELAY_MS(logLoopDelay) {}
  ~FakeThread() {}

  virtual bool init() = 0;
  bool loop();

private:
  bool logLoop();
  virtual bool loopHook() = 0;
  virtual bool logLoopHook() = 0;
  uint32_t lastLoop = 0;
  uint32_t lastLogLoop = 0;
  const uint32_t LOOP_DELAY_MS;
  const uint32_t LOG_LOOP_DELAY_MS;
};

#endif // _FAKE_THREAD_H_