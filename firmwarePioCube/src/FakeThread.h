#ifndef _FAKE_THREAD_H_
#define _FAKE_THREAD_H_

#include "utils/log.h"
#include <stdint.h>

class FakeThread {
public:
  FakeThread(uint32_t loopDelay) : LOOP_DELAY_MS(loopDelay) {}
  ~FakeThread() {}

  virtual bool init() = 0;
  virtual bool loopHook() = 0;
  bool loop();

private:
  uint32_t lastLoop = 0;
  const uint32_t LOOP_DELAY_MS;
};

#endif // _FAKE_THREAD_H_