#include "FakeThread.h"

bool FakeThread::loop() {
  uint32_t now = HAL_GetTick();
  if (now - lastLoop > LOOP_DELAY_MS) {
    lastLoop = now;
    return loopHook();
  }
  if (now - lastLogLoop > LOG_LOOP_DELAY_MS) {
    lastLogLoop = now;
    return logLoopHook();
  }
  return true;
}