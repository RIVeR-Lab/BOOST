#include "FakeThread.h"

bool FakeThread::loop() {
  uint32_t now = HAL_GetTick();
  if (now - lastLoop > LOOP_DELAY_MS) {
    lastLoop = now;
    // LOGINFO("FakeThread::loop()");
    return loopHook();
  }
  return true;
}