#include "OdometryManager.h"
#include "RealMain.h"

bool OdometryManager::loopHook() {
  LOGDEBUG("%s", __func__);
  return true;
}

bool OdometryManager::logLoopHook() {
#if PRINT_ODOMETRY_DATA
  LOGEVENT("%s", __func__);
  printOdomData();
#endif
  return true;
}

void OdometryManager::printOdomData() {
  int32_t leftPos = leftQuadEnc.read();
  int32_t rightPos = rightQuadEnc.read();
  LOGEVENT("LeftEnc: %d, RightEnc: %d", leftPos, rightPos);
}
