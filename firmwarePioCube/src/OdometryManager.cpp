#include "OdometryManager.h"
#include "RealMain.h"

bool OdometryManager::loopHook() {
  LOGEVENT("EncoderManager::loopHook()");

  return true;
}
