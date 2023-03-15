#include "GpsManager.h"

bool GpsManager::init() {
  bool success = true;
  success = success && gps.init();
  return success;
}

bool GpsManager::loopHook() {
  // Read latest GPS data
  uint8_t data[256]{};
  gps.recv(data, 256);
  char buf[256];
  for (int i = 0; i < 256; i++) {
    sprintf(buf, "%c", data[i]);
  }
  LOGEVENT(buf);
  return true;
}

bool GpsManager::logLoopHook() {
  char buf[256];
  LOGEVENT(buf);

  return true;
}
