/**
 * gpsManager.h
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#ifndef SRC_GPSMANAGER_H
#define SRC_GPSMANAGER_H
#include "uartBase.h"
#include "ubxNeo6M.h"
#include <device.h>
#include <zephyr.h>

class gpsManager {
public:
  gpsManager(uartBase _uart);
  ~gpsManager();
  void create();
  void start();
  static const uint32_t kStackSize = 500; // in bytes
private:
  static const uint32_t kThreadPriority = 2;
  static struct k_thread gps_manager_thread_data;
  static k_tid_t kThreadId;
  static const uint32_t loopTimeMs = 1;
  ubxNeo6M gps;

  void loopHook();
  static void entryPoint(void *, void *, void *);
};

#endif // SRC_GPSMANAGER_H
