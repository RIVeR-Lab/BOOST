/**
 * gpsManager.h
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#ifndef SRC_GPSMANAGER_H
#define SRC_GPSMANAGER_H
#include "3rd_party/TinyGPSPlus.h"
#include "export/gpsDatagram.h"
#include "uartBase.h"
#include "ubxNeo6M.h"
#include <device.h>
#include <zephyr.h>

class gpsManager {
public:
  gpsManager(uartBase _uart);
  ~gpsManager();
  void create();
  bool initialize();
  void start();
  static const uint32_t kStackSize = 1024; // in bytes

  bool getGpsData(gpsDatagram &outData);

private:
  static const uint32_t kThreadPriority = 3;
  static struct k_thread gps_manager_thread_data;
  static k_tid_t kThreadId;
  static const uint32_t loopTimeMs = 100;

  // All possible messages that can be sent to this thread's mailbox.
  enum message_t : uint32_t { UNKNOWN_MSG, GET_GPS_DATAGRAM };

  struct k_mbox mailbox;

  ubxNeo6M gps;

  void loopHook();
  bool processMbx();
  static void entryPoint(void *, void *, void *);
};

#endif // SRC_GPSMANAGER_H
