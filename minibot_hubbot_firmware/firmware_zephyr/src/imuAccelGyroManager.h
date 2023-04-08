/**
 * imuAccelGyroManager.h
 *
 * Author: David Antaki
 * Date: 1/21/2022
 */

#ifndef SRC_IMUACCELGYROMANAGER_H
#define SRC_IMUACCELGYROMANAGER_H
#include <device.h>
#include <zephyr.h>
#include "logging/log.h"
#include "drivers/i2c.h"

class imuAccelGyroManager {
public:
  imuAccelGyroManager(const struct device &_i2cDev);
  ~imuAccelGyroManager();
  void create();
  bool initialize();
  void start();
  static const uint32_t kStackSize = 1024; // in bytes

private:
  static const uint32_t kThreadPriority = 3;
  static struct k_thread imuAccelGyroManager_thread_data;
  static k_tid_t kThreadId;
  static const uint32_t loopTimeMs = 100;

  const struct device &i2cDev;

//   Adafruit_BNO055 bno055;

  // All possible messages that can be sent to this thread's mailbox.
  enum message_t : uint32_t { UNKNOWN_MSG, GET_GPS_DATAGRAM };

  struct k_mbox mailbox;

  void loopHook();
  bool processMbx();
  static void entryPoint(void *, void *, void *);

  void getImuData(){
    uint8_t buf[256]{};
    int ret = i2c_read(&i2cDev, buf, 1, 1);

  }
};

#endif // SRC_IMUACCELGYROMANAGER_H
