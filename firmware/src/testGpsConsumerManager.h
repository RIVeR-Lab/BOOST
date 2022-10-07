/**
 * testGpsConsumerManager.h
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#ifndef SRC_TEST_GPS_CONSUMER_MANAGER_H
#define SRC_TEST_GPS_CONSUMER_MANAGER_H
#include "3rd_party/TinyGPSPlus.h"
#include <device.h>
#include <stdio.h>
#include <zephyr.h>

class testGpsConsumerManager {
public:
  testGpsConsumerManager();
  ~testGpsConsumerManager();
  void create();
  bool initialize();
  void start();

  static const uint32_t kStackSize = 1024; // in bytes

private:
  static const uint32_t kThreadPriority = 2;
  static struct k_thread testGpsConsumerManagerData;
  static k_tid_t kThreadId;
  static const uint32_t loopTimeMs = 2000;

  void loopHook();
  static void entryPoint(void *, void *, void *);
};

#endif // SRC_TEST_GPS_CONSUMER_MANAGER_H
