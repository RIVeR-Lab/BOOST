/**
 * gpsManager.cpp
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#include "gpsManager.h"
LOG_MODULE_REGISTER(gps, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(gps_manager_stack, gpsManager::kStackSize);

gpsManager::gpsManager(uartBase _uart) : gps(_uart) {}

gpsManager::~gpsManager() {}

// The entry point and/or "main" function of this thread
void gpsManager::loopHook() {
  int64_t lastPrintMs = k_uptime_get();
  while (1) {
    gps.readIn();
    k_sleep(K_MSEC(loopTimeMs));
    // Print GPS data:
    if(k_uptime_get() - lastPrintMs > 1000){
      lastPrintMs = k_uptime_get();
      char buffer [100];
      printk("DateTime: ");
      printDateTime(gps.getDate(), gps.getTime());
      printk("Lat(deg): ");
      printFloat(gps.getLocation().lat(), gps.getLocation().isValid(), 11, 6);
      printk("Lng(deg): ");
      printFloat(gps.getLocation().lng(), gps.getLocation().isValid(), 12, 6);
    }
  }
}

void gpsManager::entryPoint(void *thisThread, void *, void *) {
  gpsManager *pThis = (gpsManager *)thisThread;
  pThis->loopHook();
}

k_tid_t gpsManager::kThreadId;
struct k_thread gpsManager::gps_manager_thread_data;

// Creates this thread
void gpsManager::create() {
  // We pass a pointer to "this" class instance as a parameter because
  // entryPoint() must be static. Then we can call the passed class instance's
  // loopHook() from within entryPoint().
  kThreadId = k_thread_create(&gps_manager_thread_data, gps_manager_stack,
                              K_THREAD_STACK_SIZEOF(gps_manager_stack),
                              gpsManager::entryPoint, (gpsManager *)this, NULL,
                              NULL, kThreadPriority, 0, K_FOREVER);
}

// Starts this thread
void gpsManager::start() { k_thread_start(kThreadId); }
