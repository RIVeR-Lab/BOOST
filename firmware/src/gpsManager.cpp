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
    /* Allow other thread/workqueue to work. */
    // Yield to other theads with same or higher priority.
    k_yield();

    // Print GPS data:
    if(k_uptime_get() - lastPrintMs > 3000){
      lastPrintMs = k_uptime_get();
      // TODO: Change to use LOGGING instead
      printk("Sats  Latitude   Longitude   Fix      Date    Time     Date  Alt    Course Speed Card  Chars Sentences Checksum\r\n");
      printk("       (deg)      (deg)      Age                       Age   (m)    --- from GPS ----   RX    RX        Fail\r\n");
      printk("----------------------------------------------------------------------------------------------------------------------------------------\r\n");
      printInt(gps.getSatellites().value(), gps.getSatellites().isValid(), 5);
      // printFloat(gps.getHdop().hdop(), gps.getHdop().isValid(), 6, 1);
      printFloat(gps.getLocation().lat(), gps.getLocation().isValid(), 11, 6);
      printFloat(gps.getLocation().lng(), gps.getLocation().isValid(), 12, 6);      
      printInt(gps.getLocation().age(), gps.getLocation().isValid(), 5);
      printDateTime(gps.getDate(), gps.getTime());
      printFloat(gps.getAltitude().meters(), gps.getAltitude().isValid(), 7, 2);
      printFloat(gps.getCourse().deg(), gps.getCourse().isValid(), 7, 2);
      printFloat(gps.getSpeed().kmph(), gps.getSpeed().isValid(), 6, 2);
      printStr(gps.getCourse().isValid() ? TinyGPSPlus::cardinal(gps.getCourse().deg()) : "*** ", 6);
      printk("        ");
      printInt(gps.charsProcessed(), true, 6);
      printk("  ");
      printInt(gps.sentencesWithFix(), true, 10);
      printk("  ");
      printInt(gps.failedChecksum(), true, 9);
      printk("\r\n\r\n");
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
  // TODO: figure out how to put a thread name in as string so that it 
  // comes up in thread_analyze().
  kThreadId = k_thread_create(&gps_manager_thread_data, gps_manager_stack,
                              K_THREAD_STACK_SIZEOF(gps_manager_stack),
                              gpsManager::entryPoint, (gpsManager *)this, NULL,
                              NULL, kThreadPriority, 0, K_FOREVER);
}

bool gpsManager::initialize() {
  bool success = true;
  success = success && gps.initialize();

  if(!success){
    LOG_ERR("Failed to initialize gpsManager.");
  }else{
    LOG_INF("Initialized gpsManager successfully.");
  }
  return success;
}

// Starts this thread
void gpsManager::start() { k_thread_start(kThreadId); }
