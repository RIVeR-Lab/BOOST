/**
 * testGpsConsumerManager.cpp
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#include "testGpsConsumerManager.h"
#include "mainThread.h"
LOG_MODULE_REGISTER(testGpsConsumerManager, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(testGpsConsumerManagerStack, testGpsConsumerManager::kStackSize);

testGpsConsumerManager::testGpsConsumerManager() {}

testGpsConsumerManager::~testGpsConsumerManager() {}

// The entry point and/or "main" function of this thread
void testGpsConsumerManager::loopHook() {
  int64_t lastPrintMs = k_uptime_get();
  while (1) {
    /* Allow other thread/workqueue to work. */
    k_msleep(loopTimeMs);
    
    // This is simply to test passing data from the the GPS thread to this thread.
    gpsDatagram outData;
    theMainThread.gpsThread.getGpsData(outData);
    
    // Print GPS data:
    if(k_uptime_get() - lastPrintMs > 3000){
      lastPrintMs = k_uptime_get();
      // TODO: Change to use LOGGING instead
      printk("Sats  Latitude   Longitude   Fix      Date    Time     Date  Alt    Course Speed Card  Chars Sentences Checksum\r\n");
      printk("       (deg)      (deg)    Age(ms)                     Age   (m)    --- from GPS ----   RX    RX        Fail\r\n");
      printk("----------------------------------------------------------------------------------------------------------------------------------------\r\n");
      printInt(outData.satellites.value(), outData.satellites.isValid(), 5);
      // printFloat(gps.getHdop().hdop(), gps.getHdop().isValid(), 6, 1);
      // printFloat(gps.getLocation().lat(), gps.getLocation().isValid(), 11, 6);
      // printFloat(gps.getLocation().lng(), gps.getLocation().isValid(), 12, 6);      
      // printInt(gps.getLocation().age(), gps.getLocation().isValid(), 5);
      // printDateTime(gps.getDate(), gps.getTime());
      // printFloat(gps.getAltitude().meters(), gps.getAltitude().isValid(), 7, 2);
      // printFloat(gps.getCourse().deg(), gps.getCourse().isValid(), 7, 2);
      // printFloat(gps.getSpeed().kmph(), gps.getSpeed().isValid(), 6, 2);
      // // const char* deg = TinyGPSPlus::cardinal(gps.getCourse().deg());
      // // printStr(gps.getCourse().isValid(), deg); // This causes fault
      // printk("        ");
      // printInt(gps.charsProcessed(), true, 6);
      // printk("  ");
      // printInt(gps.sentencesWithFix(), true, 10);
      // printk("  ");
      // printInt(gps.failedChecksum(), true, 9);
      // printk("\r\n\r\n");
    }
  }
}

void testGpsConsumerManager::entryPoint(void *thisThread, void *, void *) {
  testGpsConsumerManager *pThis = (testGpsConsumerManager *)thisThread;
  pThis->loopHook();
}

k_tid_t testGpsConsumerManager::kThreadId;
struct k_thread testGpsConsumerManager::testGpsConsumerManagerData;

// Creates this thread
void testGpsConsumerManager::create() {
  // We pass a pointer to "this" class instance as a parameter because
  // entryPoint() must be static. Then we can call the passed class instance's
  // loopHook() from within entryPoint().
  // TODO: figure out how to put a thread name in as string so that it 
  // comes up in thread_analyze().
  kThreadId = k_thread_create(&testGpsConsumerManagerData, testGpsConsumerManagerStack,
                              K_THREAD_STACK_SIZEOF(testGpsConsumerManagerStack),
                              testGpsConsumerManager::entryPoint, (testGpsConsumerManager *)this, NULL,
                              NULL, kThreadPriority, 0, K_FOREVER);
}

bool testGpsConsumerManager::initialize() {
  bool success = true;

  if(!success){
    LOG_ERR("Failed to initialize testGpsConsumerManager.");
  }else{
    LOG_INF("Initialized testGpsConsumerManager successfully.");
  }
  return success;
}

// Starts this thread
void testGpsConsumerManager::start() { k_thread_start(kThreadId); }
