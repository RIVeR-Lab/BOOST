/**
 * testGpsConsumerManager.cpp
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#include "testGpsConsumerManager.h"
#include "mainThread.h"
LOG_MODULE_REGISTER(testGpsConsumerManager, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(testGpsConsumerManagerStack,
                      testGpsConsumerManager::kStackSize);

testGpsConsumerManager::testGpsConsumerManager() {}

testGpsConsumerManager::~testGpsConsumerManager() {}

// The entry point and/or "main" function of this thread
void testGpsConsumerManager::loopHook() {
  while (1) {
    /* Allow other thread/workqueue to work. */
    k_msleep(loopTimeMs);

    // This is simply to test passing data from the the GPS thread to this
    // thread.
    gpsDatagram outData;
    theMainThread.gpsThread.getGpsData(outData);

    // Print GPS data:
    // TODO: Change to use LOGGING instead
    // TODO: Change this dumb format to a json like output format or something
    printk("Sats  HDOP  Latitude   Longitude   Fix      Date    Time     Date  Alt   Course Speed Card  Chars Sentences Checksum  Checksum  RX Fail Prob.\r\n");
    printk("             (deg)      (deg)    Age(ms)                     Age   (m)    --- from GPS ----   RX    RX        Fail       Pass        (%)      \r\n");
    printk("-----------------------------------------------------------------"
            "-----------------------------------------------------------------"
            "------\r\n");
    ubxNeo6M::printInt(outData.getSatellites().value(),
                        outData.getSatellites().isValid(), 5);
    ubxNeo6M::printFloat(outData.getHdop().hdop(), outData.getHdop().isValid(), 6, 1);
    ubxNeo6M::printFloat(outData.getLocation().lat(), outData.getLocation().isValid(),
                          11, 6);
    ubxNeo6M::printFloat(outData.getLocation().lng(), outData.getLocation().isValid(),
                          12, 6);
    ubxNeo6M::printInt(outData.getLocation().age(), outData.getLocation().isValid(),
                        5);
    ubxNeo6M::printDateTime(outData.getDate(), outData.getTime());
    ubxNeo6M::printFloat(outData.getAltitude().meters(),
                          outData.getAltitude().isValid(), 7, 2);
    ubxNeo6M::printFloat(outData.getCourse().deg(), outData.getCourse().isValid(), 7,
                          2);
    ubxNeo6M::printFloat(outData.getSpeed().kmph(), outData.getSpeed().isValid(), 6,
                          2);
    // const char* deg = TinyGPSPlus::cardinal(outData.getCourse().deg());
    // printStr(outData.getCourse().isValid(), deg); // This causes fault
    printk("        ");
    ubxNeo6M::printInt(outData.getCharsProcessed(), true, 6);
    printk("  ");
    ubxNeo6M::printInt(outData.getSentencesWithFix(), true, 10);
    printk("  ");
    ubxNeo6M::printInt(outData.getFailedChecksum(), true, 9);
    printk("  ");
    ubxNeo6M::printInt(outData.getPassedChecksum(), true, 9);
    printk("  ");
    printk("%.2f", ((double)outData.getFailedChecksum() / (double)outData.getPassedChecksum())*100);
    printk("\r\n\r\n");
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
  kThreadId = k_thread_create(
      &testGpsConsumerManagerData, testGpsConsumerManagerStack,
      K_THREAD_STACK_SIZEOF(testGpsConsumerManagerStack),
      testGpsConsumerManager::entryPoint, (testGpsConsumerManager *)this, NULL,
      NULL, kThreadPriority, 0, K_FOREVER);
}

bool testGpsConsumerManager::initialize() {
  bool success = true;

  if (!success) {
    LOG_ERR("Failed to initialize testGpsConsumerManager.");
  } else {
    LOG_INF("Initialized testGpsConsumerManager successfully.");
  }
  return success;
}

// Starts this thread
void testGpsConsumerManager::start() { k_thread_start(kThreadId); }
