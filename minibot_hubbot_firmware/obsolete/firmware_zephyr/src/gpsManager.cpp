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
    gps.checkProbOfChecksumFail();
    processMbx();
    // Yield to other theads with same or higher priority.
    k_yield();

    // Print GPS data:
    // if (k_uptime_get() - lastPrintMs > 3000) {
    //   lastPrintMs = k_uptime_get();
    //   // TODO: Change to use LOGGING instead
    //   printk("Sats  Latitude   Longitude   Fix      Date    Time     Date  Alt "
    //          "   Course Speed Card  Chars Sentences Checksum\r\n");
    //   printk("       (deg)      (deg)    Age(ms)                     Age   (m) "
    //          "   --- from GPS ----   RX    RX        Fail\r\n");
    //   printk("-----------------------------------------------------------------"
    //          "-----------------------------------------------------------------"
    //          "------\r\n");
    //   printInt(gps.getSatellites().value(), gps.getSatellites().isValid(), 5);
    //   // printFloat(gps.getHdop().hdop(), gps.getHdop().isValid(), 6, 1);
    //   printFloat(gps.getLocation().lat(), gps.getLocation().isValid(), 11, 6);
    //   printFloat(gps.getLocation().lng(), gps.getLocation().isValid(), 12, 6);
    //   printInt(gps.getLocation().age(), gps.getLocation().isValid(), 5);
    //   printDateTime(gps.getDate(), gps.getTime());
    //   printFloat(gps.getAltitude().meters(), gps.getAltitude().isValid(), 7, 2);
    //   printFloat(gps.getCourse().deg(), gps.getCourse().isValid(), 7, 2);
    //   printFloat(gps.getSpeed().kmph(), gps.getSpeed().isValid(), 6, 2);
    //   // const char* deg = TinyGPSPlus::cardinal(gps.getCourse().deg());
    //   // printStr(gps.getCourse().isValid(), deg); // This causes fault
    //   printk("        ");
    //   printInt(gps.charsProcessed(), true, 6);
    //   printk("  ");
    //   printInt(gps.sentencesWithFix(), true, 10);
    //   printk("  ");
    //   printInt(gps.failedChecksum(), true, 9);
    //   printk("\r\n\r\n");
    // }
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
  k_mbox_init(&mailbox);
  success = success && gps.initialize();

  if (!success) {
    LOG_ERR("Failed to initialize gpsManager.");
  } else {
    LOG_INF("Initialized gpsManager successfully.");
  }
  return success;
}

// Starts this thread
void gpsManager::start() { k_thread_start(kThreadId); }

bool gpsManager::processMbx() {
  bool success = true;
  struct k_mbox_msg recv_msg;

  /* prepare to receive message */
  recv_msg.size = 10000;
  recv_msg.rx_source_thread = K_ANY;

  /* get message, but not its data */
  int hasMessage = k_mbox_get(&mailbox, &recv_msg, NULL, K_NO_WAIT);

  if (hasMessage == 0) {
    // Process the specific received message
    switch (recv_msg.info) {
    case GET_GPS_DATAGRAM: {
      // Copy data to the sender's buffer
      memcpy(recv_msg.tx_data, gps.getLastGoodReading(), sizeof(gpsDatagram));

      // Delete message from mailbox and release sender if they are waiting.
      k_mbox_data_get(&recv_msg, NULL);
    } break;
    case UNKNOWN_MSG:
    default: {
      // Delete message from mailbox and release sender if it is waiting for
      // message to be received.
      k_mbox_data_get(&recv_msg, NULL);
      LOG_ERR("Invalid message type.");
      success = false;
    }
    }
  }

  return success;
}

/**
 * This is to be called from a different thread that wants to get GPS data
 * from this thread.
 */ 
bool gpsManager::getGpsData(gpsDatagram &outData) {
  bool success = true;
  int ret;

  struct k_mbox_msg send_msg;
  char buffer[sizeof(gpsDatagram)]{};
  size_t buffer_bytes_used = sizeof(gpsDatagram);

  /* prepare to send message */
  send_msg.info = GET_GPS_DATAGRAM;
  send_msg.size = sizeof(buffer);
  send_msg.tx_data = &outData;
  send_msg.tx_block.data = NULL;
  send_msg.tx_target_thread = kThreadId;

  /* send message and wait until thread receives */
  ret = k_mbox_put(&mailbox, &send_msg, K_FOREVER);

  if(ret < 0){
    LOG_ERR("getGpsData() msg failed: err(%d)", ret);
    success = false;
  }

  return success;
}