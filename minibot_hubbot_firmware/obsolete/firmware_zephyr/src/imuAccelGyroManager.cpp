/**
 * imuAccelGyroManager.cpp
 *
 * Author: David Antaki
 * Date: 11/21/2022
 */

#include "imuAccelGyroManager.h"
LOG_MODULE_REGISTER(imuAccelGyroManager, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(imuAccelGyroManager_stack, imuAccelGyroManager::kStackSize);

imuAccelGyroManager::imuAccelGyroManager(const struct device & _i2cDev) : i2cDev(_i2cDev) {}

imuAccelGyroManager::~imuAccelGyroManager() {}

// The entry point and/or "main" function of this thread
void imuAccelGyroManager::loopHook() {
  int64_t lastPrintMs = k_uptime_get();
  while (1) {
    getImuData();
    // processMbx();
    LOG_DBG("imuAccelGyroManager thread running.");
    k_msleep(loopTimeMs);
  }
}

void imuAccelGyroManager::entryPoint(void *thisThread, void *, void *) {
  imuAccelGyroManager *pThis = (imuAccelGyroManager *)thisThread;
  pThis->loopHook();
}

k_tid_t imuAccelGyroManager::kThreadId;
struct k_thread imuAccelGyroManager::imuAccelGyroManager_thread_data;

// Creates this thread
void imuAccelGyroManager::create() {
  // We pass a pointer to "this" class instance as a parameter because
  // entryPoint() must be static. Then we can call the passed class instance's
  // loopHook() from within entryPoint().
  // TODO: figure out how to put a thread name in as string so that it
  // comes up in thread_analyze().
  kThreadId = k_thread_create(&imuAccelGyroManager_thread_data, imuAccelGyroManager_stack,
                              K_THREAD_STACK_SIZEOF(imuAccelGyroManager_stack),
                              imuAccelGyroManager::entryPoint, (imuAccelGyroManager *)this, NULL,
                              NULL, kThreadPriority, 0, K_FOREVER);
}

bool imuAccelGyroManager::initialize() {
  bool success = true;
  k_mbox_init(&mailbox);


  if (!success) {
    LOG_ERR("Failed to initialize imuAccelGyroManager.");
  } else {
    LOG_INF("Initialized imuAccelGyroManager successfully.");
  }
  return success;
}

// Starts this thread
void imuAccelGyroManager::start() { k_thread_start(kThreadId); }

bool imuAccelGyroManager::processMbx() {
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
