/**
 * mainThread.cpp
 *
 * Author: David Antaki
 * Date: 10/5/2022
 */

#include "mainThread.h"
LOG_MODULE_REGISTER(mainThread, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(mainThreadStack, mainThread::kStackSize);

// The one and only mainThread
mainThread theMainThread;

mainThread::mainThread()
    : uart2(*uart2Dev), testGpsManager(), gpsThread(uart2){};

void mainThread::entryPoint(void *thisThread, void *, void *) {
  mainThread *pThis = (mainThread *)thisThread;
  pThis->loopHook();
}

k_tid_t mainThread::kThreadId;
struct k_thread mainThread::threadData;

// Creates this thread
void mainThread::create() {
  // We pass a pointer to "this" class instance as a parameter because
  // entryPoint() must be static. Then we can call the passed class instance's
  // loopHook() from within entryPoint().
  // TODO: figure out how to put a thread name in as string so that it
  // comes up in thread_analyze().
  kThreadId = k_thread_create(&threadData, mainThreadStack,
                              K_THREAD_STACK_SIZEOF(mainThreadStack),
                              mainThread::entryPoint, (mainThread *)this, NULL,
                              NULL, kThreadPriority, 0, K_FOREVER);
}

bool mainThread::initDevices() {
  bool success = true;
  int ret;

  if (led0_dev == NULL) {
    LOG_ERR("Failed to initialized LED0.");
    return false;
  }

  ret = gpio_pin_configure(led0_dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
  if (ret < 0) {
    LOG_ERR("Failed to configure GPIO for LED0.");
    return false;
  }

  return success;
}

bool mainThread::initialize() {
  bool success = true;

  success = success && initDevices();

  gpsThread.create();
  success = success && gpsThread.initialize();
  gpsThread.start();

  testGpsManager.create();
  success = success && testGpsManager.initialize();
  testGpsManager.start();

  if (!success) {
    LOG_ERR("Failed to initialize mainThread.");
  } else {
    LOG_INF("Initialized mainThread successfully.");
  }
  return success;
}

// Starts this thread
void mainThread::start() { k_thread_start(kThreadId); }

// The entry point and/or "main" function of this thread
void mainThread::loopHook() {
  bool led_is_on = true;

  while (1) {
    // Blink the LED
    gpio_pin_set(led0_dev, PIN, (int)led_is_on);
    led_is_on = !led_is_on;

    k_msleep(loopTimeMs);
  }
}