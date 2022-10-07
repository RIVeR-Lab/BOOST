/**
 * mainThread.h
 *
 * Author: David Antaki
 * Date: 10/5/2022
 */

#ifndef SRC_MAINTHREAD_H
#define SRC_MAINTHREAD_H
#include "3rd_party/TinyGPSPlus.h"
#include "export/gpsDatagram.h"
#include "gpsManager.h"
#include "testGpsConsumerManager.h"
#include "uartBase.h"
#include "ubxNeo6M.h"
#include <device.h>
#include <drivers/gpio.h>
#include <stdio.h>
#include <zephyr.h>

class testGpsConsumerManager;

// LED0
#define LED0_NODE DT_ALIAS(led0)
#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0 ""
#define PIN 0
#define FLAGS 0
#endif

// USART2
#define USART2_ID DT_NODELABEL(usart2)

#if DT_NODE_HAS_STATUS(USART2_ID, okay)
#define USART2_LABEL DT_LABEL(USART2_ID)
#else
#error "Unsupported board"
#endif

// class testGpsConsumerManager;

class mainThread {
public:
  mainThread();
  ~mainThread(){};
  void create();
  bool initialize();
  void start();
  static const uint32_t kStackSize = 1024; // in bytes

  bool initDevices();

  /* ------------------ DEVICES ------------------ */
  const struct device *led0_dev = device_get_binding(LED0);
  const struct device *uart2Dev = device_get_binding(USART2_LABEL);
  uartBase uart2;
  /* ------------------ END DEVICES ------------------ */

  /* ------------------ THREADS ------------------ */
  testGpsConsumerManager testGpsManager;
  gpsManager gpsThread;
  /* ------------------ END THREADS ------------------ */

private:
  static const uint32_t kThreadPriority = 99;
  static struct k_thread threadData;
  static k_tid_t kThreadId;
  static const uint32_t loopTimeMs = 1000;

  void loopHook();
  bool processMbx();
  static void entryPoint(void *, void *, void *);
};

extern mainThread theMainThread;

#endif // SRC_MAINTHREAD_H
