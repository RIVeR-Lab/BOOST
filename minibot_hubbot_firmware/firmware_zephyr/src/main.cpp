#include "gpsManager.h"
#include "logging/log.h"
#include "mainThread.h"
#include "testGpsConsumerManager.h"
#include "uartBase.h"
#include <debug/thread_analyzer.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/uart.h>
#include <stdio.h>
#include <zephyr.h>

LOG_MODULE_REGISTER(main_cpp, LOG_LEVEL_DBG);

#define MAIN_CPP_LOOP_TIME_MS 1000

void main(void) {
  LOG_INF("Main Function\n");

  // Spawn TheMainThread which spawns all other threads
  theMainThread.create();
  theMainThread.initialize();
  theMainThread.start();

  thread_analyzer_print();

  k_sleep(K_FOREVER);
  while (1) {
    LOG_ERR("main.cpp should never get here!!\n");
  }
}