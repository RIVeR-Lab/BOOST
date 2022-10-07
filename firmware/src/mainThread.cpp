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

mainThread::mainThread() : uart2(*uart2Dev), testGpsManager(), gpsThread(uart2)
{};

// The entry point and/or "main" function of this thread
void mainThread::loopHook() {
  int64_t lastPrintMs = k_uptime_get();
  while (1) {
    
  }
}

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

bool mainThread::initialize() {
  bool success = true;

	gpsThread.create();
	success = success && gpsThread.initialize();
	gpsThread.start();

	testGpsManager.create();
	success = success && testGpsManager.initialize();
	testGpsManager.start();

  if(!success){
    LOG_ERR("Failed to initialize mainThread.");
  }else{
    LOG_INF("Initialized mainThread successfully.");
  }
  return success;
}

// Starts this thread
void mainThread::start() { k_thread_start(kThreadId); }