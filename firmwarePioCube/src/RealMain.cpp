#include "RealMain.h"

RealMain realMain;

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(const char *msg, int val) {
  /* User can add his own implementation to report the HAL error return state */
  LOGERROR("Error: %s (%i)", msg, val);
  while (1) {
    LOGERROR("Error: %s (%i)", msg, val);
  }
}

bool RealMain::initialize() {
  bool success = true;
  delay(2000);
#if NUCLEO_F446RE_CUSTOM
  Serial2.end();
  Serial2.setRx(PA_3);
  Serial2.setTx(PA_2);
  Serial2.begin(RosManager::rosSerialBaud);
  while (!Serial2) {
    yield();
  }
#endif
  LOGEVENT("Initializeing...");
  Console.println("RealMain::Initializeing..."); // Bypass LOGGING

#if ENABLE_ROSHANDLER
  success = success && rosManager.init();
#endif

#if ENABLE_IMU
  success = success && imuManager.init();
#endif

#if ENABLE_ODOMETRY
  success = success && odomManager.init();
#endif

#if ENABLE_GPS
  success = success && gpsManager.init();
#endif

  if (!success) {
    LOGERROR("Failed to initialize");
    Console.println("RealMain::Failed to initialize"); // Bypass LOGGING
    initialized = false;
  } else {
    LOGEVENT("Initialized");
    Console.println("RealMain::Initialized"); // Bypass LOGGING
    initialized = true;
  }

  return success;
}

void RealMain::loop() {
  while (1) {
    // Blink LED every 1 second
    static uint32_t counter = 0;
    if ((millis() - counter) > 1000) {
      counter = millis();
      LOGEVENT("Looping...");
      // TODO: Make LEDs on PCB blink
      // Serial2.println("Looping...");
      // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

#if ENABLE_ROSHANDLER
    rosManager.loop();
#endif

#if ENABLE_IMU
    imuManager.loop();
#endif

#if ENABLE_ODOMETRY
    odomManager.loop();
#endif

#if ENABLE_GPS
    gpsManager.loop();
#endif
  }
}