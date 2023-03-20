#include "RealMain.h"

RealMain realMain;
HardwareSerial console6(USART6);

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

bool RealMain::init() {
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
  console6.end();
  console6.setTx(PC_6);
  console6.setRx(PC_7);
  console6.begin(57600);
  while (!console6) {
    yield();
  }
  LOGEVENT("Initializing...");
  Console.println("RealMain::Initializeing..."); // Bypass LOGGING
  LOGINFO("%s: %s", Version::getBuildTimestamp().c_str(),
          Version::getGitCommitSha1().c_str());
  ROSLOGINFO("%s: %s", Version::getBuildTimestamp().c_str(),
             Version::getGitCommitSha1().c_str());

#if ENABLE_ROSMANAGER
  success = rosManager.init() && success;
#endif

#if ENABLE_IMU
  success = imuManager.init() && success;
#endif

#if ENABLE_ODOMETRY
  success = odomManager.init() && success;
#endif

#if ENABLE_GPS
  success = gpsManager.init() && success;
#endif

#if ENABLE_DRVMANAGER
  success = drvManager.init() && success;
#endif

  if (!success) {
    LOGERROR("Failed to initialize");
    Console.println("RealMain::Failed to initialize"); // Bypass LOGGING
    ROSLOGERROR("Failed to initialize");
    initialized = false;
  } else {
    LOGEVENT("Initialized");
    Console.println("RealMain::Initialized"); // Bypass LOGGING
    ROSLOGINFO("Initialized");
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
      Console.println("Looping...");
      // TODO: Make LEDs on PCB blink
      // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

#if ENABLE_ROSMANAGER
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