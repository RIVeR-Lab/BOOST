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
  console6.setTx(SERIAL_CONSOLE_TX_PIN);
  console6.setRx(SERIAL_CONSOLE_RX_PIN);
  console6.begin(57600);
  while (!console6) {
    yield();
  }
  LOGEVENT("Initializing...");
  Console.println("RealMain::Initializeing..."); // Bypass LOGGING
  LOGINFO("FIRMWARE_VERSION:%s:SHA:%s", Version::getBuildTimestamp().c_str(),
          Version::getGitCommitSha1().c_str());

// TODO: Replace with XMacro
#if ENABLE_ROSMANAGER
  success = rosManager.init() && success;
#endif

  // Has to be after rosManager.init() or it won't publish.
  ROSLOGINFO("FIRMWARE_VERSION:%s:SHA:%s", Version::getBuildTimestamp().c_str(),
             Version::getGitCommitSha1().c_str());

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

  pinMode(BATT0_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(BATT1_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(BATT2_CONTINUITY_PIN, INPUT_PULLUP);
  pinMode(WING_CONTINUITY_PIN, INPUT_PULLUP);

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

    static uint32_t contDataCounter = 0;
    if ((millis() - contDataCounter) > 500) {
      contDataCounter = millis();
      // Get and send all battery coninuity data
      int batt0 = digitalRead(BATT0_CONTINUITY_PIN);
      int batt1 = digitalRead(BATT1_CONTINUITY_PIN);
      int batt2 = digitalRead(BATT2_CONTINUITY_PIN);
      int wing = digitalRead(WING_CONTINUITY_PIN);
      Serial2.printf(">batt0Cont=%d\r\n", batt0);
      Serial2.printf(">batt1Cont=%d\r\n", batt1);
      Serial2.printf(">batt2Cont=%d\r\n", batt2);
      Serial2.printf(">wingCont=%d\r\n", wing);
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

#if ENABLE_DRVMANAGER
    drvManager.loop();
#endif
  }
}