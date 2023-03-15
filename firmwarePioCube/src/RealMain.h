#ifndef _SRC_REALMAIN_H
#define _SRC_REALMAIN_H

#include "RealMain.h"
#include "pins.h"
#include "utils/log.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include "config.h"
#include "utils.h"
#include "Encoder.h"
#include "TXB0104PWR.h"
#include "OdometryManager.h"
#include "BNO055Manager.h"
#include "RosManager.h"
#include "L293N.h"

extern void _Error_Handler(const char *msg, int val);

// Like the Controller in MVC
class RealMain {
public:
  RealMain()
      : mySerial4(UART4),
        i2c1(PB9, PB8),
        imu(55, 0x28, &i2c1),
        encLeft(L_ENCODER_PIN1, L_ENCODER_PIN2),
        encRight(R_ENCODER_PIN1, R_ENCODER_PIN2),
        encoderLvlShifter(ENCODER_LVL_SHIFTER_EN),
        rosManager(Serial2),
        odomManager(encLeft, encRight, encoderLvlShifter),
        imuManager(imu)
        {}
  ~RealMain() {}
  
  friend class RosManager;
  friend class BNO055Manager;
private:
  // ------------------------------ DEVICES ------------------------------
  HardwareSerial mySerial4;
  TwoWire i2c1;
  Adafruit_BNO055 imu;
  Encoder encLeft;
  Encoder encRight;
  TXB0104PWR encoderLvlShifter;
  L293N mtrCtrl;
  // ------------------------------ END DEVICES ------------------------------

  // ------------------------------ FAKE THREADS ------------------------------
  RosManager rosManager;
  OdometryManager odomManager;
  BNO055Manager imuManager;
  // ------------------------------ END FAKE THREADS ------------------------------
  
public:
  bool initialize() {
    bool success = true;
    delay(2000);

#if NUCLEO_F767ZI_CUSTOM
    serial2.setRx(PD_6);
    serial2.setTx(PD_5);
#endif
#if NUCLEO_F446RE_CUSTOM
    Serial2.end();
    Serial2.setRx(PA_3);
    Serial2.setTx(PA_2);
    Serial2.begin(RosManager::rosSerialBaud);
    while (!Serial2) {
      yield();
    }
  
    // mySerial4.end();
    // mySerial4.setRx(PA_1);
    // mySerial4.setTx(PA_0);
    // mySerial4.begin(RosManager::rosSerialBaud);
    // while(!mySerial4){
    //   yield();
    // }
#endif
    LOGEVENT("Setup...");

    #if ENABLE_ROSHANDLER
    success = success && rosManager.init();
    #endif
    
    #if ENABLE_IMU
    success = success && imuManager.init();
    #endif
    
    #if ENABLE_ODOMETRY
    success = success && odomManager.init();
    #endif

    // initPwm();
    // setPwm(5000, 50);
    // initAdc();
    pinMode(L_WHEEL_FORW_PIN, OUTPUT);
    pinMode(L_WHEEL_BACK_PIN, OUTPUT);
    pinMode(R_WHEEL_FORW_PIN, OUTPUT);
    pinMode(R_WHEEL_BACK_PIN, OUTPUT);
  

    return success;
  }

  // Main Big Loop
  void loop() {
    while (1) {
      // Blink LED every 1 second
      static uint32_t counter = 0;
      if ((millis() - counter) > 1000) {
        counter = millis();
        LOGEVENT("Looping...");
        Serial2.println("Looping...");
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
    }
  }

  bool deinitialize();

private:
  bool initialized = false;  
};

// The one and only main thread allocated statically.
extern RealMain realMain;
// The UART used for logging.
#define Console Serial2

#endif // _SRC_REALMAIN_H