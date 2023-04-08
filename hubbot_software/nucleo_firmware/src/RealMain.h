#ifndef _SRC_REALMAIN_H
#define _SRC_REALMAIN_H

#include "BNO055Manager.h"
#include "DriveManager.h"
#include "Encoder.h"
#include "GpsManager.h"
#include "L293N.h"
#include "OdometryManager.h"
#include "RealMain.h"
#include "RosManager.h"
#include "TXB0104PWR.h"
#include "config.h"
#include "utils/utils.h"
#include "utils/log.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include "version.h"

extern void _Error_Handler(const char *msg, int val);

// Like the Controller in MVC
class RealMain {
public:
  RealMain()
      : mySerial4(UART4), uart3Gps(USART3), i2c1(PB9, PB8),
        imu(55, 0x28, &i2c1), encLeft(L_ENCODER_PIN1, L_ENCODER_PIN2),
        encRight(R_ENCODER_PIN1, R_ENCODER_PIN2),
        encoderLvlShifter(ENCODER_LVL_SHIFTER_EN),
        mtrCtrl(L_WHEEL_FORW_PIN, L_WHEEL_BACK_PIN, R_WHEEL_FORW_PIN,
                R_WHEEL_BACK_PIN), mtrCtrlLvlShifter(MOTOR_LVL_SHIFTER_EN_PIN),
        gps(uart3Gps, GPS_RX_PIN, GPS_TX_PIN, GPS_RESET_N_PIN, GPS_1PPS_PIN,
            GPS_FORCE_ON_N_PIN),
        rosManager(Serial2), odomManager(encLeft, encRight, encoderLvlShifter),
        imuManager(imu), drvManager(mtrCtrl, mtrCtrlLvlShifter), gpsManager(gps) {}
  ~RealMain() {}

  friend class RosManager;
  friend class OdometryManager;
  friend class BNO055Manager;
  friend class DriveManager;
  friend class GpsManager;
  friend bool log_printf_level(bool ros, const char *file, uint32_t line, uint32_t level,
                      bool addNL, const char *Format, ...);
  friend bool log_printf_error(bool ros, const char *file, uint32_t line, const char *Format,
                      ...);

private:
  // ------------------------------ DEVICES ------------------------------
  HardwareSerial mySerial4;
  HardwareSerial uart3Gps;
  TwoWire i2c1;
  Adafruit_BNO055 imu;
  Encoder encLeft;
  Encoder encRight;
  TXB0104PWR encoderLvlShifter;
  L293N mtrCtrl;
  TXB0104PWR mtrCtrlLvlShifter;
  SL871 gps;
  // ------------------------------ END DEVICES ------------------------------

  // ------------------------------ FAKE THREADS ------------------------------
  RosManager rosManager;
  OdometryManager odomManager;
  BNO055Manager imuManager;
  DriveManager drvManager;
  GpsManager gpsManager;
  // ------------------------------ END FAKE THREADS
  // ------------------------------

public:
  bool init();
  // Main Big Loop
  void loop();

private:
  bool initialized = false;
};

// The one and only main thread allocated statically.
extern RealMain realMain;
extern HardwareSerial console6;
// The UART used for logging.
#define Console console6

#endif // _SRC_REALMAIN_H