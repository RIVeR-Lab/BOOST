#ifndef _SRC_REALMAIN_H
#define _SRC_REALMAIN_H

#include "RealMain.h"
#include "pins.h"
#include "rosHandler.h"
#include "utils/log.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <STM32encoder.h>
#include "imu.h"

extern void _Error_Handler(const char *msg, int val);

// Like the Controller in MVC
class RealMain {
public:
  RealMain()
      : mySerial4(UART4),
        i2c1(PB9, PB8),
        rosHandler(Serial2),
        encLeft(TIM2),
        encRight(TIM3),
        imu(55, 0x28, i2c1) {}
  ~RealMain() {}

  // ------------------------------ DEVICES ------------------------------
  HardwareSerial mySerial4;
  TwoWire i2c1;
  RosHandler rosHandler;
  STM32encoder encLeft;
  STM32encoder encRight;
  IMU imu;
  // ----------------------------------------------------------------

  bool initialize() {
    bool success = true;
    delay(2000);
    pinMode(LED_BUILTIN, OUTPUT);

#if NUCLEO_F767ZI_CUSTOM
    serial2.setRx(PD_6);
    serial2.setTx(PD_5);
#endif
#if NUCLEO_F446RE_CUSTOM
    Serial2.end();
    Serial2.setRx(PA_3);
    Serial2.setTx(PA_2);
    Serial2.begin(RosHandler::rosSerialBaud);
    while (!Serial2) {
      digitalWrite(LED_BUILTIN, HIGH);
      yield();
    }
  
    mySerial4.end();
    mySerial4.setRx(PA_1);
    mySerial4.setTx(PA_0);
    // mySerial4.begin(RosHandler::rosSerialBaud);
    // while(!mySerial4){
    //   yield();
    // }
#endif
    LOGEVENT("Setup...");

    success = success && rosHandler.init();
    #if ENABLE_IMU
    success = success && imu.init();
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
        rosHandler.nodeHandle.loginfo("Looping...");
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }

      if(encLeft.isUpdated()){
        LOGEVENT("Left Encoder: %d", encLeft.pos());
      }
      if(encRight.isUpdated()){
        LOGEVENT("Right Encoder: %d", encRight.pos());
      }

      rosHandler.loop();
      #if ENABLE_IMU
      imu.loop();
      #endif
      // mySerial4.printf("looping\n");

      // analogWrite(L_WHEEL_FORW_PIN, 255);
      // LOGEVENT("ADC vRef Read (mV): %d", readVref());
      // LOGEVENT("ADC Read (mV): %d", readVoltage(readVref(), PA3));
      // HAL_Delay(500);
      // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      // HAL_Delay(10);


    }
  }

private:
  bool initialized = false;

  bool deinitialize();
};

// The one and only main thread allocated statically.
extern RealMain realMain;
// The UART used for logging.
#define Console Serial2

#endif // _SRC_REALMAIN_H