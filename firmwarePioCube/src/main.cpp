#include "main.h"
#include "rosHandler.h"

// Devices
// I just call SPI.something() directly.
HardwareSerial mySerial1(USART1);
HardwareSerial mySerial2(USART2);
HardwareSerial mySerial4(UART4);
RosHandler rosHandler(Serial2);


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(const char *msg, int val)
{
  /* User can add his own implementation to report the HAL error return state */
  LOGERROR("Error: %s (%i)", msg, val);
  while (1) {
  }
} 

void setup() {
  bool success = true;
  // put your setup code here, to run once:
  #if LOGGING
  delay(2000);
  Console.begin(RosHandler::rosSerialBaud);
  while(!Console){
    yield();
  }
  #endif

#if NUCLEO_F767ZI_CUSTOM
  serial2.setRx(PD_6);
  serial2.setTx(PD_5);
#endif
#if NUCLEO_F446RE_CUSTOM
  // serial2.setRx(PA_2);
  // serial2.setTx(PA_3);
  mySerial4.setRx(PA_1);
  mySerial4.setTx(PA_0);
#endif
  // mySerial4.begin(RosHandler::rosSerialBaud);
  // while(!mySerial4){
  //   yield();
  // }
  
  success = success && rosHandler.init();
  LOGEVENT("Setup...");

  // initPwm();
  // setPwm(5000, 50);
  // initAdc();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(L_WHEEL_FORW_PIN, OUTPUT);
  pinMode(L_WHEEL_BACK_PIN, OUTPUT);
  pinMode(R_WHEEL_FORW_PIN, OUTPUT);
  pinMode(R_WHEEL_BACK_PIN, OUTPUT);

  LOGEVENT("\n\n\n");
  // LOGEVENT("Starting RealMain");
  // realMain.go();

  // LOGEVENT("RealMain exited.");
  // LOGEVENT("Shutting off device.");
}

void loop() {
  // Blink LED every 1 second
  static uint32_t counter = 0;
  if((millis() - counter) > 1000) {
    counter = millis();
    LOGEVENT("Looping...");
    rosHandler.nodeHandle.loginfo("Looping...");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  rosHandler.loop();
  // mySerial4.printf("looping\n");
  
  
  // analogWrite(L_WHEEL_FORW_PIN, 255);
  // LOGEVENT("ADC vRef Read (mV): %d", readVref());
  // LOGEVENT("ADC Read (mV): %d", readVoltage(readVref(), PA3));
  // HAL_Delay(500);
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  // HAL_Delay(10);
}
