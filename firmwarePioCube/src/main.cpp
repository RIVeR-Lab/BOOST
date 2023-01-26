#include "main.h"

#define LOGGING true

// Devices
// I just call SPI.something() directly.
// Console
HardwareSerial Console(PIN_SERIAL_RX, PIN_SERIAL_TX);


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
  // put your setup code here, to run once:
  #if LOGGING
  delay(2000);
  Console.begin(115200);
  while(!Console){
    yield();
  }
  #endif


  // initPwm();
  // setPwm(5000, 50);
  // initAdc();

  LOGEVENT("\n\n\n");
  LOGEVENT("Starting RealMain");
  // realMain.go();

  LOGEVENT("RealMain exited.");
  LOGEVENT("Shutting off device.");
}

void loop() {
  // LOGEVENT("ADC vRef Read (mV): %d", readVref());
  // LOGEVENT("ADC Read (mV): %d", readVoltage(readVref(), PA3));
  HAL_Delay(10);
}
