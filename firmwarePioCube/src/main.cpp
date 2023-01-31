#include "main.h"
#include <ros.h>
#include <std_msgs/String.h>

// Devices
// I just call SPI.something() directly.
// HardwareSerial serial2(USART2);

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

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "MCU blink!";

void setup() {
  // put your setup code here, to run once:
  #if LOGGING
  delay(2000);
  Console.begin(115200);
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
#endif
  // serial2.begin(115200);
  // while(!serial2){
  //   yield();
  // }
  // serial2.printf("hellow:\n");


  LOGEVENT("Setup...");

  // initPwm();
  // setPwm(5000, 50);
  // initAdc();

  nh.getHardware()->setBaud(57600);
  nh.getHardware()->setPort(&Serial2);
  nh.initNode();
  nh.advertise(chatter);

  pinMode(LED_BUILTIN, OUTPUT);

  LOGEVENT("\n\n\n");
  // LOGEVENT("Starting RealMain");
  // realMain.go();

  // LOGEVENT("RealMain exited.");
  // LOGEVENT("Shutting off device.");
}

void loop() {
  LOGEVENT("Looping...");
  // serial2.printf("hellow:\r\n");
  str_msg.data = hello;
  chatter.publish( &str_msg );
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  nh.spinOnce();
  // nh.publish();
  // LOGEVENT("ADC vRef Read (mV): %d", readVref());
  // LOGEVENT("ADC Read (mV): %d", readVoltage(readVref(), PA3));
  HAL_Delay(500);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  HAL_Delay(500);
}
