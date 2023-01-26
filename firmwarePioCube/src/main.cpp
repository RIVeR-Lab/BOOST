#include "main.h"
#include <ros.h>
#include <std_msgs/String.h>

// Devices
// I just call SPI.something() directly.

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
char hello[13] = "hello world!";

void setup() {
  // put your setup code here, to run once:
  #if LOGGING
  delay(2000);
  Console.begin(115200);
  while(!Console){
    yield();
  }
  #endif

  LOGEVENT("Setup...");

  // initPwm();
  // setPwm(5000, 50);
  // initAdc();

  nh.initNode();
  nh.advertise(chatter);

  LOGEVENT("\n\n\n");
  // LOGEVENT("Starting RealMain");
  // realMain.go();

  // LOGEVENT("RealMain exited.");
  // LOGEVENT("Shutting off device.");
}

void loop() {
  LOGEVENT("Looping...");
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  // LOGEVENT("ADC vRef Read (mV): %d", readVref());
  // LOGEVENT("ADC Read (mV): %d", readVoltage(readVref(), PA3));
  HAL_Delay(1000);
}
