#include "main.h"
#include "RealMain.h"
#include "Encoder.h"

void Update_IT_callback(void)
{ // Toggle pin. 10hz toogle --> 5Hz PWM
  Serial2.printf("callback\n\r");
}

void setup() {
  // delay(2000);
  // Serial2.begin(57600);
  // delay(1000);
  // Serial2.println("Hello World!");
  realMain.initialize();
  realMain.loop();

  // HardwareTimer *MyTim = new HardwareTimer(TIM5);
  // MyTim->setOverflow(1, HERTZ_FORMAT); // 10 Hz
  // MyTim->attachInterrupt(Update_IT_callback);
  // MyTim->resume();
}
long oldPosition  = -999;
void loop() {
  // Execution should not get here!!
  Serial.print("!!!!!!!!!Execution should not get here!!!!!!!!!");
}
