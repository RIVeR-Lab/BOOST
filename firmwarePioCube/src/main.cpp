#include "main.h"
#include "RealMain.h"
#include "Encoder.h"

// STM32encoder enc(TIM2, 0, 4);
// HardwareTimer tim(TIM5);
HardwareTimer enc(TIM2);
Encoder myEnc(A0, A1);

void Update_IT_callback(void)
{ // Toggle pin. 10hz toogle --> 5Hz PWM
  Serial2.printf("callback\n\r");
}

void enc_handler(){
  Serial2.printf("enc_handler\n\r");
}

void setup() {
  delay(2000);
  Serial2.begin(57600);
  delay(1000);
  Serial2.println("Hello World!");
  // realMain.initialize();
  // realMain.loop();

  // attachInterrupt(digitalPinToInterrupt(PA0),enc_handler,CHANGE);

  

  HardwareTimer *MyTim = new HardwareTimer(TIM5);
  MyTim->setOverflow(1, HERTZ_FORMAT); // 10 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
}
long oldPosition  = -999;
void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial2.println(newPosition);
  }
  // Serial.printf("dir:%u pos:%u\n", enc.dir(), enc.pos());
  // Serial2.println("Hello World!");
  // if (enc.isUpdated()) {
  //   Serial2.printf("pos:%ld\n", enc.pos());
  // }
  // // Execution should not get here!!
  // Serial.print("!!!!!!!!!Execution should not get here!!!!!!!!!");
}
