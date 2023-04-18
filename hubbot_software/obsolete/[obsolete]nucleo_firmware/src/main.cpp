#include "main.h"
#include "RealMain.h"
#include "HardwareSerial.h"

void Update_IT_callback(void)
{ // Toggle pin. 10hz toogle --> 5Hz PWM
  Serial2.printf("callback\n\r");
}

void testGPS() {
  delay(2000);
  Serial2.begin(57600);
  delay(1000);
  Serial2.println("Hello World!");

  HardwareSerial u3(USART3);
  u3.end();
  u3.setRx(PC11);
  u3.setTx(PC10);
  u3.begin(9600);

  while(1) {
    int bytes = u3.available();
    if(bytes >= 255) {
      Serial2.println("Too many bytes!");
    }
    while(bytes > 0) {
      Serial2.printf("%d", u3.read());
      bytes--;
      if(bytes == 0) {
        Serial2.println();
      }
    }
    delay(100);
  }
}

void setup() {
  realMain.init();
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
