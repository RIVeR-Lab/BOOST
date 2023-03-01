#include "main.h"
#include "RealMain.h"

void setup() {
  realMain.initialize();
  realMain.loop();
}

void loop() {
  // Execution should not get here!!
  Serial.print("!!!!!!!!!Execution should not get here!!!!!!!!!");
}
