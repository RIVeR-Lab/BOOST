#include "GpsManager.h"

bool GpsManager::init() {
  bool success = true;
  success = success && gps.init();
  return success;
}

bool GpsManager::loopHook() {
  // Read latest GPS data
  uint8_t data[256]{};
  gps.init();
  // gps.uart.end();
  // gps.uart.setRx(PC11);
  // gps.uart.setTx(PC10);
  // gps.uart.begin(9600);
  while (1) {
    int bytes = gps.uart.available();
    if (bytes >= 255) {
      Serial2.println("UART3 BUFFER IS FULL!");
    }
    while (bytes > 0) {
      Serial2.printf("%d", gps.uart.read());
      bytes--;
      if (bytes == 0) {
        Serial2.println();
      }
    }
    delay(100);
  }

  // gps.recv(data, 256);
  // char buf[256];
  // for (int i = 0; i < 256; i++) {
  //   sprintf(buf, "%c", data[i]);
  // }
  // LOGEVENT(buf);
  // return true;
}

bool GpsManager::logLoopHook() {
  char buf[256];
  LOGEVENT(buf);

  return true;
}
