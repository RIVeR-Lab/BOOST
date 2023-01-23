#ifndef _SRC_REALMAIN_H
#define _SRC_REALMAIN_H

#include <Arduino.h>
#include "RealMain.h"
#include "pins.h"
#include <SPI.h>
#include <Wire.h>
#include "utils/log.h"

// Like the Controller in MVC
class RealMain {
public:
  RealMain() {}
  ~RealMain() {}
  bool go();

private:
  bool initialized = false;
  bool initialize();
  bool deinitialize();
  
};

// extern RealMain realMain;

#endif // _SRC_REALMAIN_H