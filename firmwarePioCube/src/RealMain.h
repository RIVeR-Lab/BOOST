#ifndef _SRC_REALMAIN_H
#define _SRC_REALMAIN_H

#include "OrcaHardware/HPT5K0.h" // This must be the first include to avoid MACRO "INPUT" conflict
//
#include <Arduino.h>
#include "PsuManager.h"
#include "RealMain.h"
#include "common.h"
#include "pins.h"
#include <SPI.h>
#include <Wire.h>
#include "MantaCommon/src/utils/log.h"
#include "Board2Board/Logging.h"
#include "MantaHW/I2cBase.h"
#include "View/IView.h"
#include "View/ePaper.h"

class B2BReporter;

// Like the Controller in MVC
class RealMain {
public:
  RealMain(ePaper& _view) : fakeLog(nullptr, false, false), psuPort(Wire, 50000),
        psuManager(fakeLog, psuPort), view(_view) {}
  ~RealMain() {}
  bool go();

private:
  State_t state{};
  Logging fakeLog; // Do not use this. It is stubbed out.
  I2cBase psuPort;
  PsuManager psuManager;
  ePaper &view;
  bool initialized = false;
  bool initialize();
  bool deinitialize();
  bool initDipSwitch();
  bool getPsuVoltageSelection();
  bool program();
  
};

// extern RealMain realMain;

#endif // _SRC_REALMAIN_H