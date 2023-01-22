#include "RealMain.h"

bool RealMain::go() {
  bool success = true;
  success = success && initialize();
  if (success) {
    success = program();
  }
  success = view.updateDisplay(state) && success;
  success = success && deinitialize();
  return success;
}

bool RealMain::initialize() {
  bool success = true;
  LOGEVENT("RealMain initializing...");
  success = success && view.initialize();
  success = success && psuPort.initialize();
  success = success && initDipSwitch();
  success = success && getPsuVoltageSelection();

  if (!success) {
    initialized = false;
    LOGERROR("RealMain FAILED TO INITIALIZE");
  } else {
    initialized = true;
    LOGERROR("RealMain INITIALIZED SUCCESSFULLY");
  }

  return success;
}

bool RealMain::deinitialize() {
  bool success = true;
  if (initialized) {
    success = success && psuPort.deinitialize();
  }

  if (success) {
    initialized = false;
    LOGERROR("RealMain de-initialized SUCCESSFULLY");
  } else {
    LOGERROR("RealMain FAILED to de-initialize");
  }
  return success;
}

bool RealMain::initDipSwitch() {
  bool success = true;
  LOGEVENT("Initializing DipSwitch...");
  pinMode(vpmDIP_24V_PIN, INPUT_PULLUP);
  pinMode(vpmDIP_48V_PIN, INPUT_PULLUP);
  pinMode(vpmDIP_200V_PIN, INPUT_PULLUP);

  if (success) {
    LOGERROR("DipSwitch initialized SUCCESSFULLY");
  } else {
    LOGERROR("DipSwitch FAILED initialize.");
  }
  return success;
  return success;
}

bool RealMain::getPsuVoltageSelection() {
  bool success = true;
  LOGEVENT("Getting PSU Voltage Selection...");
  int DIP_24V_PIN_VAL = digitalRead(vpmDIP_24V_PIN);
  int DIP_48V_PIN_VAL = digitalRead(vpmDIP_48V_PIN);
  int DIP_200V_PIN_VAL = digitalRead(vpmDIP_200V_PIN);

  // TODO: Add check for if multiple volages are selected, or just change input
  // DIP switch to binary.
  if (DIP_24V_PIN_VAL == LOW) {
    state.selectedVoltage = getSelectedVoltage_t(vpmDIP_24V_PIN);
    LOGEVENT("24V Selected");
  } else if (DIP_48V_PIN_VAL == LOW) {
    state.selectedVoltage = getSelectedVoltage_t(vpmDIP_48V_PIN);
    LOGEVENT("48V Selected");
  } else if (DIP_200V_PIN_VAL == LOW) {
    state.selectedVoltage = getSelectedVoltage_t(vpmDIP_200V_PIN);
    LOGEVENT("200V Selected");
  } else {
    state.selectedVoltage = getSelectedVoltage_t(vpmDIP_NONE_PIN);
    state.prgState = getSelectedState_t(sNOT_READY);
    LOGEVENT("No Voltage Selected");
    success = false;
  }

  return success;
}

bool RealMain::program() {
  bool success = true;
  success = success && psuManager.programPsu(state, view);
  if (success) {
    state.prgState = getSelectedState_t(sPROGRAMMING_DONE_SUCCEEDED);
  } else {
    state.prgState = getSelectedState_t(sPROGRAMMING_DONE_FAILED);
  }
  return success;
}
