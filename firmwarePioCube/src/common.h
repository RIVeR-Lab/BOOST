#ifndef _SRC_COMMON_H_
#define _SRC_COMMON_H_

#include "pins.h"
#include <array>
#include <string>
#include "version.h"

#define LOGGING true
#define CORE_DEBUG true // For logging in arduinostm32 core library

struct SelectedVoltage_t {
  VoltageToPinMapping_t voltage;
  const char *name = "";
  bool operator==(const SelectedVoltage_t &other) const {
    return voltage == other.voltage;
  }
};
static constexpr uint32_t nSelectableVoltages = 4;
static constexpr std::array<SelectedVoltage_t, nSelectableVoltages>
    voltageToPinMapping = {{
        {vpmDIP_NONE_PIN, "NO VOLTAGE"},
        {vpmDIP_24V_PIN, "24V"},
        {vpmDIP_48V_PIN, "48V"},
        {vpmDIP_200V_PIN, "200V"},
    }};
static constexpr SelectedVoltage_t unknownVoltage = {vpmUNKNOWN_VOLTAGE, "UNKNOWN VOLTAGE"};

static constexpr SelectedVoltage_t
getSelectedVoltage_t(VoltageToPinMapping_t p) {
  for (size_t i = 0; i < voltageToPinMapping.size(); i++) {
    if (voltageToPinMapping[i].voltage == p) {
      return voltageToPinMapping[i];
    }
  }
  return unknownVoltage;
}

enum States_t {
  sREADY,
  sNOT_READY,
  sPROGRAMMING,
  sFAILED_RETRYING,
  sFACTORY_RESTTING,
  sSAVING_TO_EEPROM,
  sPROGRAMMING_DONE_FAILED,
  sPROGRAMMING_DONE_SUCCEEDED,
  sERROR,
  sUNKNOWN,
  //
  sLAST_ITEM,
  
};

struct CurrentState_t {
  States_t state;
  const char *name = "";
  bool operator==(const CurrentState_t &other) const {
    return state == other.state;
  }
};

static constexpr uint32_t nPossibleStates = 9;
static constexpr std::array<CurrentState_t, nPossibleStates> states = {{
    {sREADY, "READY"},
    {sNOT_READY, "NOT READY"},
    {sPROGRAMMING, "PROGRAMMING..."},
    {sFAILED_RETRYING, "FAILED. RETRYING..."},
    {sFACTORY_RESTTING, "FACTORY RESETTING..."},
    {sSAVING_TO_EEPROM, "SAVING TO EEPROM..."},
    {sPROGRAMMING_DONE_FAILED, "FAILED"},
    {sPROGRAMMING_DONE_SUCCEEDED, "SUCCESS"},
    {sERROR, "ERR"},
}};
static constexpr CurrentState_t unknownState = {sUNKNOWN, "UNKNOWN State"};

static constexpr CurrentState_t getSelectedState_t(States_t p) {
  for (size_t i = 0; i < states.size(); i++) {
    if (states[i].state == p) {
      return states[i];
    }
  }
  return unknownState;
}


enum Error_t {
  errUNKNOWN_STATE,
  //
  errLAST_ITEM,
  //
  errNOERROR,
  
};
struct ErrorState_t {
  Error_t err;
  const char *name = "";
  bool operator==(const ErrorState_t &other) const {
    return err == other.err;
  }
};
static constexpr uint32_t nErrors = 1;
static constexpr std::array<ErrorState_t, nErrors>
    errorStates_t = {{
        {errUNKNOWN_STATE, "Unknown State"},
    }};
static constexpr ErrorState_t noError = {errNOERROR, "None"};

static constexpr ErrorState_t
getErrorState_t(Error_t p) {
  for (size_t i = 0; i < errorStates_t.size(); i++) {
    if (errorStates_t[i].err == p) {
      return errorStates_t[i];
    }
  }
  return noError;
}

// This is global state that stores all other states
// i.e. state of the btn, state of the program, etc.
struct State_t {
  SelectedVoltage_t selectedVoltage = getSelectedVoltage_t(vpmDIP_NONE_PIN);
  CurrentState_t prgState = getSelectedState_t(sREADY); // Overall program state
  ErrorState_t errState = getErrorState_t(errNOERROR);
  std::string additionalInfo = "";
  bool operator==(const State_t &other) const {
    return (selectedVoltage == other.selectedVoltage) &&
           (prgState == other.prgState) &&(additionalInfo == other.additionalInfo);
  }

  std::string generateTextString(){
    std::string text = "";
    if(prgState.state == sERROR) {
      text.append("Err: ").append(errState.name);
    } else {
      text.append(prgState.name);
    }
    // text.append("\n\r");
    // text.append(additionalInfo);
    return text;
  }
};
#endif // _SRC_COMMON_H_