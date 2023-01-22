#include "PsuManager.h"

bool PsuManager::getvoltageToPSUMapping_t(VoltageToPinMapping_t voltage,
                                          uint32_t &index) {
  for (size_t i = 0; i < nVoltageToPSUMappings; i++) {
    if (voltageToPSUMappings_t[i].voltage == voltage) {
      index = i;
      return true;
    }
  }
  LOGERROR("No voltageToPSUMapping_t for voltage %d", voltage);
  return false;
} 

bool PsuManager::programPsu(State_t &s, ePaper &view) {
  bool success = true;
  uint32_t voltageToPSUMappingsI = 9999;

  // Get the PSU based on selected voltage.
  switch (s.selectedVoltage.voltage) {
    case vpmDIP_NONE_PIN: {
      LOGERROR("No PSU selected.");
      success = false;
    } break;
    case vpmDIP_24V_PIN: {
      success = success &&
                getvoltageToPSUMapping_t(vpmDIP_24V_PIN, voltageToPSUMappingsI);
    } break;
    case vpmDIP_48V_PIN: {
      success = success &&
                getvoltageToPSUMapping_t(vpmDIP_48V_PIN, voltageToPSUMappingsI);
    } break;
    case vpmDIP_200V_PIN: {
      success = success &&
                getvoltageToPSUMapping_t(vpmDIP_200V_PIN, voltageToPSUMappingsI);
    } break;
    default: {
      LOGERROR("Invalid voltage selected.");
      success = false;
    } break;
  };

  if (success) {
    s.prgState = getSelectedState_t(sFACTORY_RESTTING);
    view.updateDisplay(s);

    if (success) {
      s.prgState = getSelectedState_t(sPROGRAMMING);
      s.additionalInfo = "This may take up to 1 minute.";
      view.updateDisplay(s);
      s.additionalInfo = "";

      bool prgSuccess = false;
      // Reconfigure PSU, then check that it was done and saved to EEPROM
      // successfully.
      for (size_t i = 0; !prgSuccess && i < programXpPsusMaxRetries; i++) {
        prgSuccess = voltageToPSUMappings_t[voltageToPSUMappingsI].psu.reconfigurePsu(s, view);

        if (!prgSuccess) {
          LOGERROR("Programming PSU failed. Retrying...");
          s.prgState = getSelectedState_t(sFAILED_RETRYING);
          s.additionalInfo = "";
          view.updateDisplay(s);
        }
      }
      s.additionalInfo = "";
      success = prgSuccess;
    }
  }

  // Restore user settings so that we are in known state on programmer
  // disconnect.
  if (success) {
    HPT5K0Info_t::PMResponse_t data = HPT5K0Info_t::PMResponse_t::fromUInt16(1);
    success = success && voltageToPSUMappings_t[voltageToPSUMappingsI].psu.Write(psuCmdName_t::RESTORE_USER_ALL, data, true);
  }

  return success;
}