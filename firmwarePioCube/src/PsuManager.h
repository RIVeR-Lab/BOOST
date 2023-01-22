#ifndef _SRC_PSUMANAGER_H
#define _SRC_PSUMANAGER_H

#include "Board2Board/Logging.h"
#include "MantaExport/HPT5K0Config.h"
#include "MantaExport/HPT5K0DolphinInfo.h"
#include "OrcaHardware/HPT5K0.h"
#include "common.h"
#include "pins.h"
#include <MantaExport/HPT5K0Info.h>
#include <memory>
#include "MantaHW/I2cBase.h"
#include "View/ePaper.h"

class PsuManager {
public:
  PsuManager(Logging &log, I2cBase &i2cBus)
      : voltageToPSUMappings_t({{
            {vpmDIP_24V_PIN,
              HPT5K0(log, i2cBus, static_cast<HPT5K0::ePsuI2cAddr>(defaultPsuIICAddr),
                    HPT5K0DolphinInfo_t::iPSU4,
                    HPT5K0Config::psuConfigs[ePsuIndices::iPSU4],
                    HPT5K0Config::psuConfigLens[HPT5K0DolphinInfo_t::iPSU4],
                    Manta::LogSpecs::PSU4_STATUS,
                    HPT5K0Config::psuStatRegs[ePsuIndices::iPSU4])},
            {vpmDIP_48V_PIN,
              HPT5K0(log, i2cBus, static_cast<HPT5K0::ePsuI2cAddr>(defaultPsuIICAddr),
                    HPT5K0DolphinInfo_t::iPSU2,
                    HPT5K0Config::psuConfigs[ePsuIndices::iPSU2],
                    HPT5K0Config::psuConfigLens[HPT5K0DolphinInfo_t::iPSU2],
                    Manta::LogSpecs::PSU2_STATUS,
                    HPT5K0Config::psuStatRegs[ePsuIndices::iPSU2])},
            {vpmDIP_200V_PIN,
              HPT5K0(log, i2cBus, static_cast<HPT5K0::ePsuI2cAddr>(defaultPsuIICAddr),
                    HPT5K0DolphinInfo_t::iPSU1,
                    HPT5K0Config::psuConfigs[ePsuIndices::iPSU1],
                    HPT5K0Config::psuConfigLens[HPT5K0DolphinInfo_t::iPSU1],
                    Manta::LogSpecs::PSU1_STATUS,
                    HPT5K0Config::psuStatRegs[ePsuIndices::iPSU1])},
        }})
        {}
  ~PsuManager() {}
  bool programPsu(State_t &s, ePaper &view);

private:
  using PsuStatusRegs_t = HPT5K0DolphinInfo_t::PsuStatusRegs_t;
  using psuCmdName_t = HPT5K0DolphinInfo_t::psuCmdName_t;
  using psuCmd_t = HPT5K0Info_t::psuCmd_t;
  using ePsuIndices = HPT5K0DolphinInfo_t::ePsuIndices;
  using StatusWordRegBitIndex_t = HPT5K0DolphinInfo_t::StatusWordRegBitIndex_t;

  static constexpr uint32_t msSecPsuTurnONTimeout = 3000;
  static constexpr unsigned char defaultPsuIICAddr = 0x5F;
  static const uint32_t programXpPsusMaxRetries = 3;

  // Mapping from SelectedVoltage_t to the HPT5K0 config that sets that voltage.
  struct voltageToPSUMapping_t {
    VoltageToPinMapping_t voltage;
    HPT5K0 psu;
  };
  // Subtract 1 because we won't map DIP_NONE_PIN to a PSU.
  static constexpr uint32_t nVoltageToPSUMappings = nSelectableVoltages - 1;
  std::array<voltageToPSUMapping_t, nVoltageToPSUMappings>
      voltageToPSUMappings_t;

  bool getvoltageToPSUMapping_t(VoltageToPinMapping_t voltage,
                                          uint32_t &index);
};

#endif // _SRC_PSUMANAGER_H