/*
 * HPT5K0Config.h
 *
 *  Created on: Aug 24, 2022
 *      Author: David Antaki
 */

#ifndef SRC_MANTAHW_HPT5K0Setup_H_
#define SRC_MANTAHW_HPT5K0Setup_H_

#include <MantaExport/HPT5K0Info.h>
#include "MantaExport/HPT5K0DolphinInfo.h"
#include <stdint.h>
#include <array>

/**
 * These are the settings that we want for each power supply.
 * These settings will be checked on HPT5K0 initialization and if any of the
 * PSU's current settings do not match the current ones, then they will be set and saved to the EEPROM.
 * This is somewhat modeled after ServiceStationSetup.h/cpp
 */
class HPT5K0Config {
 public:
  HPT5K0Config() {
  }

  virtual ~HPT5K0Config() {
  }

 private:
  using psuCmdName_t = HPT5K0DolphinInfo_t::psuCmdName_t;
  using PsuStatusRegs_t = HPT5K0DolphinInfo_t::PsuStatusRegs_t;
  using psuCmdDefaultValues_t = HPT5K0Info_t::psuCmdDefaultValues_t;
  using UserConfigurationReg_t = HPT5K0Info_t::UserConfigurationReg_t;
  using UserConfigurationRegBitIndex_t = HPT5K0Info_t::UserConfigurationRegBitIndex_t;
  using PsuStatusIndex_t = HPT5K0DolphinInfo_t::PsuStatusIndex_t;
  using PMResponse_t = HPT5K0Info_t::PMResponse_t;

 public:
  static const uint16_t psu1UserConfigurationRegBits;
  static const uint16_t psu2UserConfigurationRegBits;
  static const uint16_t psu3UserConfigurationRegBits;
  static const uint16_t psu4UserConfigurationRegBits;
  static const uint16_t psu5UserConfigurationRegBits;
  static const uint32_t numPsus = 5;
  static const std::array<uint32_t, numPsus> psuConfigLens;
  static const std::array<const HPT5K0Info_t::psuCmdDefaultValues_t*, numPsus> psuConfigs;
  static const std::array<const PsuStatusRegs_t, numPsus> psuStatRegs;

 private:
  // These are C style arrays so that they can be of different length and be passed to constructor of HPT5K0
  // The "Config" arrays below get checked on boot, but not after.
  // Any configuration register that appears 2+ times in these arrays but with different values will be treated as an 'OR' operation.
  // ONLY read-only registers can have duplicates however.
  // e.g. if MFR_MODEL appears 2x, with different model numbers, then either model number will be accepted when checked on boot.
  static const HPT5K0Info_t::psuCmdDefaultValues_t psu1Config[];
  static const HPT5K0Info_t::psuCmdDefaultValues_t psu2Config[];
  static const HPT5K0Info_t::psuCmdDefaultValues_t psu3Config[];
  static const HPT5K0Info_t::psuCmdDefaultValues_t psu4Config[];
  static const HPT5K0Info_t::psuCmdDefaultValues_t psu5Config[];
  static const uint32_t psu1ConfigLen;
  static const uint32_t psu2ConfigLen;
  static const uint32_t psu3ConfigLen;
  static const uint32_t psu4ConfigLen;
  static const uint32_t psu5ConfigLen;

  // The "StatusRegs" arrays below do not get checked on boot, but get checked periodically during runtime.
  // These are the PSU registers that will be sent in the DeltaLogger.
  static const PsuStatusRegs_t psu1StatusRegs;
  static const PsuStatusRegs_t psu2StatusRegs;
  static const PsuStatusRegs_t psu3StatusRegs;
  static const PsuStatusRegs_t psu4StatusRegs;
  static const PsuStatusRegs_t psu5StatusRegs;

  template<const UserConfigurationReg_t &regConfig>
  static constexpr uint16_t generateUserConfigurationRegBits();
  static constexpr PMResponse_t fromFloat(const uint8_t cmd, float f);

  /* #################################### PSU1 Configurations #################################### */
  static constexpr UserConfigurationReg_t psu1UserConfigurationReg = { { {
      UserConfigurationRegBitIndex_t::CFG_CURRENT_SOFTSTART_ENABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_FAST_SOFTSTART_DISABLE, false }, {
      UserConfigurationRegBitIndex_t::CFG_PRELOAD_DISABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_FAN_OFF, false }, {
      UserConfigurationRegBitIndex_t::CFG_ACOK_SIG_LOGIC,
      false }, { UserConfigurationRegBitIndex_t::CFG_DCOK_SIG_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::RESERVED1,
      false }, { UserConfigurationRegBitIndex_t::FG_FAN_TEMP_OK_SIG_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::RESERVED2,
      false }, { UserConfigurationRegBitIndex_t::CFG_REMOTE_INHIBIT_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::CFG_POTENTIOMETER_DISABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_POTENTIOMETER_FULL_ADJ, false }, {
      UserConfigurationRegBitIndex_t::CFG_ANALOG_PROG,
      false }, { UserConfigurationRegBitIndex_t::CFG_DISABLE_IPROG, false }, {
      UserConfigurationRegBitIndex_t::RESERVED3,
      false }, { UserConfigurationRegBitIndex_t::RESERVED4, false } } };
  /* #################################### PSU1 Configurations #################################### */

  /* #################################### PSU2 Configurations #################################### */
  static constexpr UserConfigurationReg_t psu2UserConfigurationReg = { { {
      UserConfigurationRegBitIndex_t::CFG_CURRENT_SOFTSTART_ENABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_FAST_SOFTSTART_DISABLE, false }, {
      UserConfigurationRegBitIndex_t::CFG_PRELOAD_DISABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_FAN_OFF, false }, {
      UserConfigurationRegBitIndex_t::CFG_ACOK_SIG_LOGIC,
      false }, { UserConfigurationRegBitIndex_t::CFG_DCOK_SIG_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::RESERVED1,
      false }, { UserConfigurationRegBitIndex_t::FG_FAN_TEMP_OK_SIG_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::RESERVED2,
      false }, { UserConfigurationRegBitIndex_t::CFG_REMOTE_INHIBIT_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::CFG_POTENTIOMETER_DISABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_POTENTIOMETER_FULL_ADJ, false }, {
      UserConfigurationRegBitIndex_t::CFG_ANALOG_PROG,
      false }, { UserConfigurationRegBitIndex_t::CFG_DISABLE_IPROG, false }, {
      UserConfigurationRegBitIndex_t::RESERVED3,
      false }, { UserConfigurationRegBitIndex_t::RESERVED4, false } } };
  /* #################################### PSU2 Configurations #################################### */

  /* #################################### PSU3 Configurations #################################### */
  static constexpr UserConfigurationReg_t psu3UserConfigurationReg = { { {
      UserConfigurationRegBitIndex_t::CFG_CURRENT_SOFTSTART_ENABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_FAST_SOFTSTART_DISABLE, false }, {
      UserConfigurationRegBitIndex_t::CFG_PRELOAD_DISABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_FAN_OFF, false }, {
      UserConfigurationRegBitIndex_t::CFG_ACOK_SIG_LOGIC,
      false }, { UserConfigurationRegBitIndex_t::CFG_DCOK_SIG_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::RESERVED1,
      false }, { UserConfigurationRegBitIndex_t::FG_FAN_TEMP_OK_SIG_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::RESERVED2,
      false }, { UserConfigurationRegBitIndex_t::CFG_REMOTE_INHIBIT_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::CFG_POTENTIOMETER_DISABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_POTENTIOMETER_FULL_ADJ, false }, {
      UserConfigurationRegBitIndex_t::CFG_ANALOG_PROG,
      false }, { UserConfigurationRegBitIndex_t::CFG_DISABLE_IPROG, false }, {
      UserConfigurationRegBitIndex_t::RESERVED3,
      false }, { UserConfigurationRegBitIndex_t::RESERVED4, false } } };
  /* #################################### PSU3 Configurations #################################### */

  /* #################################### PSU4 Configurations #################################### */
  static constexpr UserConfigurationReg_t psu4UserConfigurationReg = { { {
      UserConfigurationRegBitIndex_t::CFG_CURRENT_SOFTSTART_ENABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_FAST_SOFTSTART_DISABLE, false }, {
      UserConfigurationRegBitIndex_t::CFG_PRELOAD_DISABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_FAN_OFF, false }, {
      UserConfigurationRegBitIndex_t::CFG_ACOK_SIG_LOGIC,
      false }, { UserConfigurationRegBitIndex_t::CFG_DCOK_SIG_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::RESERVED1,
      false }, { UserConfigurationRegBitIndex_t::FG_FAN_TEMP_OK_SIG_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::RESERVED2,
      false }, { UserConfigurationRegBitIndex_t::CFG_REMOTE_INHIBIT_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::CFG_POTENTIOMETER_DISABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_POTENTIOMETER_FULL_ADJ, false }, {
      UserConfigurationRegBitIndex_t::CFG_ANALOG_PROG,
      false }, { UserConfigurationRegBitIndex_t::CFG_DISABLE_IPROG, false }, {
      UserConfigurationRegBitIndex_t::RESERVED3,
      false }, { UserConfigurationRegBitIndex_t::RESERVED4, false } } };
  /* #################################### PSU4 Configurations #################################### */

  /* #################################### PSU5 Configurations #################################### */
  static constexpr UserConfigurationReg_t psu5UserConfigurationReg = { { {
      UserConfigurationRegBitIndex_t::CFG_CURRENT_SOFTSTART_ENABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_FAST_SOFTSTART_DISABLE, false }, {
      UserConfigurationRegBitIndex_t::CFG_PRELOAD_DISABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_FAN_OFF, false }, {
      UserConfigurationRegBitIndex_t::CFG_ACOK_SIG_LOGIC,
      false }, { UserConfigurationRegBitIndex_t::CFG_DCOK_SIG_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::RESERVED1,
      false }, { UserConfigurationRegBitIndex_t::FG_FAN_TEMP_OK_SIG_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::RESERVED2,
      false }, { UserConfigurationRegBitIndex_t::CFG_REMOTE_INHIBIT_LOGIC, false }, {
      UserConfigurationRegBitIndex_t::CFG_POTENTIOMETER_DISABLE,
      false }, { UserConfigurationRegBitIndex_t::CFG_POTENTIOMETER_FULL_ADJ, false }, {
      UserConfigurationRegBitIndex_t::CFG_ANALOG_PROG,
      false }, { UserConfigurationRegBitIndex_t::CFG_DISABLE_IPROG, false }, {
      UserConfigurationRegBitIndex_t::RESERVED3,
      false }, { UserConfigurationRegBitIndex_t::RESERVED4, false } } };
  /* #################################### PSU5 Configurations #################################### */

};

#endif /* SRC_MANTAHW_HPT5K0Setup_H_ */
