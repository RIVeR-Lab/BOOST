/*
 * HPT5K0Config.cpp
 *
 *  Created on: Aug 24, 2022
 *      Author: David Antaki
 */

#include "MantaExport/HPT5K0Config.h"

// !!!!!!!!!!!!!!!!!!!! IMPORTANT !!!!!!!!!!!!!!!!!!!!
// MAKE SURE LSB IS FIRST in the data bytes.
// i.e. { LSB, ... , MSB }
// !!!!!!!!!!!!!!!!!!!! IMPORTANT !!!!!!!!!!!!!!!!!!!!

template<const HPT5K0Info_t::UserConfigurationReg_t &regConfig>
constexpr uint16_t HPT5K0Config::generateUserConfigurationRegBits() {
  uint16_t result = 0x0;
  for (size_t i = 0; i < regConfig.size(); i++) {
    HPT5K0Info_t::UserConfigurationRegBit_t bit = regConfig[i];
    if (bit.enable) {
      result = result | (0x1 << static_cast<HPT5K0Info_t::UserConfigureationRegBitIndexType_t>(bit.bitIndex));
    } else {
      result = result & ~(0x1 << static_cast<HPT5K0Info_t::UserConfigureationRegBitIndexType_t>(bit.bitIndex));
    }
  }
  return result;
}

const std::array<uint32_t, HPT5K0Config::numPsus> HPT5K0Config::psuConfigLens = { {
    psu1ConfigLen,
    psu2ConfigLen,
    psu3ConfigLen,
    psu4ConfigLen,
    psu5ConfigLen } };

const std::array<const HPT5K0Info_t::psuCmdDefaultValues_t*, HPT5K0Config::numPsus> HPT5K0Config::psuConfigs = { {
    psu1Config,
    psu2Config,
    psu3Config,
    psu4Config,
    psu5Config } };

const std::array<const HPT5K0Config::PsuStatusRegs_t, HPT5K0Config::numPsus> HPT5K0Config::psuStatRegs = { {
    psu1StatusRegs,
    psu2StatusRegs,
    psu3StatusRegs,
    psu4StatusRegs,
    psu5StatusRegs } };

/* #################################### PSU1 Configurations #################################### */
constexpr HPT5K0Info_t::UserConfigurationReg_t HPT5K0Config::psu1UserConfigurationReg;
const uint16_t HPT5K0Config::psu1UserConfigurationRegBits =
    generateUserConfigurationRegBits<psu1UserConfigurationReg>();
static_assert(sizeof(HPT5K0Config::psu1UserConfigurationRegBits) == sizeof(uint16_t), "Must be a uint16_t because of the casting done below in psuConfig array.");
static_assert(HPT5K0Config::psu1UserConfigurationRegBits == 0x0, "Unexpected value for PSU1's User_Configuration Register setting.");
// For my own sanity
static_assert((static_cast<bool>(HPT5K0Info_t::epPsEn_t::PSU_OUTPUT_ON) == true) && ((HPT5K0Config::psu1UserConfigurationRegBits &
            (0x1 << static_cast<HPT5K0Info_t::UserConfigureationRegBitIndexType_t>(HPT5K0Info_t::UserConfigurationRegBitIndex_t::CFG_REMOTE_INHIBIT_LOGIC))) == 0x0),
    "If CFG_REMOTE_INHIBIT_LOGIC=0, then epPsEn_t::PSU_ON must =false (inhibit line is active low i.e. the PSU is inhibited when the inhibit line is LOW).");

// PSU1 = HPL5K0TS200 model set to 200V
const uint32_t HPT5K0Config::psu1ConfigLen = 12;
const HPT5K0Info_t::psuCmdDefaultValues_t HPT5K0Config::psu1Config[psu1ConfigLen] = {
    { psuCmdName_t::OPERATION, PMResponse_t::fromUInt16(HPT5K0Info_t::OPERATION_ON) },
    { psuCmdName_t::MFR_MODEL, PMResponse_t::fromBlock({ 0x48, 0x50, 0x4C, 0x35, 0x4B, 0x30, 0x54, 0x53, 0x32, 0x30, 0x30 } )},  //HPL5K0TS200
    { psuCmdName_t::MFR_MODEL, PMResponse_t::fromBlock({ 0x48, 0x50, 0x54, 0x35, 0x4B, 0x30, 0x54, 0x53, 0x32, 0x30, 0x30 } )},  //HPT5K0TS200
    { psuCmdName_t::MFR_VOUT_MAX, PMResponse_t::fromFloat(210.0f) },
    { psuCmdName_t::VOUT_COMMAND, PMResponse_t::fromFloat(200.0f) },
    { psuCmdName_t::VOUT_OV_WARN_LIMIT, PMResponse_t::fromFloat(224.0f) },  // This is factory default for the HPL5K0TS200
    { psuCmdName_t::VOUT_OV_FAULT_LIMIT, PMResponse_t::fromFloat(230.0f) },  // This is factory default for the HPL5K0TS200
    { psuCmdName_t::VOUT_UV_WARN_LIMIT, PMResponse_t::fromFloat(192.0f) },  // This is factory default for the HPL5K0TS200
    { psuCmdName_t::VOUT_UV_FAULT_LIMIT, PMResponse_t::fromFloat(190.0f) },  // This is factory default for the HPL5K0TS200
    { psuCmdName_t::USER_CONFIGURATION, PMResponse_t::fromUInt16(psu1UserConfigurationRegBits) },
    { psuCmdName_t::VOUT_MODE, PMResponse_t::fromFloat(0x18) },
    { psuCmdName_t::LAST_ITEM, { 0 } }
};

const HPT5K0Config::PsuStatusRegs_t HPT5K0Config::psu1StatusRegs = { { {
    psuCmdName_t::READ_VIN, PsuStatusIndex_t::READ_VIN },
    { psuCmdName_t::READ_VOUT, PsuStatusIndex_t::READ_VOUT },
    { psuCmdName_t::READ_IOUT, PsuStatusIndex_t::READ_IOUT },
    { psuCmdName_t::READ_TEMPERATURE_1, PsuStatusIndex_t::READ_TEMPERATURE_1 },  // In celsius
    { psuCmdName_t::READ_TEMPERATURE_2, PsuStatusIndex_t::READ_TEMPERATURE_2 },  // In celsius
    { psuCmdName_t::READ_TEMPERATURE_3, PsuStatusIndex_t::READ_TEMPERATURE_3 },  // In celsius
    { psuCmdName_t::STATUS_WORD, PsuStatusIndex_t::STATUS_WORD } } };

/* #################################### END PSU1 Configurations #################################### */

/* #################################### PSU2 Configurations #################################### */
constexpr HPT5K0Info_t::UserConfigurationReg_t HPT5K0Config::psu2UserConfigurationReg;
const uint16_t HPT5K0Config::psu2UserConfigurationRegBits =
    generateUserConfigurationRegBits<psu2UserConfigurationReg>();
static_assert(sizeof(HPT5K0Config::psu2UserConfigurationRegBits) == sizeof(uint16_t), "Must be a uint16_t because of the casting done below in psuConfig array.");
static_assert(HPT5K0Config::psu2UserConfigurationRegBits == 0x0, "Unexpected value for PSU2's User_Configuration Register setting.");
// For my own sanity
static_assert((static_cast<bool>(HPT5K0Info_t::epPsEn_t::PSU_OUTPUT_ON) == true) && ((HPT5K0Config::psu2UserConfigurationRegBits &
            (0x1 << static_cast<HPT5K0Info_t::UserConfigureationRegBitIndexType_t>(HPT5K0Info_t::UserConfigurationRegBitIndex_t::CFG_REMOTE_INHIBIT_LOGIC))) == 0x0),
    "If CFG_REMOTE_INHIBIT_LOGIC=0, then epPsEn_t::PSU_ON must =false (inhibit line is active low i.e. the PSU is inhibited when the inhibit line is LOW).");

// PSU2 = HPT5K0TS048 model set to 48V
const uint32_t HPT5K0Config::psu2ConfigLen = 12;
const HPT5K0Info_t::psuCmdDefaultValues_t HPT5K0Config::psu2Config[psu2ConfigLen] = {
    { psuCmdName_t::OPERATION, PMResponse_t::fromUInt16(HPT5K0Info_t::OPERATION_ON) },
    { psuCmdName_t::MFR_MODEL, PMResponse_t::fromBlock({ 0x48, 0x50, 0x4C, 0x35, 0x4B, 0x30, 0x54, 0x53, 0x30, 0x34, 0x38 }) },  //HPL5K0TS048
    { psuCmdName_t::MFR_MODEL, PMResponse_t::fromBlock({ 0x48, 0x50, 0x54, 0x35, 0x4B, 0x30, 0x54, 0x53, 0x30, 0x34, 0x38 }) },  //HPT5K0TS048
    { psuCmdName_t::MFR_VOUT_MAX, PMResponse_t::fromFloat(50.4f) },
    { psuCmdName_t::VOUT_COMMAND, PMResponse_t::fromFloat(48.0f) },  // This is factory default for the HPT5K0TS048
    { psuCmdName_t::VOUT_OV_WARN_LIMIT, PMResponse_t::fromFloat(54.0f) }, // This is factory default for the HPT5K0TS048
    { psuCmdName_t::VOUT_OV_FAULT_LIMIT, PMResponse_t::fromFloat(56.0f) },  // This is factory default for the HPT5K0TS048
    { psuCmdName_t::VOUT_UV_WARN_LIMIT, PMResponse_t::fromFloat(46.0f) },  // This is factory default for the HPT5K0TS048
    { psuCmdName_t::VOUT_UV_FAULT_LIMIT, PMResponse_t::fromFloat(45.0f) },  // This is factory default for the HPT5K0TS048
    { psuCmdName_t::USER_CONFIGURATION, PMResponse_t::fromUInt16(psu2UserConfigurationRegBits) },
    { psuCmdName_t::VOUT_MODE, PMResponse_t::fromFloat(0x18) },
    { psuCmdName_t::LAST_ITEM, { 0 } } };

const HPT5K0Config::PsuStatusRegs_t HPT5K0Config::psu2StatusRegs = { { {
    psuCmdName_t::READ_VIN, PsuStatusIndex_t::READ_VIN },
    { psuCmdName_t::READ_VOUT, PsuStatusIndex_t::READ_VOUT },
    { psuCmdName_t::READ_IOUT, PsuStatusIndex_t::READ_IOUT },
    { psuCmdName_t::READ_TEMPERATURE_1, PsuStatusIndex_t::READ_TEMPERATURE_1 },  // In celsius
    { psuCmdName_t::READ_TEMPERATURE_2, PsuStatusIndex_t::READ_TEMPERATURE_2 },  // In celsius
    { psuCmdName_t::READ_TEMPERATURE_3, PsuStatusIndex_t::READ_TEMPERATURE_3 },  // In celsius
    { psuCmdName_t::STATUS_WORD, PsuStatusIndex_t::STATUS_WORD } } };

/* #################################### END PSU2 Configurations #################################### */

/* #################################### PSU3 Configurations #################################### */
constexpr HPT5K0Info_t::UserConfigurationReg_t HPT5K0Config::psu3UserConfigurationReg;
const uint16_t HPT5K0Config::psu3UserConfigurationRegBits =
    generateUserConfigurationRegBits<psu3UserConfigurationReg>();
static_assert(sizeof(HPT5K0Config::psu3UserConfigurationRegBits) == sizeof(uint16_t), "Must be a uint16_t because of the casting done below in psuConfig array.");
static_assert(HPT5K0Config::psu3UserConfigurationRegBits == 0x0, "Unexpected value for PSU3's User_Configuration Register setting.");
// For my own sanity
static_assert((static_cast<bool>(HPT5K0Info_t::epPsEn_t::PSU_OUTPUT_ON) == true) && ((HPT5K0Config::psu3UserConfigurationRegBits &
            (0x1 << static_cast<HPT5K0Info_t::UserConfigureationRegBitIndexType_t>(HPT5K0Info_t::UserConfigurationRegBitIndex_t::CFG_REMOTE_INHIBIT_LOGIC))) == 0x0),
    "If CFG_REMOTE_INHIBIT_LOGIC=0, then epPsEn_t::PSU_ON must =false (inhibit line is active low i.e. the PSU is inhibited when the inhibit line is LOW).");

// PSU3 = HPT5K0TS048 model set to 48V
const uint32_t HPT5K0Config::psu3ConfigLen = 12;
const HPT5K0Info_t::psuCmdDefaultValues_t HPT5K0Config::psu3Config[psu3ConfigLen] = {
    { psuCmdName_t::OPERATION, PMResponse_t::fromUInt16(HPT5K0Info_t::OPERATION_ON) },
    { psuCmdName_t::MFR_MODEL, PMResponse_t::fromBlock({ 0x48, 0x50, 0x4C, 0x35, 0x4B, 0x30, 0x54, 0x53, 0x30, 0x34, 0x38 }) },  //HPL5K0TS048
    { psuCmdName_t::MFR_MODEL, PMResponse_t::fromBlock({ 0x48, 0x50, 0x54, 0x35, 0x4B, 0x30, 0x54, 0x53, 0x30, 0x34, 0x38 }) },  //HPT5K0TS048
    { psuCmdName_t::MFR_VOUT_MAX, PMResponse_t::fromFloat(50.4f) },
    { psuCmdName_t::VOUT_COMMAND, PMResponse_t::fromFloat(48.0f) },  // This is factory default for the HPT5K0TS048
    { psuCmdName_t::VOUT_OV_WARN_LIMIT, PMResponse_t::fromFloat(54.0f) },  // This is factory default for the HPT5K0TS048
    { psuCmdName_t::VOUT_OV_FAULT_LIMIT, PMResponse_t::fromFloat(56.0f) },  // This is factory default for the HPT5K0TS048
    { psuCmdName_t::VOUT_UV_WARN_LIMIT, PMResponse_t::fromFloat(46.0f) },  // This is factory default for the HPT5K0TS048
    { psuCmdName_t::VOUT_UV_FAULT_LIMIT, PMResponse_t::fromFloat(45.0f) },  // This is factory default for the HPT5K0TS048
    { psuCmdName_t::USER_CONFIGURATION, PMResponse_t::fromUInt16(psu3UserConfigurationRegBits) },
    { psuCmdName_t::VOUT_MODE, PMResponse_t::fromFloat(0x18) },
    { psuCmdName_t::LAST_ITEM, { 0 } } };

const HPT5K0Config::PsuStatusRegs_t HPT5K0Config::psu3StatusRegs = { { {
    psuCmdName_t::READ_VIN, PsuStatusIndex_t::READ_VIN },
    { psuCmdName_t::READ_VOUT, PsuStatusIndex_t::READ_VOUT },
    { psuCmdName_t::READ_IOUT, PsuStatusIndex_t::READ_IOUT },
    { psuCmdName_t::READ_TEMPERATURE_1, PsuStatusIndex_t::READ_TEMPERATURE_1 },  // In celsius
    { psuCmdName_t::READ_TEMPERATURE_2, PsuStatusIndex_t::READ_TEMPERATURE_2 },  // In celsius
    { psuCmdName_t::READ_TEMPERATURE_3, PsuStatusIndex_t::READ_TEMPERATURE_3 },  // In celsius
    { psuCmdName_t::STATUS_WORD, PsuStatusIndex_t::STATUS_WORD } } };
/* #################################### END PSU3 Configurations #################################### */

/* #################################### PSU4 Configurations #################################### */
constexpr HPT5K0Info_t::UserConfigurationReg_t HPT5K0Config::psu4UserConfigurationReg;
const uint16_t HPT5K0Config::psu4UserConfigurationRegBits =
    generateUserConfigurationRegBits<psu4UserConfigurationReg>();
static_assert(sizeof(HPT5K0Config::psu4UserConfigurationRegBits) == sizeof(uint16_t), "Must be a uint16_t because of the casting done below in psuConfig array.");
static_assert(HPT5K0Config::psu4UserConfigurationRegBits == 0x0, "Unexpected value for PSU4's User_Configuration Register setting.");
// For my own sanity
static_assert((static_cast<bool>(HPT5K0Info_t::epPsEn_t::PSU_OUTPUT_ON) == true) && ((HPT5K0Config::psu4UserConfigurationRegBits &
            (0x1 << static_cast<HPT5K0Info_t::UserConfigureationRegBitIndexType_t>(HPT5K0Info_t::UserConfigurationRegBitIndex_t::CFG_REMOTE_INHIBIT_LOGIC))) == 0x0),
    "If CFG_REMOTE_INHIBIT_LOGIC=0, then epPsEn_t::PSU_ON must =false (inhibit line is active low i.e. the PSU is inhibited when the inhibit line is LOW).");

// PSU4 = HPT5K0TS048 model set to 24V
const uint32_t HPT5K0Config::psu4ConfigLen = 12;
const HPT5K0Info_t::psuCmdDefaultValues_t HPT5K0Config::psu4Config[psu4ConfigLen] = {
    { psuCmdName_t::OPERATION, PMResponse_t::fromUInt16(HPT5K0Info_t::OPERATION_ON) },
    { psuCmdName_t::MFR_MODEL, PMResponse_t::fromBlock({ 0x48, 0x50, 0x4C, 0x35, 0x4B, 0x30, 0x54, 0x53, 0x30, 0x34, 0x38 }) },  //HPL5K0TS048
    { psuCmdName_t::MFR_MODEL, PMResponse_t::fromBlock({ 0x48, 0x50, 0x54, 0x35, 0x4B, 0x30, 0x54, 0x53, 0x30, 0x34, 0x38 }) },  //HPT5K0TS048
    { psuCmdName_t::MFR_VOUT_MAX, PMResponse_t::fromFloat(50.4f) },
    { psuCmdName_t::VOUT_COMMAND, PMResponse_t::fromFloat(24.0f) },
    { psuCmdName_t::VOUT_OV_WARN_LIMIT, PMResponse_t::fromFloat(26.0f) },
    { psuCmdName_t::VOUT_OV_FAULT_LIMIT, PMResponse_t::fromFloat(28.0f) },
    { psuCmdName_t::VOUT_UV_WARN_LIMIT, PMResponse_t::fromFloat(22.0f) },
    { psuCmdName_t::VOUT_UV_FAULT_LIMIT, PMResponse_t::fromFloat(20.0f) },
    { psuCmdName_t::USER_CONFIGURATION, PMResponse_t::fromUInt16(psu4UserConfigurationRegBits) },
    { psuCmdName_t::VOUT_MODE, PMResponse_t::fromFloat(0x18) },
    { psuCmdName_t::LAST_ITEM, { 0 } }
};

const HPT5K0Config::PsuStatusRegs_t HPT5K0Config::psu4StatusRegs = { { {
    psuCmdName_t::READ_VIN, PsuStatusIndex_t::READ_VIN },
    { psuCmdName_t::READ_VOUT, PsuStatusIndex_t::READ_VOUT },
    { psuCmdName_t::READ_IOUT, PsuStatusIndex_t::READ_IOUT },
    { psuCmdName_t::READ_TEMPERATURE_1, PsuStatusIndex_t::READ_TEMPERATURE_1 },  // In celsius
    { psuCmdName_t::READ_TEMPERATURE_2, PsuStatusIndex_t::READ_TEMPERATURE_2 },  // In celsius
    { psuCmdName_t::READ_TEMPERATURE_3, PsuStatusIndex_t::READ_TEMPERATURE_3 },  // In celsius
    { psuCmdName_t::STATUS_WORD, PsuStatusIndex_t::STATUS_WORD } } };
/* #################################### END PSU4 Configurations #################################### */

/* #################################### PSU5 Configurations #################################### */
constexpr HPT5K0Info_t::UserConfigurationReg_t HPT5K0Config::psu5UserConfigurationReg;
const uint16_t HPT5K0Config::psu5UserConfigurationRegBits =
    generateUserConfigurationRegBits<psu5UserConfigurationReg>();
static_assert(sizeof(HPT5K0Config::psu5UserConfigurationRegBits) == sizeof(uint16_t), "Must be a uint16_t because of the casting done below in psuConfig array.");
static_assert(HPT5K0Config::psu5UserConfigurationRegBits == 0x0, "Unexpected value for PSU4's User_Configuration Register setting.");
// For my own sanity
static_assert((static_cast<bool>(HPT5K0Info_t::epPsEn_t::PSU_OUTPUT_ON) == true) && ((HPT5K0Config::psu5UserConfigurationRegBits &
            (0x1 << static_cast<HPT5K0Info_t::UserConfigureationRegBitIndexType_t>(HPT5K0Info_t::UserConfigurationRegBitIndex_t::CFG_REMOTE_INHIBIT_LOGIC))) == 0x0),
    "If CFG_REMOTE_INHIBIT_LOGIC=0, then epPsEn_t::PSU_ON must =false (inhibit line is active low i.e. the PSU is inhibited when the inhibit line is LOW).");

// PSU5 = HPT5K0TS048 model set to 24V
const uint32_t HPT5K0Config::psu5ConfigLen = 12;
const HPT5K0Info_t::psuCmdDefaultValues_t HPT5K0Config::psu5Config[psu5ConfigLen] = {
    { psuCmdName_t::OPERATION, PMResponse_t::fromUInt16(HPT5K0Info_t::OPERATION_ON) },
    { psuCmdName_t::MFR_MODEL, PMResponse_t::fromBlock({ 0x48, 0x50, 0x4C, 0x35, 0x4B, 0x30, 0x54, 0x53, 0x30, 0x34, 0x38 } )},  //HPL5K0TS048
    { psuCmdName_t::MFR_MODEL, PMResponse_t::fromBlock({ 0x48, 0x50, 0x54, 0x35, 0x4B, 0x30, 0x54, 0x53, 0x30, 0x34, 0x38 } )},  //HPT5K0TS048
    { psuCmdName_t::MFR_VOUT_MAX, PMResponse_t::fromFloat(50.4f) },
    { psuCmdName_t::VOUT_COMMAND, PMResponse_t::fromFloat(24.0f) },
    { psuCmdName_t::VOUT_OV_WARN_LIMIT, PMResponse_t::fromFloat(26.0f) },
    { psuCmdName_t::VOUT_OV_FAULT_LIMIT, PMResponse_t::fromFloat(28.0f) },
    { psuCmdName_t::VOUT_UV_WARN_LIMIT, PMResponse_t::fromFloat(22.0f) },
    { psuCmdName_t::VOUT_UV_FAULT_LIMIT, PMResponse_t::fromFloat(20.0f) },
    { psuCmdName_t::USER_CONFIGURATION, PMResponse_t::fromUInt16(psu5UserConfigurationRegBits) },
    { psuCmdName_t::VOUT_MODE, PMResponse_t::fromFloat(0x18) },
    { psuCmdName_t::LAST_ITEM, { 0 } } };

const HPT5K0Config::PsuStatusRegs_t HPT5K0Config::psu5StatusRegs = { { {
    psuCmdName_t::READ_VIN, PsuStatusIndex_t::READ_VIN },
    { psuCmdName_t::READ_VOUT, PsuStatusIndex_t::READ_VOUT },
    { psuCmdName_t::READ_IOUT, PsuStatusIndex_t::READ_IOUT },
    { psuCmdName_t::READ_TEMPERATURE_1, PsuStatusIndex_t::READ_TEMPERATURE_1 },  // In celsius
    { psuCmdName_t::READ_TEMPERATURE_2, PsuStatusIndex_t::READ_TEMPERATURE_2 },  // In celsius
    { psuCmdName_t::READ_TEMPERATURE_3, PsuStatusIndex_t::READ_TEMPERATURE_3 },  // In celsius
    { psuCmdName_t::STATUS_WORD, PsuStatusIndex_t::STATUS_WORD } } };
/* #################################### END PSU5 Configurations #################################### */
