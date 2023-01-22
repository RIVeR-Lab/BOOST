/*
 * HPT5K0Info.h
 *
 * Power Supply stuff NOT relevant to Dolphin.
 *
 *  Created on: Sep 6, 2022
 *      Author: DavidAntaki
 */

#ifndef SRC_MANTAEXPORT_HPT5K0INFO_H_
#define SRC_MANTAEXPORT_HPT5K0INFO_H_

#include <stdint.h>
#include <array>
#include <string.h>
#include "HPT5K0DolphinInfo.h"
#include <math.h>

// msvc & gcc pack syntax is different
#ifdef _MSC_VER
__pragma(pack(push, 1))
// disable GCC pack inst
#define __attribute__(...)
#endif

// Type for reporting PSU status
struct __attribute__((packed))HPT5K0Info_t {
 public:
  HPT5K0Info_t() {
  }

  /* ########################### COMMANDS RELATED ########################### */

  using psuCmdName_t = HPT5K0DolphinInfo_t::psuCmdName_t;

  static const uint32_t NUM_CMDS = psuCmdName_t::LAST_ITEM;
  static const uint32_t NUM_READ_CMDS = 93;
  static const uint32_t TOTAL_REG_LEN = 353;  // Sum of all the PsuCMD::regLen contained in PsuCmdsDefaults
  using regData_t = std::array<uint8_t, TOTAL_REG_LEN>;

  enum regAccess_t
    : uint8_t {
      RO,  // Read-only
    RW,  // Read-write
    WO,  // Write-only
    ROE,  // Read-only and can be stored in EEPROM using STORE_USER_ALL command (this memory will be used as default at power up).
    RWE  // Read-write and can be stored in EEPROM using STORE_USER_ALL command (this memory will be used as default at power up).
  };

  // There are 3 overarching PSU register types that dictate the register formatting and how the register
  // is to be interpretted.
  enum eRegTypes
    : uint8_t {
      etOutVoltLinearFormat,  // For registers in linear format that ARE output voltage registers (see datasheet 3.11 Data Format for Output Voltage). Formatted value stored as float.
    etNonOutVoltLinearFormat,  // For registers in linear format that are NOT output voltage registers (see datasheet 3.12 Data Format for Other Parameters). Formatted value stored as float.
    etBlock,  // Block Registers (Just a stream of bytes) (see datasheet 3.9 and 3.10 PMBus Block Read/Write Packet Structure). Formatted Value stored as uint8_t array.
    etOther,  // Other Registers which are neither in linear format (both as a output-voltage register or a non-output-voltage register), nor block format. Formatted value stored as uint16_t
  };

  struct psuCmd_t {
    psuCmdName_t index;
    uint8_t cmdCode;
    uint8_t regLen;  // In bytes
    regAccess_t regAccess;
    uint32_t dataStartIndex;  // Start index of the data of this CMD in variables regData. Must be initialized at construction.
    eRegTypes regType;
    const char* cmdNameStr;
    // need operator< for std::sort()
    bool operator<(const psuCmd_t& rhs) {
      return index < rhs.index;
    }
  };
  using psuCmds_t = std::array<psuCmd_t, NUM_CMDS>;

  static const uint32_t DEFAULT_VALUE_SIZE = 32;

  // The response type gotten back from the PSU.
  struct PMResponse_t {
    union PMResponseType_t {
      uint8_t block[DEFAULT_VALUE_SIZE]{};
      uint16_t i;
      float f;
    } data;

    bool equal(const PMResponse_t &that, float epsilon) const {
      if (this->tag != that.tag) {
        return false;
      }

      switch (this->tag) {
        case eBlock: {
          for (size_t i = 0; i < sizeof(this->data.block); i++) {
            if (this->data.block[i] != that.data.block[i]) {
              return false;
            }
          }
        }
          break;
        case eUint16: {
          return this->data.i == that.data.i;
        }
          break;
        case eFloat: {
          return fabs(this->data.f - that.data.f) <= epsilon;
        }
          break;
        case eNone: {
          return true;
        }
          break;
        default: {
          return false;
        }
          break;
      };
      return true;
    }

    enum eTags_t {
      eBlock,
      eUint16,
      eFloat,
      eNone
    };
    eTags_t tag = eBlock;  // The currently set/active member.

    static PMResponse_t fromBlock(std::initializer_list<uint8_t> b) {
      PMResponse_t v;
      size_t c = 0;
      for (auto i = b.begin(); i < b.end() && c < sizeof(data.block); i++) {
        v.data.block[c++] = *i;
      }
      v.tag = eBlock;
      return v;
    }
    static PMResponse_t fromUInt16(uint16_t i) {
      PMResponse_t v;
      v.data.i = i;
      v.tag = eUint16;
      return v;
    }
    static PMResponse_t fromFloat(float f) {
      PMResponse_t v;
      v.data.f = f;
      v.tag = eFloat;
      return v;
    }
  };

  // Below maps what each register type corresponds to which integral type that represents it data.
  struct eRegTypeToDataTypeMap_t {
    eRegTypes regType;
    PMResponse_t::eTags_t integralType;
  };
  using eRegTypeToDataTypeMapArr_t = std::array<eRegTypeToDataTypeMap_t, 4>;
  static constexpr eRegTypeToDataTypeMapArr_t eRegTypeToDataTypeMap = { { { etBlock, PMResponse_t::eTags_t::eBlock }, {
      etNonOutVoltLinearFormat,
      PMResponse_t::eTags_t::eFloat }, { etOutVoltLinearFormat, PMResponse_t::eTags_t::eFloat }, {
      etOther,
      PMResponse_t::eTags_t::eUint16 } } };
  static constexpr PMResponse_t::eTags_t getRegIntegralType(eRegTypes e) {
    for (size_t i = 0; i < eRegTypeToDataTypeMap.size(); i++) {
      if (eRegTypeToDataTypeMap[i].regType == e) {
        return eRegTypeToDataTypeMap[i].integralType;
      }
    }
    return PMResponse_t::eTags_t::eNone;
  }

  // These are to be used in HPT5K0Config.h/cpp
  struct psuCmdDefaultValues_t {
    psuCmdName_t index;
    PMResponse_t defaultValue;  // Must be LSB first.
  };

  static constexpr psuCmd_t getCmd(uint32_t cmd) {
    for (size_t i = 0; i < psuCmdName_t::LAST_ITEM; i++) {
      if (PsuCmds[i].index == cmd) {
        return PsuCmds[i];
      }
    }
    return UNKNOWN_CMD;
  }

  // The inhibits
  enum class epPsEn_t
    : bool {
      PIN_HIGH = true,
    PIN_LOW = false,
    PSU_OUTPUT_ON = PIN_HIGH,
    PSU_OUTPUT_OFF = PIN_LOW
  };

  enum RW_BIT_t
    : uint8_t {
      READ_BIT = 0x01,
    WRITE_BIT = 0x00
  };
  enum WRITE_PROTECT_t
    : uint8_t {
      WRITE_PROTECT_DISABLE = 0x00,
    WRITE_PROTECT_ENABLE = 0x80
  };
  enum OPERATION_t
    : uint8_t {
      OPERATION_OFF = 0x00,
    OPERATION_ON = 0x80
  };
  /* ########################### END COMMANDS RELATED ########################### */

  /* ########################### USER CONFIGURATION REGISTER RELATED ########################### */
  using UserConfigureationRegBitIndexType_t = uint32_t;
  enum class UserConfigurationRegBitIndex_t
    : UserConfigureationRegBitIndexType_t {
      CFG_CURRENT_SOFTSTART_ENABLE = 0,
    CFG_FAST_SOFTSTART_DISABLE,
    CFG_PRELOAD_DISABLE,
    CFG_FAN_OFF,
    CFG_ACOK_SIG_LOGIC,
    CFG_DCOK_SIG_LOGIC,
    RESERVED1 = 6,
    FG_FAN_TEMP_OK_SIG_LOGIC,
    RESERVED2 = 8,
    CFG_REMOTE_INHIBIT_LOGIC,
    CFG_POTENTIOMETER_DISABLE,
    CFG_POTENTIOMETER_FULL_ADJ,
    CFG_ANALOG_PROG,
    CFG_DISABLE_IPROG,
    RESERVED3,
    RESERVED4 = 15,
//
    LAST_ITEM
  };

  struct UserConfigurationRegBit_t {
    UserConfigurationRegBitIndex_t bitIndex;
    bool enable;  // true=1, false=0
  };

  using UserConfigurationReg_t = std::array<UserConfigurationRegBit_t, static_cast<UserConfigureationRegBitIndexType_t>(UserConfigurationRegBitIndex_t::LAST_ITEM)>;
  /* ########################### END USER CONFIGURATION REGISTER RELATED ########################### */

  static bool isReadCmd(psuCmdName_t cmd) {
    return ((HPT5K0Info_t::getCmd(cmd).regAccess == regAccess_t::RW)
        || (HPT5K0Info_t::getCmd(cmd).regAccess == regAccess_t::RWE)
        || (HPT5K0Info_t::getCmd(cmd).regAccess == regAccess_t::RO)
        || (HPT5K0Info_t::getCmd(cmd).regAccess == regAccess_t::ROE));
  }
  static bool isWriteCmd(psuCmdName_t cmd) {
    return ((HPT5K0Info_t::getCmd(cmd).regAccess == regAccess_t::RW)
        || (HPT5K0Info_t::getCmd(cmd).regAccess == regAccess_t::RWE)
        || (HPT5K0Info_t::getCmd(cmd).regAccess == regAccess_t::WO));
  }

 private:
  // These are all the possible commands that can be sent to the PSUs
  static constexpr psuCmds_t PsuCmds = { {
       { psuCmdName_t::OPERATION, 0x01, 1, regAccess_t::RWE, 0, etOther, "OPERATION" },
       { psuCmdName_t::CLEAR_FAULTS, 0x03, 0, regAccess_t::WO, 1, etOther, "CLEAR_FAULTS" },
       { psuCmdName_t::WRITE_PROTECT, 0x10, 1, regAccess_t::RW, 1, etOther, "WRITE_PROTECT" },
       { psuCmdName_t::STORE_DEFAULT_ALL, 0x11, 0, regAccess_t::WO, 2, etOther, "STORE_DEFAULT_ALL" },
       { psuCmdName_t::RESTORE_DEFAULT_ALL, 0x12, 0, regAccess_t::WO, 2, etOther, "RESTORE_DEFAULT_ALL" },
       { psuCmdName_t::STORE_USER_ALL, 0x15, 0, regAccess_t::WO, 2, etOther, "STORE_USER_ALL" },
       { psuCmdName_t::RESTORE_USER_ALL, 0x16, 0, regAccess_t::WO, 2, etOther, "RESTORE_USER_ALL" },
       { psuCmdName_t::VOUT_MODE, 0x20, 1, regAccess_t::RO, 2, etNonOutVoltLinearFormat, "VOUT_MODE" },
       { psuCmdName_t::VOUT_COMMAND, 0x21, 2, regAccess_t::RWE, 3, etOutVoltLinearFormat, "VOUT_COMMAND" },
       { psuCmdName_t::POUT_MAX, 0x31, 2, regAccess_t::RO, 5, etNonOutVoltLinearFormat, "POUT_MAX" },
       { psuCmdName_t::FAN_CONFIG_1_2, 0x3A, 1, regAccess_t::RO, 7, etOther, "FAN_CONFIG_1_2" },
       { psuCmdName_t::FAN_COMMAND_1, 0x3B, 2, regAccess_t::RWE, 8, etNonOutVoltLinearFormat, "FAN_COMMAND_1" },
       { psuCmdName_t::VOUT_OV_FAULT_LIMIT, 0x40, 2, regAccess_t::RWE, 10, etOutVoltLinearFormat, "VOUT_OV_FAULT_LIMIT" },
       { psuCmdName_t::VOUT_OV_FAULT_RESPONSE, 0x41, 1, regAccess_t::RO, 12, etOther, "VOUT_OV_FAULT_RESPONSE" },
       { psuCmdName_t::VOUT_OV_WARN_LIMIT, 0x42, 2, regAccess_t::RWE, 13, etOutVoltLinearFormat, "VOUT_OV_WARN_LIMIT" },
       { psuCmdName_t::VOUT_UV_WARN_LIMIT, 0x43, 2, regAccess_t::RWE, 15, etOutVoltLinearFormat, "VOUT_UV_WARN_LIMIT" },
       { psuCmdName_t::VOUT_UV_FAULT_LIMIT, 0x44, 2, regAccess_t::RWE, 17, etOutVoltLinearFormat, "VOUT_UV_FAULT_LIMIT" },
       { psuCmdName_t::VOUT_UV_FAULT_RESPONSE, 0x45, 1, regAccess_t::RWE, 19, etOther, "VOUT_UV_FAULT_RESPONSE" },
       { psuCmdName_t::IOUT_OC_FAULT_LIMIT, 0x46, 2, regAccess_t::RWE, 20, etNonOutVoltLinearFormat, "IOUT_OC_FAULT_LIMIT" },
       { psuCmdName_t::IOUT_OC_FAULT_RESPONSE, 0x47, 1, regAccess_t::RWE, 22, etOther, "IOUT_OC_FAULT_RESPONSE" },
       { psuCmdName_t::IOUT_OC_LV_FAULT_LIMIT, 0x48, 2, regAccess_t::RWE, 23, etNonOutVoltLinearFormat, "IOUT_OC_LV_FAULT_LIMIT" },
       { psuCmdName_t::IOUT_OC_WARN_LIMIT, 0x4A, 2, regAccess_t::RWE, 25, etNonOutVoltLinearFormat, "IOUT_OC_WARN_LIMIT" },
       { psuCmdName_t::OT_PRI_WARN_LIMIT, 0x4D, 2, regAccess_t::RWE, 27, etNonOutVoltLinearFormat, "OT_PRI_WARN_LIMIT" },
       { psuCmdName_t::OT_PRI_FAULT_LIMIT, 0x4E, 2, regAccess_t::RWE, 29, etNonOutVoltLinearFormat, "OT_PRI_FAULT_LIMIT" },
       { psuCmdName_t::OT_SEC_FAULT_LIMIT, 0x4F, 2, regAccess_t::RWE, 31, etNonOutVoltLinearFormat, "OT_SEC_FAULT_LIMIT" },
       { psuCmdName_t::OT_FAULT_RESPONSE, 0x50, 1, regAccess_t::RWE, 33, etOther, "OT_FAULT_RESPONSE" },
       { psuCmdName_t::OT_SEC_WARN_LIMIT, 0x51, 2, regAccess_t::RWE, 34, etNonOutVoltLinearFormat, "OT_SEC_WARN_LIMIT" },
       { psuCmdName_t::VIN_OV_FAULT_LIMIT, 0x55, 2, regAccess_t::RO, 36, etNonOutVoltLinearFormat, "VIN_OV_FAULT_LIMIT" },
       { psuCmdName_t::VIN_OV_FAULT_RESPONSE, 0x56, 1, regAccess_t::RWE, 38, etOther, "VIN_OV_FAULT_RESPONSE" },
       { psuCmdName_t::VIN_OV_WARN_LIMIT, 0x57, 2, regAccess_t::RO, 39, etNonOutVoltLinearFormat, "VIN_OV_WARN_LIMIT" },
       { psuCmdName_t::VIN_UV_WARN_LIMIT, 0x58, 2, regAccess_t::RO, 41, etNonOutVoltLinearFormat, "VIN_UV_WARN_LIMIT" },
       { psuCmdName_t::VIN_UV_FAULT_LIMIT, 0x59, 2, regAccess_t::RO, 43, etNonOutVoltLinearFormat, "VIN_UV_FAULT_LIMIT" },
       { psuCmdName_t::VIN_UV_FAULT_RESPONSE, 0x5A, 1, regAccess_t::RWE, 45, etOther, "VIN_UV_FAULT_RESPONSE" },
       { psuCmdName_t::STATUS_BYTE, 0x78, 1, regAccess_t::RO, 46, etOther, "STATUS_BYTE" },
       { psuCmdName_t::STATUS_WORD, 0x79, 2, regAccess_t::RO, 47, etOther, "STATUS_WORD" },
       { psuCmdName_t::STATUS_VOUT, 0x7A, 1, regAccess_t::RO, 49, etOther, "STATUS_VOUT" },
       { psuCmdName_t::STATUS_IOUT, 0x7B, 1, regAccess_t::RO, 50, etOther, "STATUS_IOUT" },
       { psuCmdName_t::STATUS_INPUT, 0x7C, 1, regAccess_t::RO, 51, etOther, "STATUS_INPUT" },
       { psuCmdName_t::STATUS_TEMPERATURE, 0x7D, 1, regAccess_t::RO, 52, etOther, "STATUS_TEMPERATURE" },
       { psuCmdName_t::STATUS_CML, 0x7E, 1, regAccess_t::RO, 53, etOther, "STATUS_CML" },
       { psuCmdName_t::STATUS_OTHER, 0x7F, 1, regAccess_t::RO, 54, etOther, "STATUS_OTHER" },
       { psuCmdName_t::STATUS_MFR_SPECIFIC, 0x80, 1, regAccess_t::RO, 55, etOther, "STATUS_MFR_SPECIFIC" },
       { psuCmdName_t::STATUS_FAN_1_2, 0x81, 1, regAccess_t::RO, 56, etOther, "STATUS_FAN_1_2" },
       { psuCmdName_t::READ_VIN, 0x88, 2, regAccess_t::RO, 57, etNonOutVoltLinearFormat, "READ_VIN" },
       { psuCmdName_t::READ_VOUT, 0x8B, 2, regAccess_t::RO, 59, etOutVoltLinearFormat, "READ_VOUT" },
       { psuCmdName_t::READ_IOUT, 0x8C, 2, regAccess_t::RO, 61, etNonOutVoltLinearFormat, "READ_IOUT" },
       { psuCmdName_t::READ_TEMPERATURE_1, 0x8D, 2, regAccess_t::RO, 63, etNonOutVoltLinearFormat, "READ_TEMPERATURE_1" },
       { psuCmdName_t::READ_TEMPERATURE_2, 0x8E, 2, regAccess_t::RO, 65, etNonOutVoltLinearFormat, "READ_TEMPERATURE_2" },
       { psuCmdName_t::READ_TEMPERATURE_3, 0x8F, 2, regAccess_t::RO, 67, etNonOutVoltLinearFormat, "READ_TEMPERATURE_3" },
       { psuCmdName_t::READ_FAN_SPEED_1, 0x90, 2, regAccess_t::RO, 69, etNonOutVoltLinearFormat, "READ_FAN_SPEED_1" },
       { psuCmdName_t::READ_FAN_SPEED_2, 0x91, 2, regAccess_t::RO, 71, etNonOutVoltLinearFormat, "READ_FAN_SPEED_2" },
       { psuCmdName_t::READ_FAN_SPEED_3, 0x92, 2, regAccess_t::RO, 73, etNonOutVoltLinearFormat, "READ_FAN_SPEED_3" },
       { psuCmdName_t::READ_FAN_SPEED_4, 0x93, 2, regAccess_t::RO, 75, etNonOutVoltLinearFormat, "READ_FAN_SPEED_4" },
       { psuCmdName_t::READ_POUT, 0x96, 2, regAccess_t::RO, 77, etOther, "READ_POUT" },
       { psuCmdName_t::MFR_ID, 0x99, 16, regAccess_t::RO, 79, etBlock, "MFR_ID" },
       { psuCmdName_t::MFR_MODEL, 0x9A, 32, regAccess_t::RO, 95, etBlock, "MFR_MODEL" },
       { psuCmdName_t::MFR_REVISION, 0x9B, 4, regAccess_t::RO, 127, etBlock, "MFR_REVISION" },
       { psuCmdName_t::MFR_LOCATION, 0x9C, 16, regAccess_t::RO, 131, etBlock, "MFR_LOCATION" },
       { psuCmdName_t::MFR_DATE, 0x9D, 6, regAccess_t::RO, 147, etBlock, "MFR_DATE" },
       { psuCmdName_t::MFR_SERIAL, 0x9E, 16, regAccess_t::RO, 153, etBlock, "MFR_SERIAL" },
       { psuCmdName_t::MFR_VIN_MIN, 0xA0, 2, regAccess_t::RO, 169, etNonOutVoltLinearFormat, "MFR_VIN_MIN" },
       { psuCmdName_t::MFR_VIN_MAX, 0xA1, 2, regAccess_t::RO, 171, etNonOutVoltLinearFormat, "MFR_VIN_MAX" },
       { psuCmdName_t::MFR_IIN_MAX, 0xA2, 2, regAccess_t::RO, 173, etNonOutVoltLinearFormat, "MFR_IIN_MAX" },
       { psuCmdName_t::MFR_PIN_MAX, 0xA3, 2, regAccess_t::RO, 175, etNonOutVoltLinearFormat, "MFR_PIN_MAX" },
       { psuCmdName_t::MFR_VOUT_MIN, 0xA4, 2, regAccess_t::RO, 177, etOutVoltLinearFormat, "MFR_VOUT_MIN" },
       { psuCmdName_t::MFR_VOUT_MAX, 0xA5, 2, regAccess_t::RO, 179, etOutVoltLinearFormat, "MFR_VOUT_MAX" },
       { psuCmdName_t::MFR_IOUT_MAX, 0xA6, 2, regAccess_t::RO, 181, etNonOutVoltLinearFormat, "MFR_IOUT_MAX" },
       { psuCmdName_t::MFR_POUT_MAX, 0xA7, 2, regAccess_t::RO, 183, etNonOutVoltLinearFormat, "MFR_POUT_MAX" },
       { psuCmdName_t::MFR_TAMBIENT_MAX, 0xA8, 2, regAccess_t::RO, 185, etNonOutVoltLinearFormat, "MFR_TAMBIENT_MAX" },
       { psuCmdName_t::MFR_TAMBIENT_MIN, 0xA9, 2, regAccess_t::RO, 187, etNonOutVoltLinearFormat, "MFR_TAMBIENT_MIN" },
       { psuCmdName_t::MFR_PRODUCT_CODE, 0xAD, 2, regAccess_t::RO, 189, etOther, "MFR_PRODUCT_CODE" },
       { psuCmdName_t::USER_DATA_00, 0xB0, 16, regAccess_t::RWE, 191, etBlock, "USER_DATA_00" },
       { psuCmdName_t::USER_DATA_01, 0xB1, 16, regAccess_t::RWE, 207, etBlock, "USER_DATA_01" },
       { psuCmdName_t::FIRMWARE_REVISION, 0xD0, 1, regAccess_t::RO, 223, etOther, "FIRMWARE_REVISION" },
       { psuCmdName_t::RUN_TIME, 0xD1, 3, regAccess_t::RO, 224, etBlock, "RUN_TIME" },
       { psuCmdName_t::VOUT_RAMP_UP, 0xD2, 2, regAccess_t::RWE, 227, etOutVoltLinearFormat, "VOUT_RAMP_UP" },
       { psuCmdName_t::SLAVE_ID, 0xD3, 1, regAccess_t::RWE, 229, etOther, "SLAVE_ID" },
       { psuCmdName_t::SLAVE_BASE_ADR, 0xD4, 1, regAccess_t::RWE, 230, etOther, "SLAVE_BASE_ADR" },
       { psuCmdName_t::CANBUS_BIT_RATE, 0xD5, 4, regAccess_t::RWE, 231, etBlock, "CANBUS_BIT_RATE" },
       { psuCmdName_t::USER_CONFIGURATION, 0xD6, 2, regAccess_t::RWE, 235, etOther, "USER_CONFIGURATION" },
       { psuCmdName_t::SERIAL_COMM_CONFIG, 0xD7, 8, regAccess_t::RWE, 237, etBlock, "SERIAL_COMM_CONFIG" },
       { psuCmdName_t::READ_IOUT1, 0xD8, 2, regAccess_t::RO, 245, etNonOutVoltLinearFormat, "READ_IOUT1" },
       { psuCmdName_t::READ_IOUT2, 0xD9, 2, regAccess_t::RO, 247, etNonOutVoltLinearFormat, "READ_IOUT2" },
       { psuCmdName_t::READ_IOUT3, 0xDA, 2, regAccess_t::RO, 249, etNonOutVoltLinearFormat, "READ_IOUT3" },
       { psuCmdName_t::HARDWARE_CONFIG, 0xDE, 1, regAccess_t::RWE, 251, etOther, "HARDWARE_CONFIG" },
       { psuCmdName_t::VOUT_RAM_DOWN, 0xDF, 2, regAccess_t::RWE, 252, etOutVoltLinearFormat, "VOUT_RAM_DOWN" },
       { psuCmdName_t::READ_DATA_PFC1, 0xE0, 9, regAccess_t::RO, 254, etBlock, "READ_DATA_PFC1" },
       { psuCmdName_t::READ_DATA_PFC2, 0xE1, 9, regAccess_t::RO, 263, etBlock, "READ_DATA_PFC2" },
       { psuCmdName_t::READ_DATA_PFC3, 0xE2, 9, regAccess_t::RO, 272, etBlock, "READ_DATA_PFC3" },
       { psuCmdName_t::READ_INFO_PFC1, 0xE3, 18, regAccess_t::RO, 281, etBlock, "READ_INFO_PFC1" },
       { psuCmdName_t::READ_INFO_PFC2, 0xE4, 18, regAccess_t::RO, 299, etBlock, "READ_INFO_PFC2" },
       { psuCmdName_t::READ_INFO_PFC3, 0xE5, 18, regAccess_t::RO, 317, etBlock, "READ_INFO_PFC3" },
       { psuCmdName_t::SHUTDOWN_EVENT, 0xE8, 4, regAccess_t::RO, 335, etBlock, "SHUTDOWN_EVENT" },
       { psuCmdName_t::SHUTDOWN_EVENT_LAST, 0xE9, 4, regAccess_t::RO, 339, etBlock, "SHUTDOWN_EVENT_LAST" },
       { psuCmdName_t::STATUS_INTERNAL, 0xEB, 4, regAccess_t::RO, 343, etBlock, "STATUS_INTERNAL" },
       { psuCmdName_t::STATE_INTERNAL, 0xEC, 2, regAccess_t::RO, 347, etOther, "STATE_INTERNAL" },
       { psuCmdName_t::STATUS_PRIMARY, 0xED, 2, regAccess_t::RO, 349, etOther, "STATUS_PRIMARY" },
       { psuCmdName_t::FAN_DUTY_CYCLE, 0xEE, 2, regAccess_t::RO, 351, etNonOutVoltLinearFormat, "FAN_DUTY_CYCLE" } } };

  static constexpr psuCmd_t UNKNOWN_CMD = { psuCmdName_t::UNKNOWN, 0x0, 0, regAccess_t::RO, 0, etOther, "UNKNOWN" };
};

#ifdef _MSC_VER
__pragma(pack(pop))
#endif

#endif /* SRC_MANTAEXPORT_HPT5K0INFO_H_ */
