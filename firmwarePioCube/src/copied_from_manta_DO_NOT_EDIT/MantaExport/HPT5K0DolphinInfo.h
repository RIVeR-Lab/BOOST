/*
 * HPT5K0DolphinInfo.h
 *
 * Power Supply information useful for Dolphin.
 *
 *  Created on: Sep 6, 2022
 *      Author: DavidAntaki
 */

#ifndef SRC_MANTAEXPORT_HPT5K0DOLPHININFO_H_
#define SRC_MANTAEXPORT_HPT5K0DOLPHININFO_H_

#include <stdint.h>
#include <array>
#include <string.h>

// msvc & gcc pack syntax is different
#ifdef _MSC_VER
__pragma(pack(push, 1))
// disable GCC pack inst
#define __attribute__(...)
#endif

// Type for reporting PSU status
struct __attribute__((packed))HPT5K0DolphinInfo_t {
 public:
  HPT5K0DolphinInfo_t() {
  }

  /* ########################### DELTA LOGGER RELATED ########################### */
  // Source of truth for what value in the LogSpecs DeltaLogger PSU#Status corresponds to what PSU register.
  enum class PsuStatusIndex_t
    : uint32_t {
      READ_VIN,
    READ_VOUT,
    READ_IOUT,
    READ_TEMPERATURE_1,
    READ_TEMPERATURE_2,
    READ_TEMPERATURE_3,
    STATUS_WORD,
//
    nStatusRegs = 16, // The max number of status registers there can be.
    NONE,

  };

  enum psuCmdName_t :uint32_t;
  struct PsuStatusReg_t {
    psuCmdName_t index = UNKNOWN;
    PsuStatusIndex_t psuStatusIndex = PsuStatusIndex_t::NONE;
    float lastReading = 0.0f;
  };

  static const size_t nStatusRegs = static_cast<uint32_t>(PsuStatusIndex_t::nStatusRegs);
  using PsuStatusRegs_t = std::array<PsuStatusReg_t, nStatusRegs>;
  /* ########################### END DELTA LOGGER RELATED ########################### */

  enum ePsuIndices
    : uint32_t {
      iPSU1,
    iPSU2,
    iPSU3,
    iPSU4,
    iPSU5,
    //
    LAST_PSU_INDEX
  };

  /* ########################### COMMANDS RELATED ########################### */
  enum psuCmdName_t
    : uint32_t {
      OPERATION = 0,
    CLEAR_FAULTS,
    WRITE_PROTECT,
    STORE_DEFAULT_ALL,
    RESTORE_DEFAULT_ALL,
    STORE_USER_ALL,
    RESTORE_USER_ALL,
    VOUT_MODE,
    VOUT_COMMAND = 8,
    POUT_MAX,
    FAN_CONFIG_1_2,
    FAN_COMMAND_1,
    VOUT_OV_FAULT_LIMIT,
    VOUT_OV_FAULT_RESPONSE,
    VOUT_OV_WARN_LIMIT,
    VOUT_UV_WARN_LIMIT,
    VOUT_UV_FAULT_LIMIT = 16,
    VOUT_UV_FAULT_RESPONSE,
    IOUT_OC_FAULT_LIMIT,
    IOUT_OC_FAULT_RESPONSE,
    IOUT_OC_LV_FAULT_LIMIT,
    IOUT_OC_WARN_LIMIT,
    OT_PRI_WARN_LIMIT,
    OT_PRI_FAULT_LIMIT,
    OT_SEC_FAULT_LIMIT = 24,
    OT_FAULT_RESPONSE,
    OT_SEC_WARN_LIMIT,
    VIN_OV_FAULT_LIMIT,
    VIN_OV_FAULT_RESPONSE,
    VIN_OV_WARN_LIMIT,
    VIN_UV_WARN_LIMIT,
    VIN_UV_FAULT_LIMIT,
    VIN_UV_FAULT_RESPONSE = 32,
    STATUS_BYTE,
    STATUS_WORD,
    STATUS_VOUT,
    STATUS_IOUT,
    STATUS_INPUT,
    STATUS_TEMPERATURE,
    STATUS_CML,
    STATUS_OTHER,
    STATUS_MFR_SPECIFIC,
    STATUS_FAN_1_2,
    READ_VIN,
    READ_VOUT,
    READ_IOUT,
    READ_TEMPERATURE_1,
    READ_TEMPERATURE_2,
    READ_TEMPERATURE_3,
    READ_FAN_SPEED_1,
    READ_FAN_SPEED_2,
    READ_FAN_SPEED_3,
    READ_FAN_SPEED_4,
    READ_POUT,
    MFR_ID,
    MFR_MODEL,
    MFR_REVISION,
    MFR_LOCATION,
    MFR_DATE,
    MFR_SERIAL,
    MFR_VIN_MIN,
    MFR_VIN_MAX,
    MFR_IIN_MAX,
    MFR_PIN_MAX,
    MFR_VOUT_MIN,
    MFR_VOUT_MAX,
    MFR_IOUT_MAX,
    MFR_POUT_MAX,
    MFR_TAMBIENT_MAX,
    MFR_TAMBIENT_MIN,
    MFR_PRODUCT_CODE,
    USER_DATA_00,
    USER_DATA_01,
    FIRMWARE_REVISION,
    RUN_TIME,
    VOUT_RAMP_UP,
    SLAVE_ID,
    SLAVE_BASE_ADR,
    CANBUS_BIT_RATE,
    USER_CONFIGURATION,
    SERIAL_COMM_CONFIG,
    READ_IOUT1,
    READ_IOUT2,
    READ_IOUT3,
    HARDWARE_CONFIG,
    VOUT_RAM_DOWN,
    READ_DATA_PFC1,
    READ_DATA_PFC2,
    READ_DATA_PFC3,
    READ_INFO_PFC1,
    READ_INFO_PFC2,
    READ_INFO_PFC3,
    SHUTDOWN_EVENT,
    SHUTDOWN_EVENT_LAST,
    STATUS_INTERNAL,
    STATE_INTERNAL,
    STATUS_PRIMARY,
    FAN_DUTY_CYCLE,
    //
    LAST_ITEM,
    UNKNOWN
  };
  /* ########################### END COMMANDS RELATED ########################### */

  /* ########################### STATUS WORD REGISTER RELATED ########################### */
  enum class StatusWordRegBitIndex_t
    : uint32_t {
      NONE_OF_THE_ABOVE = 0,  // A fault or warning not listed in bits [7:1] has occurred
    CML,  // A communications, memory or logic fault has occurred
    TEMPERATURE,  // A temperature fault or warning has occurred
    VIN_UV_FAULT,  // An input under voltage fault has occurred
    IOUT_OC_FAULT,  // An output overcurrent fault has occurred
    VOUT_OV_FAULT,  // An output overvoltage fault has occurred
    OFF,  // This bit is asserted if the unit is not providing power to the output, regardless of the reason, including simply not being enabled.
    BUSY,  // A fault was declared because the device was busy and unable to respond.
    UNKNOWN,  // A fault type not given in bits [15:1] of the STATUS_WORD has been detected
    OTHER,  // A bit in STATUS_OTHER is set
    FANS,  // A fan or airflow fault or warning has occurred
    POWER_GOOD,  // The POWER_GOOD signal, if present, is negated
    MFR_SPECIFIC,  // A manufacturer specific fault or warning has occurred
    INPUT,  // An input voltage, input current, or input power fault or warning has occurred
    IOU_POUT,  // An output current or output power fault or warning has occurred
    VOUT = 15,  // An output voltage fault or warning has occurred
//
    LAST_ITEM
  };

  struct StatusWordRegBit_t {
    StatusWordRegBitIndex_t bitIndex;
    const char* name;
  };

  static constexpr std::array<StatusWordRegBit_t, static_cast<uint32_t>(StatusWordRegBitIndex_t::LAST_ITEM)> StatusWordRegBits =
      { {
          { StatusWordRegBitIndex_t::NONE_OF_THE_ABOVE, "NONE_OF_THE_ABOVE" },
          { StatusWordRegBitIndex_t::CML, "CML" },
          { StatusWordRegBitIndex_t::TEMPERATURE, "TEMPERATURE" },
          { StatusWordRegBitIndex_t::VIN_UV_FAULT, "VIN_UV_FAULT" },
          { StatusWordRegBitIndex_t::IOUT_OC_FAULT, "IOUT_OC_FAULT" },
          { StatusWordRegBitIndex_t::VOUT_OV_FAULT, "VOUT_OV_FAULT" },
          { StatusWordRegBitIndex_t::OFF, "OFF" },
          { StatusWordRegBitIndex_t::BUSY, "BUSY" },
          { StatusWordRegBitIndex_t::UNKNOWN, "UNKNOWN" },
          { StatusWordRegBitIndex_t::OTHER, "OTHER" },
          { StatusWordRegBitIndex_t::FANS, "FANS" },
          { StatusWordRegBitIndex_t::POWER_GOOD, "POWER_GOOD" },
          { StatusWordRegBitIndex_t::MFR_SPECIFIC, "MFR_SPECIFIC" },
          { StatusWordRegBitIndex_t::INPUT, "INPUT" },
          { StatusWordRegBitIndex_t::IOU_POUT, "IOU_POUT" },
          { StatusWordRegBitIndex_t::VOUT, "VOUT" } } };

  static const char* getStatusWordBitStr(StatusWordRegBitIndex_t bit) {
    for (size_t i = 0; i < StatusWordRegBits.size(); i++) {
      if (bit == StatusWordRegBits[i].bitIndex) {
        return StatusWordRegBits[i].name;
      }
    }
    return "Status Bit NOT FOUND";
  }

  // Little-endian: MSB on the right.
  static bool isStatusWordBitSet(uint16_t regData, StatusWordRegBitIndex_t bit);
  /* ########################### END STATUS WORD REGISTER RELATED ########################### */

};

#ifdef _MSC_VER
__pragma(pack(pop))
#endif

#endif /* SRC_MANTAEXPORT_HPT5K0DOLPHININFO_H_ */
