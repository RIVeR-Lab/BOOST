/*
 * HPT5K0.h
 *
 *  Created on: Aug 24, 2022
 *      Author: David Antaki
 */

#ifndef SRC_MANTAHW_HPT5K0_H_
#define SRC_MANTAHW_HPT5K0_H_
#include <MantaExport/HPT5K0Info.h>
#include "MantaExport/HPT5K0DolphinInfo.h"
#include "MantaHW/I2cBase.h"
#include "Board2Board/Logging.h"
#include "View/ePaper.h"

class HPT5K0 {
 private:
  static const size_t kMaxNonBlockRegSize = 4;
  static const size_t kMaxBlockRegSize = 32;
  static const size_t kRegAddrSize = sizeof(uint8_t);  // aka 'command'
  static const size_t kSlaveAddrSize = sizeof(uint8_t);
  static const size_t kCRCSize = sizeof(uint8_t);  // SMBus uses 1 byte crc
  static const size_t kCRCDataBufSize = kSlaveAddrSize + kRegAddrSize + kCRCSize + kMaxNonBlockRegSize;  // Write data + crc byte
  static const size_t kBlockReadBytesByteSize = sizeof(uint8_t);
  static const size_t kCRCReadDataBufSize = kSlaveAddrSize + kRegAddrSize + kSlaveAddrSize + kMaxBlockRegSize + kCRCSize
      + kBlockReadBytesByteSize;

  using psuCmdDefaultValues_t = HPT5K0Info_t::psuCmdDefaultValues_t;
  using PsuStatusRegs_t = HPT5K0DolphinInfo_t::PsuStatusRegs_t;
  using psuCmdName_t = HPT5K0DolphinInfo_t::psuCmdName_t;
  using psuCmd_t = HPT5K0Info_t::psuCmd_t;
  using psuCmds_t = HPT5K0Info_t::psuCmds_t;
  using regAccess_t = HPT5K0Info_t::regAccess_t;
  using PsuStatusIndex_t = HPT5K0DolphinInfo_t::PsuStatusIndex_t;
  using PMResponse_t = HPT5K0Info_t::PMResponse_t;

  // For when we are force reading/writing to PSUs and we need to temporarily set initialize=true.
  class TempInitialize {
   public:
    TempInitialize(bool& _tempInit, bool _force)
        : tempInit(_tempInit),
          force(_force) {
      if (force) {
        tempInit = true;
      }
    }
    ~TempInitialize() {
      if (force) {
        tempInit = false;
      }
    }
   private:
    bool &tempInit;
    bool force;
  };

 public:
  // PSU I2C addresses
  // Address for all PSUs have format: 0xBx = 0b1011A2A1A0
  enum class ePsuI2cAddr
    : unsigned char {
      PSU1 = 0x5F,
    PSU2 = 0x5E,
    PSU3 = 0x5D,
    PSU4 = 0x5C,
    PSU5 = 0x5B
  };

  HPT5K0(Logging &log, I2cBase& i2cbus2, HPT5K0::ePsuI2cAddr i2cAddr, uint32_t psuIndex,
         const psuCmdDefaultValues_t defaults[], const uint32_t defaultsSize, Manta::LogSpecs::Subject subj,
         PsuStatusRegs_t statusRegsConfig);
  virtual ~HPT5K0() {
  }

  static const uint32_t DEFAULT_VALUE_SIZE = HPT5K0Info_t::DEFAULT_VALUE_SIZE;
  using regData_t = HPT5K0Info_t::regData_t;  // Used purely for DeltaLogger.

  bool initialize();
  bool isInitialized() {
    return initialized;
  }

  // Only use from BoardCLI!!
  bool reconfigurePsu(State_t &s, ePaper &view);

  bool checkPsuConfiguration();

  /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! IMPORTANT NOTE: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
  // Voltage OUTPUT commands (commands in the voltageCmds array) are converted.
  // to the actual voltage in volts i.e. when writing, call the write() function with actual voltage value in volts, when reading,
  // call the read() function and it will return the actual voltage in volts. The read/write() functions will convert the voltage to what should ACTUALLY
  // get written to the PSU's registers internally. We can do this only if VOUT_MODE==0x18 which means we multiply any voltage output register
  // by 1/256 and since all voltage output registers are 16bit unsigned integers, we can guarantee that the actual voltage value in volts will
  // be less than 16bits wide, thus we know that we can always store a voltage output register as the actual voltage value in volts.
  //
  // Linearly formatted registers, however, must be converted from the register value to their actual value AFTER reading the data
  // using the Read() function. This conversion does not happen internally to the Read() function.
  bool Read(psuCmdName_t cmd, PMResponse_t &outResponse, bool force = false);
  bool Write(psuCmdName_t cmd, const PMResponse_t data, bool force = false);
  bool doWrite(psuCmdName_t cmd, const PMResponse_t data);
  /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! END IMPORTANT NOTE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

  // Force logging of status registers current state.
  bool forceLogStatusRegisters() {
    return deltaLoggerStatus.sendData(true);
  }

  // Uses DeltaLogger to log psuStatus if values changed.
  bool checkPsuStatusRegisters();

  static constexpr bool isLinearFormatOutVoltCommand(psuCmdName_t cmd) {
    return HPT5K0Info_t::getCmd(cmd).regType == HPT5K0Info_t::etOutVoltLinearFormat;
  }

  static constexpr bool isLinearFormatNonOutVoltCommand(psuCmdName_t cmd) {
    return HPT5K0Info_t::getCmd(cmd).regType == HPT5K0Info_t::etNonOutVoltLinearFormat;
  }

  static constexpr bool isBlockCommand(psuCmdName_t cmd) {
    return HPT5K0Info_t::getCmd(cmd).regType == HPT5K0Info_t::etBlock;
  }

  static constexpr bool isOtherCommand(psuCmdName_t cmd) {
    return HPT5K0Info_t::getCmd(cmd).regType == HPT5K0Info_t::etOther;
  }

  const PsuStatusRegs_t getPsuStatusRegs() {
    return psuStatus;
  }

  // Convers a register with type HPT5K0Info_t::etOutVoltLinearFormat to a float.
  constexpr bool linearFormatOutVoltRegToFloat(psuCmdName_t cmd, uint16_t data, float& out);
  // Does opposite of above func.
  constexpr bool floatToLinearFormatOutVoltReg(psuCmdName_t cmd, float data, uint16_t &out);

  // Converts a register with type HPT5K0Info_t::etNonOutVoltLinearFormat to a float.
  static constexpr bool linearFormatNonOutVoltRegToFloat(psuCmdName_t cmd, const uint16_t data, float &out);
  // Does the opposite of above func.
  bool floatToLinearFormatNonOutVoltReg(psuCmdName_t cmd, float data, uint16_t &out);

 private:
  bool initialized = false;
//	static const uint32_t WRITEMILLIS = 5;
  static const uint32_t WAITTIME_MS = 200;  // let theses reads & writes be extra patient

  // A shadow copy of all the PSU register data.
  // Used purely for DeltaLogger.
  regData_t regData;

  bool enableWriteProtect(bool en);

  // Given a copy of all the registers, convert to float value.
  // Used purely for DeltaLogger.
  bool regValueToFloat(psuCmdName_t reg, regData_t regData, float& output);

  /**
   * Calculates CRC-8 as written in the SMBus spec for a single byte.
   */
  static constexpr inline uint8_t smbusCrc8(uint8_t inCrc, uint8_t inData);
  /**
   * Calculates CRC-8 as written in the SMBus spec for multiple bytes.
   */
  static constexpr inline uint8_t smbusCrc8Block(const uint8_t *data, uint8_t len);

  I2cBase& port;
  HPT5K0::ePsuI2cAddr devAddr;
  const uint32_t psuIndex;  // 0-indexed
  // This is the default values of registers from the HPT5K0Config.h/cpp specific
  // to this PSU.
  const psuCmdDefaultValues_t *cmdDefaults;
  const uint32_t cmdDefaultsSize;

  bool psuStatusIsSorted = false;
  PsuStatusRegs_t psuStatus;
  Logging::DeltaLoggerPtr<float, HPT5K0DolphinInfo_t::nStatusRegs> deltaLoggerStatus;

  bool initLogging();

  // Read in given register to internal shadow copy.
  // Used purely for DeltaLogger.
  bool readInRegister(psuCmdName_t cmd);

  // Only read in status registers set in HPT5K0Config.h/cpp
  // Used purely for DeltaLogger.
  bool readInStatusRegisters();

  // Reads in all registers to internal shadow copy.
  // Used purely for DeltaLogger.
  bool readInAllRegisters();

  const regData_t getAllData() {
    return regData;
  }

};

#endif /* SRC_MANTAHW_HPT5K0_H_ */
