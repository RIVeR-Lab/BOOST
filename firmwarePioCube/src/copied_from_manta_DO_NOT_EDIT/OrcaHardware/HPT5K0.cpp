/*
 * HPT5K0.cpp
 *
 *  Created on: Aug 24, 2022
 *      Author: David Antaki
 */

#include <OrcaHardware/HPT5K0.h>

HPT5K0::HPT5K0(Logging &log, I2cBase& i2cbus2, HPT5K0::ePsuI2cAddr i2cAddr, uint32_t psuIndex,
               const psuCmdDefaultValues_t defaults[], const uint32_t defaultsSize, Manta::LogSpecs::Subject subj,
               PsuStatusRegs_t statusRegsConfig)
    : port(i2cbus2),
      devAddr(i2cAddr),
      psuIndex(psuIndex),
      cmdDefaults(defaults),
      cmdDefaultsSize(defaultsSize),
      deltaLoggerStatus(subj, log, 0.0f) {

  bool success = true;

  /* ------ Ensure the HPT5K0Config::statusRegs is constructed correctly for DeltaLogging. ------ */
  // The following must be true:
  // If psuStatusIndex < LAST_ITEM.
  // There cannot be duplicates of psuStatusIndex.
  for (size_t i = 0; success && i < statusRegsConfig.size(); i++) {
    // Check no duplicates of PsuStatusIndex_t
    for (size_t j = i + 1; success && j < statusRegsConfig.size() - 1; j++) {
      if (statusRegsConfig[i].psuStatusIndex == statusRegsConfig[j].psuStatusIndex
          && statusRegsConfig[i].psuStatusIndex != PsuStatusIndex_t::NONE) {
        success = false;
      }
    }
    // Check no duplicates of PsuCmdName_t
    for (size_t j = i + 1; success && j < statusRegsConfig.size() - 1; j++) {
      if (statusRegsConfig[i].psuStatusIndex == statusRegsConfig[j].psuStatusIndex
          && statusRegsConfig[i].index != psuCmdName_t::UNKNOWN) {
        success = false;
      }
    }
    // Check no block registers in psuStatus because they cannot be converted to floats easily.
    for (size_t j = i + 1; success && j < statusRegsConfig.size() - 1; j++) {
      if (HPT5K0Info_t::getCmd(statusRegsConfig[i].index).regType == HPT5K0Info_t::etBlock) {
        success = false;
      }
    }
  }
  /* ------ END Ensure the HPT5K0Config::statusRegs is constructed correctly for DeltaLogging. ------ */

  /* ---- Ensure HPT5K0Config::psu#config is constructed corrected. ---- */
  // Ensure the defaults array matches it length
  size_t regCount = 1;
  for (size_t i = 0; success && defaults[i].index != psuCmdName_t::LAST_ITEM; i++) {
    regCount++;
  }
  if (regCount != cmdDefaultsSize) {
    success = false;
  }
  // Check no duplicates of PsuCmdName_t that are not read-only commands.
  for (size_t i = 0; success && i < defaultsSize; i++) {
    psuCmd_t fullCmd = HPT5K0Info_t::getCmd(defaults[i].index);
    for (size_t j = i + 1; success && j < defaultsSize; j++) {
      if ((defaults[i].index == defaults[j].index) && (fullCmd.regAccess != regAccess_t::RO)) {
        success = false;
      }
    }
  }

  // Check that the register type in the config array matches the integral type that stores its data.
  for (size_t i = 0; success && defaults[i].index != psuCmdName_t::LAST_ITEM; i++) {
    psuCmd_t fullCmd = HPT5K0Info_t::getCmd(defaults[i].index);
    if (HPT5K0Info_t::getRegIntegralType(fullCmd.regType) != defaults[i].defaultValue.tag) {
      success = false;
    }
  }
  /* ---- END Ensure HPT5K0Config::psu#config is constructed corrected. ---- */

  // Check that only block format registers have a register length over 2 bytes.
  for (size_t i = 0; success && i < HPT5K0DolphinInfo_t::LAST_ITEM; i++) {
    psuCmd_t fullCmd = HPT5K0Info_t::getCmd(i);
    if (fullCmd.regLen > 2 && !(isBlockCommand(fullCmd.index))) {
      success = false;
    }
  }

  if (success) {
    // Init this.psuStatus
    // This ensures that the registers found in HPT5K0Config.h/cpp are found in this.psuStatus in the order found in HPT5K0Info::PsuStatusIndex_t enum.
    // Basically sorts the array.
    for (size_t i = 0; i < statusRegsConfig.size(); i++) {
      psuStatus[static_cast<uint32_t>(statusRegsConfig[i].psuStatusIndex)] = statusRegsConfig[i];
    }
    psuStatusIsSorted = true;
  } else {
    psuStatusIsSorted = false;
  }
}

// Initializes this PSU and ensures it is set correctly.
bool HPT5K0::initialize() {
  bool success = true;
  LOGEVENT("PSU%d initializing...", psuIndex+1);

  if (!psuStatusIsSorted) {
    success = false;
  }

  if (success) {
    // Check that the PsuCmds was constructed correctly.
    // Make sure the following is true:
    // reglen=0 if and only if command type=write-only.
    // There was previously a bug where this was not true and was causing problems.
    for (uint32_t i = 0; success && i < psuCmdName_t::LAST_ITEM; i++) {
      psuCmd_t fullCmd = HPT5K0Info_t::getCmd(i);
      if ((fullCmd.regAccess == regAccess_t::WO && fullCmd.regLen != 0)
          || (fullCmd.regAccess != regAccess_t::WO && fullCmd.regLen == 0)) {
        LOGERROR("PSU%d: PSU register lengths are 0 if and only if it is a write-only register.", psuIndex + 1);
        success = false;
      }
    }

    // Check that all PSU register match it's configuration.
    success = success && checkPsuConfiguration();

    // Init DeltaLogging
    success = success && initLogging();
  }

  if (success) {
    initialized = true;
    LOGEVENT("PSU%d INITIALIZED SUCCESSFULLY", psuIndex + 1);
  } else {
    initialized = false;
    LOGERROR("PSU%d FAILED TO INITIALIZE", psuIndex + 1);
  }
  return success;
}

/**
 * Will check the current PSUs register settings and whether they match its configuration stored in cmdDefaults[].
 *
 * @return False if any configuration does not match.
 */
bool HPT5K0::checkPsuConfiguration() {
  bool success = true;
  PMResponse_t data{};

  // Clear any faults
  data = PMResponse_t::fromUInt16(1);
  success = success && Write(psuCmdName_t::CLEAR_FAULTS, data, true);

  // Restore user settings from EEPROM (this is also done automatically on power cycle)
  data = PMResponse_t::fromUInt16(1);
  success = success && Write(psuCmdName_t::RESTORE_USER_ALL, data, true);

  // First check the default value of OPERATION because once we turn it to OFF, it may not match what we want it to be at default.
  success = success && Read(psuCmdName_t::OPERATION, data, true);
  for (size_t i = 0; success && cmdDefaults[i].index != psuCmdName_t::LAST_ITEM; i++) {
    HPT5K0Info_t::psuCmd_t fullCmd = HPT5K0Info_t::getCmd(cmdDefaults[i].index);
    if(fullCmd.index == psuCmdName_t::OPERATION) {
      if(!cmdDefaults[i].defaultValue.equal(data, 0.0)) {
        success = false;
        LOGERROR("PSU%d setting incorrectly configured; cmd=(0x%X)%s, value=(dec)%1.2f", psuIndex + 1,
                 cmdDefaults[i].index, fullCmd.cmdNameStr, data.data.i);
      }
    }
  }

  // Ensure the PSU is off
  data = PMResponse_t::fromUInt16(HPT5K0Info_t::OPERATION_OFF);
  success = success && Write(psuCmdName_t::OPERATION, data, true);

  // Loop through each command in defaults and check that that is what the PSU has.
  for (size_t i = 0; success && cmdDefaults[i].index != psuCmdName_t::LAST_ITEM; i++) {
    HPT5K0Info_t::psuCmd_t fullCmd = HPT5K0Info_t::getCmd(cmdDefaults[i].index);

    // Do not check OPERATION register
    if(cmdDefaults[i].index != psuCmdName_t::OPERATION) {
      if (!HPT5K0Info_t::isReadCmd(fullCmd.index)) {
        LOGERROR("PSU%d Write-only commands are not supported as part of PSU configuration.", psuIndex + 1);
        success = false;
      }

      PMResponse_t psuReadOut { };
      bool set_correctly = true;

      // If read-only command then loop back through entire defaults array and find at least 1
      // setting that is correct. This is to support having multiple acceptable values for read-only registers.
      if (fullCmd.regAccess == regAccess_t::RO) {
        // Loop back through entire defaults array
        set_correctly = false;
        for (size_t k = 0; success && cmdDefaults[k].index != psuCmdName_t::LAST_ITEM; k++) {
          // Only if same command index
          if (cmdDefaults[k].index == fullCmd.index) {
            success = success && Read(fullCmd.index, psuReadOut, true);
            // Check if the register is set to what we want
            // If correct, break.
            if (psuReadOut.equal(cmdDefaults[k].defaultValue, 0.1)) {
              set_correctly = true;
              break;
            }
          }
        }
        // Else if not read-only command.
      } else {
        // Check if the register is set to what we want
        success = success && Read(fullCmd.index, psuReadOut, true);
        if (!(psuReadOut.equal(cmdDefaults[i].defaultValue, 0.1))) {
          set_correctly = false;
        }
      }

      if (!set_correctly) {
        success = false;
        float tempRegFloat = 0.0;
        regValueToFloat(cmdDefaults[i].index, regData, tempRegFloat);
        LOGERROR("PSU%d setting incorrectly configured; cmd=(0x%X)%s, value=(dec)%1.2f", psuIndex + 1,
                 cmdDefaults[i].index, fullCmd.cmdNameStr, tempRegFloat);
      }
    }
  }

  if(!success) {
    LOGERROR("PSU%d is configured INCORRECTLY.", psuIndex + 1);
  } else {
    LOGEVENT("PSU%d is configured CORRECTLY.", psuIndex + 1);
  }

  return success;
}

// Only use from BoardCLI!!
// This function will delay multiple times to give the PSU time to save its settings to EEPROM.
// During testing, if the 10s Sleep() was not there, the settings would not get saved to EEPROM.
// Immediately after adding the 10s delay, the settings were saved to EEPROM.
bool HPT5K0::reconfigurePsu(State_t &s, ePaper &view) {
  bool success = true;
  PMResponse_t data{};
  LOGEVENT("PSU%d Reconfiguring PSU. This could take up to 30sec.", psuIndex + 1);

  // Restore factory defaults
  data = PMResponse_t::fromUInt16(1);
  success = success && Write(psuCmdName_t::RESTORE_DEFAULT_ALL, data, true);
  success = success && Write(psuCmdName_t::STORE_USER_ALL, data, true);
  LOGEVENT("PSU%d Reseting PSU to factory defaults. Delaying 10s...", psuIndex + 1);
  Sleep(10000);

  // Loop through all cmdDefaults and set them.
  for (size_t i = 0; success && cmdDefaults[i].index != psuCmdName_t::LAST_ITEM; i++) {
    HPT5K0Info_t::psuCmd_t fullCmd = HPT5K0Info_t::getCmd(cmdDefaults[i].index);

    // If writable command, then set it.
    // Else leave it alone.
    // This will not check read-only commands.
    if (HPT5K0Info_t::isWriteCmd(fullCmd.index)) {
      success = success && Write(cmdDefaults[i].index, cmdDefaults[i].defaultValue, true);

      // Now check again that we set it correctly
      PMResponse_t psuReadOut{};
      success = success && Read(cmdDefaults[i].index, psuReadOut, true);
      float tempRegFloat = 0.0;
      regValueToFloat(cmdDefaults[i].index, regData, tempRegFloat);

      if(!psuReadOut.equal(cmdDefaults[i].defaultValue, 0.1)){
        success = false;
        LOGEVENT("PSU%d setting did not get set correctly; cmd=(0x%X)%s, value=(dec)%1.2f, success=%s", psuIndex + 1, cmdDefaults[i].index,
                 fullCmd.cmdNameStr, tempRegFloat, (success ? "true" : "false"));
      } else {
        LOGEVENT("PSU%d setting set; cmd=(0x%X)%s, value=(dec)%1.2f, success=%s", psuIndex + 1, cmdDefaults[i].index,
                 fullCmd.cmdNameStr, tempRegFloat, (success ? "true" : "false"));
      }
    } else {
      // leave it alone
    }
  }

  // Save to EEPROM
  if (success) {
    LOGEVENT("PSU%d Saving PSU settings to EEPROM. Delaying 10s...", psuIndex+1);
    data = PMResponse_t::fromUInt16(1);
    success = success && Write(psuCmdName_t::STORE_USER_ALL, data, true);
    // This delay is necessary to give time to write settings to EEPROM.
    // They do not give an EEPROM reading/writing time in the datasheet.
    s.prgState = getSelectedState_t(sSAVING_TO_EEPROM);
    s.additionalInfo = "";
    view.updateDisplay(s);
    Sleep(10000);
    LOGEVENT("PSU%d Done saving PSU settings to EEPROM.", psuIndex+1);
  }

  // Check that settings were properly saved to EEPROM
  success = success && checkPsuConfiguration();

  if(success) {
    LOGEVENT("PSU%d Reconfiguring SUCCEEDED.", psuIndex + 1);
  } else {
    LOGEVENT("PSU%d Reconfiguring FAILED.", psuIndex + 1);
  }

  return success;
}

bool HPT5K0::initLogging() {
  bool success = true;

  if (!psuStatusIsSorted) {
    LOGERROR("PSU%d: Failed init logging due to failure to construct psuStatus", psuIndex + 1);
    return false;
  }

// set periods & tolerance
  deltaLoggerStatus.setTimeIncrement(1000);
  deltaLoggerStatus.setEpsilon(0.2f);

  // Map the PSU1Status values to the PSU1DeltaLoggerStatus values
  for (size_t reg = 0; reg < psuStatus.size(); reg++) {
    success = success && deltaLoggerStatus.setValPtr(reg, psuStatus[reg].lastReading);
  }

  return success;
}

/**
 * Reads in the given register from the PSU and also stores in shadow copy.
 * Checks if the commands is a voltage command and if so, will convert the register
 * value to a voltage in volts.
 *
 * 8, 16 or 32 bit data is all converted to 32 (no sign extension)
 * linear data is converted to float
 * block data is returned as received
 */
bool HPT5K0::Read(psuCmdName_t cmd, PMResponse_t &outResponse, bool force) {
  int success = 9999999;
  psuCmd_t fullCmd = HPT5K0Info_t::getCmd(cmd);

  // Force read even if the PSU hasn't been initialized
  TempInitialize tempInit(initialized, force);

  if (!initialized) {
    LOGERROR("PSU%d has not been initialized!!", psuIndex + 1);
    success = 0;
  }

// Make sure the command is a read command
  if (!HPT5K0Info_t::isReadCmd(cmd)) {
    LOGERROR("PSU%d: This command is not a read command.", psuIndex + 1);
    success = 0;
  }

  if (success != 0) {
    // Add device address, write bit, command code, read bit.
    uint8_t pFullOutput[kCRCReadDataBufSize] = { 0 };
    uint32_t pFullOutputI = 0;
    pFullOutput[pFullOutputI++] = (static_cast<uint8_t>(devAddr) << 1) | HPT5K0Info_t::WRITE_BIT;
    pFullOutput[pFullOutputI++] = fullCmd.cmdCode;
    pFullOutput[pFullOutputI++] = (static_cast<uint8_t>(devAddr) << 1) | HPT5K0Info_t::READ_BIT;

    // Read data bytes
    // Need extra byte that represents the number of bytes in the received byte stream, for block commands.
    if (success != 0 && isBlockCommand(cmd)) {
      success = port.read(static_cast<uint8_t>(devAddr), fullCmd.cmdCode, kRegAddrSize, &pFullOutput[pFullOutputI],
                          kBlockReadBytesByteSize + fullCmd.regLen + kCRCSize, WAITTIME_MS, true, true);
      pFullOutputI += fullCmd.regLen + kCRCSize + kBlockReadBytesByteSize;
    } else if (success != 0) {
      success = port.read(static_cast<uint8_t>(devAddr), fullCmd.cmdCode, kRegAddrSize, &pFullOutput[pFullOutputI],
                          fullCmd.regLen + kCRCSize, WAITTIME_MS);
      pFullOutputI += fullCmd.regLen + kCRCSize;
    }

// Calculate CRC8 on all bytes excluding CRC byte
    uint8_t receivedCrc = pFullOutput[pFullOutputI - 1];
    uint8_t expectedCrc = smbusCrc8Block(pFullOutput, pFullOutputI - 1);

//  Check CRC
    if (receivedCrc != expectedCrc) {
      LOGERROR("PSU%d read failed, cmd: (0x%X)%s, recieved CRC: 0x%X, expected CRC: 0x%X", psuIndex + 1, cmd,
               fullCmd.cmdNameStr, receivedCrc, expectedCrc);
      success = 0;
    } else {
      uint16_t data { };
      switch (fullCmd.regType) {
        case HPT5K0Info_t::etBlock: {
          memcpy(outResponse.data.block, &pFullOutput[4], fullCmd.regLen);
          outResponse.tag = PMResponse_t::eBlock;
        }
          break;
        default: {
          // Check length. All non-block commands should have register length of 0, 1, or 2 bytes.
          if (fullCmd.regLen > 2) {
            LOGERROR("PSU%d This PSU command is not below a length of 2.", psuIndex + 1);
            success = false;
          }
          // Get uint16_t value
          if (fullCmd.regLen == 1) {
            data = static_cast<uint16_t>(pFullOutput[3]);
          } else {
            data = static_cast<uint16_t>(pFullOutput[3]) | (static_cast<uint16_t>(pFullOutput[4]) << 8);
          }

          switch (fullCmd.regType) {
            case HPT5K0Info_t::etNonOutVoltLinearFormat: {
              // Convert the register value to float.
              success = success && linearFormatNonOutVoltRegToFloat(cmd, data, outResponse.data.f);
              outResponse.tag = PMResponse_t::eFloat;
            }
              break;
            case HPT5K0Info_t::etOutVoltLinearFormat: {
              // Convert the register value to float.
              success = success && linearFormatOutVoltRegToFloat(cmd, data, outResponse.data.f);
              outResponse.tag = PMResponse_t::eFloat;
            }
              break;
            case HPT5K0Info_t::etOther: {
              outResponse.data.i = data;
              outResponse.tag = PMResponse_t::eUint16;
            }
              break;
            default: {
              LOGERROR("PSU%d Execution should not have reached here.", psuIndex + 1);
            }
              break;
          };
        }

          break;
      };

      // Also copy to the internal shadow copy
      if (success) {
        if (isBlockCommand(fullCmd.index)) {
          for (size_t i = 0; i < fullCmd.regLen; i++) {
            regData[fullCmd.dataStartIndex + i] = outResponse.data.block[i];
          }
        } else {
          uint16_t dataTemp = data;
          for (size_t i = 0; i < fullCmd.regLen; i++) {
            regData[fullCmd.dataStartIndex + i] = (dataTemp & 0x00ff);
            dataTemp = dataTemp >> 8;
          }
        }
      }
    }
  }
  return success;
}

bool HPT5K0::Write(psuCmdName_t cmd, const PMResponse_t writeData, bool force) {
  bool success = true;
  psuCmd_t fullCmd = HPT5K0Info_t::getCmd(cmd);

  // Force write even if the PSU hasn't been initialized
  TempInitialize tempInit(initialized, force);

  if (!initialized) {
    LOGERROR("PSU%d has not been initialized!!", psuIndex + 1);
    success = false;
  }

  if (isBlockCommand(cmd) && HPT5K0Info_t::isWriteCmd(cmd)) {
    LOGERROR("PSU%d: Block writing is currently not implemented. Block reading IS implemented however.", psuIndex + 1);
    success = false;
  }

// Make sure the command is a write command
  if (!HPT5K0Info_t::isWriteCmd(cmd)) {
    LOGERROR("PSU%d: This command is not a write command.", psuIndex + 1);
    success = false;
  }

  // Check that the integral type of outReponse matches the corresponding integral type that this psuCmdName_t's data should be stored as.
  if (HPT5K0Info_t::getRegIntegralType(fullCmd.regType) != writeData.tag) {
    LOGERROR("PSU%d The given write data's integral type does not match the register's integral type.");
    success = 0;
  }

  PMResponse_t writeDataConverted { };
  writeDataConverted.tag = PMResponse_t::eBlock;  // We should always write from the block.
  if (success) {
    switch (fullCmd.regType) {
      case HPT5K0Info_t::etBlock: {
        // Check correct tag is set.
        if (writeData.tag != PMResponse_t::eBlock) {
          success = false;
          LOGERROR("PSU%d Incorrect tag set for writing this PSU register %d.", psuIndex + 1, fullCmd.cmdNameStr);
        }
        // Do nothing to the data.
        memcpy(writeDataConverted.data.block, writeData.data.block, sizeof(writeData.data.block));
      }
        break;
      case HPT5K0Info_t::etNonOutVoltLinearFormat: {
        // Check correct tag is set.
        if (writeData.tag != PMResponse_t::eFloat) {
          success = false;
          LOGERROR("PSU%d Incorrect tag set for writing this PSU register %d.", psuIndex + 1, fullCmd.cmdNameStr);
        }
        // Convert the register value from float.
        success = success && floatToLinearFormatNonOutVoltReg(cmd, writeData.data.f, writeDataConverted.data.i);
      }
        break;
      case HPT5K0Info_t::etOutVoltLinearFormat: {
        // Check correct tag is set.
        if (writeData.tag != PMResponse_t::eFloat) {
          success = false;
          LOGERROR("PSU%d Incorrect tag set for writing this PSU register %d.", psuIndex + 1, fullCmd.cmdNameStr);
        }
        // Convert the register value from float.
        success = success && floatToLinearFormatOutVoltReg(cmd, writeData.data.f, writeDataConverted.data.i);
      }
        break;
      case HPT5K0Info_t::etOther: {
        // Check correct tag is set.
        if (writeData.tag != PMResponse_t::eUint16) {
          success = false;
          LOGERROR("PSU%d Incorrect tag set for writing this PSU register %d.", psuIndex + 1, fullCmd.cmdNameStr);
        }
        // Do nothing to the data.
        writeDataConverted.data.i = writeData.data.i;
      }
        break;
      default: {
        LOGERROR("PSU%d Execution should not have reached here.", psuIndex + 1);
      }
        break;
    }

    // Disable write protect
    success = success && enableWriteProtect(false);

    // Send the actual command
    success = success && doWrite(cmd, writeDataConverted);

    // Enable write protect
    success = success && enableWriteProtect(true);

    // Also copy to the internal shadow copy
    memcpy(&(regData[fullCmd.dataStartIndex]), writeDataConverted.data.block, fullCmd.regLen);
  }

  return success;
}

bool HPT5K0::doWrite(psuCmdName_t cmd, const PMResponse_t writeData) {
  bool success = true;
  HPT5K0Info_t::psuCmd_t fullCmd = HPT5K0Info_t::getCmd(cmd);

  // Full transaction bytes excluding control bits (ACK/NACK/START/STOP/RESTART)
  // Length = [addr+w][cmdCode][regLen/data]
  if (kSlaveAddrSize + kRegAddrSize + fullCmd.regLen + kCRCSize > kCRCDataBufSize
      || fullCmd.regLen > kMaxNonBlockRegSize) {
    LOGERROR("PSU%d: The data sent by a non-block write command should always be less than 4 bytes.", psuIndex + 1);
    success = false;
  }

  if (writeData.tag != PMResponse_t::eBlock) {
    LOGERROR("PSU%d: The data's tag type must be eBlock.", psuIndex + 1);
    success = false;
  }

  std::array<uint8_t, kCRCDataBufSize> fullTransactionBuf = { 0x0 };
  size_t dataBufI = 0;

  fullTransactionBuf[dataBufI++] = (static_cast<uint8_t>(devAddr) << 1) | HPT5K0Info_t::WRITE_BIT;
  fullTransactionBuf[dataBufI++] = fullCmd.cmdCode;

  std::copy_n(std::cbegin(writeData.data.block), fullCmd.regLen, fullTransactionBuf.begin() + dataBufI);
  dataBufI += fullCmd.regLen;

  // Calculate the CRC8 and add it to data.
  // Subtract 1 to exclude the last byte which is the crc byte which is not populated at this point.
  uint8_t crc = smbusCrc8Block(fullTransactionBuf.data(), kSlaveAddrSize + kRegAddrSize + fullCmd.regLen);

  // Set the last byte to be the crc byte
  fullTransactionBuf[dataBufI++] = crc;

  if (success) {
    // Send command
    success = success
        && port.write(static_cast<uint8_t>(devAddr), fullCmd.cmdCode, kRegAddrSize, &fullTransactionBuf[2],
                      fullCmd.regLen + kCRCSize, WAITTIME_MS);
  }

  if (!success) {
    LOGERROR("PSU%d write failed, cmd: (0x%X)%s, crc: 0x%X", psuIndex + 1, cmd, fullCmd.cmdNameStr, crc);
  }

  return success;
}

bool HPT5K0::enableWriteProtect(bool en) {
  bool success = true;

  PMResponse_t dataBuf { };
  if (en) {
    dataBuf = PMResponse_t::fromUInt16(HPT5K0Info_t::WRITE_PROTECT_t::WRITE_PROTECT_ENABLE);
  } else {
    dataBuf = PMResponse_t::fromUInt16(HPT5K0Info_t::WRITE_PROTECT_t::WRITE_PROTECT_DISABLE);
  }
  dataBuf.tag = PMResponse_t::eBlock;  // Can only call doWrite with eBlock tag.

  doWrite(HPT5K0DolphinInfo_t::WRITE_PROTECT, dataBuf);

// Check it was written correctly.
  PMResponse_t wp = PMResponse_t::fromUInt16(0xff);
  success = success && Read(psuCmdName_t::WRITE_PROTECT, wp);
  dataBuf.tag = PMResponse_t::eUint16;  // Change type back.
  success = success && wp.equal(dataBuf, 0);
  return success;
}

/**
 * See the following for explanations:
 * https://www.ti.com/lit/an/slua475/slua475.pdf?ts=1662062124341&ref_url=https%253A%252F%252Fwww.google.com%252F
 * http://ww1.microchip.com/downloads/en/Appnotes/doc2583.pdf
 * https://en.wikipedia.org/wiki/Cyclic_redundancy_check
 * http://smbus.org/faq/faq_main.htm#:~:text=What%20is%20the%20CRC%2D8%20polynomial%20for%20SMBus%3F&text=C(x)%20%3D%201%200000%200111.
 * http://sbs-forum.org/marcom/dc2/20_crc-8_firmware_implementations.pdf
 * https://crccalc.com/
 */
constexpr inline uint8_t HPT5K0::smbusCrc8(uint8_t inCrc, uint8_t inData) {
  uint8_t i { };
  uint8_t data { };

  data = inCrc ^ inData;

  for (i = 0; i < 8; i++) {
    if ((data & 0x80) != 0) {
      data <<= 1;
      data ^= 0x07;
    } else {
      data <<= 1;
    }
  }
  return data;
}

constexpr inline uint8_t HPT5K0::smbusCrc8Block(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  while (len > 0) {
    crc = smbusCrc8(crc, *data++);
    len--;
  }
  return crc;
}

// Used only for DeltaLogger.
bool HPT5K0::readInRegister(psuCmdName_t cmd) {
  bool success = true;
  PMResponse_t throwAwayOutput { };
  success = success && Read(cmd, throwAwayOutput);
  return success;
}

// Used only for DeltaLogger.
bool HPT5K0::readInStatusRegisters() {
  bool success = true;
  for (size_t i = 0; success && psuStatus[i].psuStatusIndex != PsuStatusIndex_t::NONE && i < psuStatus.size(); i++) {
    // Only read in READ registers. Read() also checks this, but will log an error
    // when this is not an error in this case, therefore we check it here.
    if (HPT5K0Info_t::isReadCmd(psuStatus[i].index)) {
      success = success && readInRegister(psuStatus[i].index);
    }
  }
  return success;
}

// Used only for DeltaLogger.
bool HPT5K0::readInAllRegisters() {
  bool success = true;
  for (size_t i = 0; success && i < psuCmdName_t::LAST_ITEM; i++) {
    // Only read in READ registers. Read() also checks this, but will log an error
    // when this is not an error in this case, therefore we check it here.
    if (HPT5K0Info_t::isReadCmd(static_cast<HPT5K0Info_t::psuCmdName_t>(i))) {
      success = success && readInRegister(static_cast<HPT5K0Info_t::psuCmdName_t>(i));
    }
  }
  return success;
}

// Used only for DeltaLogger.
bool HPT5K0::checkPsuStatusRegisters() {
  if (!initialized) {
    LOGERROR("PSU%d has not been initialized!!", psuIndex + 1);
    return false;
  }

  bool success = true;
  success = success && readInStatusRegisters();
  if (deltaLoggerStatus.logTime()) {
    auto regDataCopy = getAllData();
    for (size_t i = 0; success && psuStatus[i].psuStatusIndex != PsuStatusIndex_t::NONE && i < psuStatus.size(); i++) {
      float tempRegVal = 0.0f;
      success = success && regValueToFloat(psuStatus[i].index, regDataCopy, tempRegVal);
      psuStatus[i].lastReading = tempRegVal;
    }
    success = success && deltaLoggerStatus.sendData();
  }

  return success;
}

// Used only for DeltaLogger.
bool HPT5K0::regValueToFloat(psuCmdName_t reg, regData_t regData, float& output) {
  bool success = true;
  psuCmd_t fullCmd = HPT5K0Info_t::getCmd(reg);

  if (!HPT5K0Info_t::isReadCmd(reg)) {
    LOGERROR("PSU Must be a read command : %s", fullCmd.cmdNameStr);
    success = false;
  }

  uint64_t tempVal = 0x0;
  for (int i = fullCmd.dataStartIndex + fullCmd.regLen - 1; i >= static_cast<int>(fullCmd.dataStartIndex); i--) {
    tempVal = (tempVal << 8) | regData[i];
  }

  switch (fullCmd.regType) {
    case HPT5K0Info_t::etBlock: {
      LOGERROR("PSU Using PSU block commands in telemetry is not supported!");
      success = false;
    }
      break;
    case HPT5K0Info_t::etNonOutVoltLinearFormat: {
      success = success && linearFormatNonOutVoltRegToFloat(reg, static_cast<uint16_t>(tempVal), output);
    }
      break;
    case HPT5K0Info_t::etOutVoltLinearFormat: {
      success = success && linearFormatOutVoltRegToFloat(reg, static_cast<uint16_t>(tempVal), output);
    }
      break;
    case HPT5K0Info_t::etOther: {
      output = static_cast<float>(tempVal);
    }
      break;
    default: {
      LOGERROR("Execution should never reach here!");
    }
      break;
  };

  return success;
}

constexpr bool HPT5K0::linearFormatOutVoltRegToFloat(psuCmdName_t cmd, uint16_t data, float& out) {
  bool success = true;
  HPT5K0Info_t::psuCmd_t fullCmd = HPT5K0Info_t::getCmd(cmd);

  if (!isLinearFormatOutVoltCommand(cmd)) {
    LOGERROR("PSU%d %s: This command is not a voltage command as specified in the voltageCmds array.", psuIndex + 1,
             fullCmd.cmdNameStr);
    return false;
  }

  // Check register length == 2
  if (fullCmd.regLen != 2) {
    LOGERROR("PSU %s: This command must have a reglen of 2 bytes.", fullCmd.cmdNameStr);
    return false;
  }

  // Get and check the voltage mode
  static_assert(HPT5K0Info_t::getCmd(psuCmdName_t::VOUT_MODE).regLen == sizeof(uint8_t), "Must have regLen=1");
  PMResponse_t voutMode { };
  success = success && Read(psuCmdName_t::VOUT_MODE, voutMode);
  // Ensure the mode bits = 0b00011000. If they don't then I don't know what to do (check the datasheet).
  // The PSU models that we use should always have this register set to 0x18.
  if ((voutMode.data.i & ~0x18) != 0x0) {
    LOGERROR(
        "PSU%d Mode bits do not equal '0x18=0b00011000', check the datasheet under section \"3.11 Data Format for Output Voltage\".",
        psuIndex + 1);
    return false;
  }

  // rawVal * 2^-8
  out = ((float) data) / 256.0;

  return success;
}

constexpr bool HPT5K0::floatToLinearFormatOutVoltReg(psuCmdName_t cmd, float data, uint16_t &outData) {
  bool success = true;
  HPT5K0Info_t::psuCmd_t fullCmd = HPT5K0Info_t::getCmd(cmd);

  if (!isLinearFormatOutVoltCommand(cmd)) {
    LOGERROR("PSU%d %s: This command is not a voltage command as specified in the voltageCmds array.", psuIndex + 1,
             fullCmd.cmdNameStr);
    return false;
  }

  // Check register length == 2
  if (fullCmd.regLen != 2) {
    LOGERROR("PSU%d %s: This command must have a reglen of 2 bytes.", psuIndex + 1, fullCmd.cmdNameStr);
    return false;
  }

  // Get and check the voltage mode
  static_assert(HPT5K0Info_t::getCmd(psuCmdName_t::VOUT_MODE).regLen == sizeof(uint8_t), "Must have regLen=1");
  PMResponse_t voutMode { };
  success = success && Read(psuCmdName_t::VOUT_MODE, voutMode);
  // Ensure the mode bits = 0b00011000. If they don't then I don't know what to do (check the datasheet).
  // The PSU models that we use should always have this register set to 0x18.
  if ((voutMode.data.i & ~0x18) != 0x0) {
    LOGERROR(
        "PSU%d Mode bits do not equal '0x18=0b00011000', check the datasheet under section \"3.11 Data Format for Output Voltage\".",
        psuIndex + 1);
    return false;
  }

  // data / 2^-8
  outData = static_cast<uint16_t>(data * 256.0f);

  return success;
}

// See datasheet "3.12 Data Format for Other Parameters"
constexpr bool HPT5K0::linearFormatNonOutVoltRegToFloat(psuCmdName_t cmd, const uint16_t data, float &out) {
  HPT5K0Info_t::psuCmd_t fullCmd = HPT5K0Info_t::getCmd(cmd);
  bool success = true;

  if (!isLinearFormatNonOutVoltCommand(cmd)) {
    LOGERROR(
        "PSU %s: This command is not a linear format register that is NOT an output voltage register command per HPT5K0 datasheet "
        "section \"3.12 Data Format for Other Parameters\".",
        fullCmd.cmdNameStr);
    return false;
  }

  // Linear Format:
  // [15][14][13][12][11]   [10][9][8][7][6][5][4][3][2][1][0]
  //      Exponent N                     Mantissa Y
  // X = Y * 2^N
  // Where:
  //  X: real world value
  //  Y: 11 bits, signed twos complement binary integer mantissa
  //  N: signed 5 bits twos complement binary integer exponent

  // Mantissa is 11bits two's complement
  uint16_t mantissa2Comp = (data & 0x7FF);  // 0x7FF=0b0000011111111111
  int mantissa = 0;
  if (((mantissa2Comp & 0x0400) >> 10) == 1) {  // If 11th bit is 1, then it is negative exponent
    mantissa2Comp = mantissa2Comp & 0x03FF;  // Take 10 lowest bits (i.e. remove 11th bit).
    mantissa2Comp = (~mantissa2Comp) + 1;  // Flip and add 1
    mantissa2Comp = mantissa2Comp & 0x03FF;  // Take 10 lowest bits in case of overflow.
    mantissa = -mantissa2Comp;  // Add the negative sign
  } else {  // Else it is a positive number and 11th bit==0
    mantissa = mantissa2Comp;
  }

  uint8_t exp2Comp = (data >> 11);
  int8_t exp = 0;
  // Exponent is 5bits two's complement
  if (((exp2Comp & 0x10) >> 4) == 1) {  // If 5th bit is 1, then it is negative exponent
    exp2Comp = exp2Comp & 0x0F;  // Take 4 lowest bits (i.e. remove 5th bit).
    exp2Comp = (~exp2Comp) + 1;  // Flip and add 1
    exp2Comp = exp2Comp & 0x0F;  // Take 4 lowest bits in case of overflow.
    exp = -exp2Comp;  // Add the negative sign
  } else {  // Else it is a positive number and 11th bit==0
    exp = exp2Comp;
  }

  // out = "X:real world value"
  out = mantissa * (pow(2, exp));

  return success;
}

// TODO: Need to check/re-write this. Currently NOT correct but the generally form is there.
// Make constexpr when you implement it and remove LOGERROR()'s so that it can be constexpr.
//
// See datasheet "3.12 Data Format for Other Parameters"
// A function that converts a float to uint16_t with a an 11bit 2's complement mantissa and 5bit 2's complement exponent.
bool HPT5K0::floatToLinearFormatNonOutVoltReg(psuCmdName_t cmd, float data, uint16_t &out) {
  LOGERROR("PSU%d floatToLinearFormatNonOutVoltReg() is not implemented/tested YET!!", psuIndex + 1);
  return false;

  // Linear Format:
  // [15][14][13][12][11]   [10][9][8][7][6][5][4][3][2][1][0]
  //      Exponent N                     Mantissa Y
  // X = Y * 2^N
  // Where:
  //  X: real world value
  //  Y: 11 bits, signed twos complement binary integer mantissa
  //  N: signed 5 bits twos complement binary integer exponent

  // Check if the input is in range
  if (data < -2048.0 || data > 2047.9375) {
    LOGERROR("Input value is out of range. Must be between -2048.0 and 2047.9375.");
    return false;
  }

  // Check if the input is a whole number
  if (data == static_cast<int>(data)) {
    // If it is a whole number, then the exponent is 0
    out = static_cast<uint16_t>(data);
    return true;
  }

  // Find the exponent
  int exp = 0;
  float inAbs = fabs(data);
  while (inAbs < 1.0) {
    inAbs *= 2.0;
    exp--;
  }
  while (inAbs >= 2.0) {
    inAbs /= 2.0;
    exp++;
  }

  // Find the mantissa
  int mantissa = static_cast<int>(inAbs * 2048.0);

  // Convert to 2's complement
  if (mantissa < 0) {
    mantissa = (~mantissa) + 1;
    mantissa = mantissa & 0x7FF;
    mantissa = -mantissa;
  }
  if (exp < 0) {
    exp = (~exp) + 1;
    exp = exp & 0x0F;
    exp = -exp;
  }

  // Combine the exponent and mantissa
  out = (exp << 11) | (mantissa & 0x7FF);

  return true;
}

/* -------------------- STATIC TESTS -------------------- */
static constexpr float linearFormatToValue(HPT5K0Info_t::psuCmdName_t cmd, const uint16_t data) {
  float tempResult = 0.0;
  HPT5K0::linearFormatNonOutVoltRegToFloat(cmd, data, tempResult);
  return tempResult;
}
// Test positive exponent=2, positive mantissa=1.
static constexpr uint16_t linearFormatToValueTest1RawReg = 0x1001;
static constexpr float linearFormatToValueTest1ActualResult = linearFormatToValue(HPT5K0DolphinInfo_t::READ_IOUT,
                                                                                  linearFormatToValueTest1RawReg);
static constexpr float linearFormatToValueTest1ExpectedResult = 4;
static_assert(linearFormatToValueTest1ActualResult == linearFormatToValueTest1ExpectedResult, "Test Fail");

// Test negative exponent, positive mantissa.
static constexpr uint16_t linearFormatToValueTest2RawReg = 0xf802;
static constexpr float linearFormatToValueTest2ActualResult = linearFormatToValue(HPT5K0DolphinInfo_t::READ_IOUT,
                                                                                  linearFormatToValueTest2RawReg);
static constexpr float linearFormatToValueTest2ExpectedResult = 1;
static_assert(linearFormatToValueTest2ActualResult == linearFormatToValueTest2ExpectedResult, "Test Fail");

// Test positive exponent, negative mantissa.
static constexpr uint16_t linearFormatToValueTest3RawReg = 0x17bb;
static constexpr float linearFormatToValueTest3ActualResult = linearFormatToValue(HPT5K0DolphinInfo_t::READ_IOUT,
                                                                                  linearFormatToValueTest3RawReg);
static constexpr float linearFormatToValueTest3ExpectedResult = -276;
static_assert(linearFormatToValueTest3ActualResult == linearFormatToValueTest3ExpectedResult, "Test Fail");

// Test negative exponent, negative mantissa.
static constexpr uint16_t linearFormatToValueTest4RawReg = 0xf7ec;
static constexpr float linearFormatToValueTest4ActualResult = linearFormatToValue(HPT5K0DolphinInfo_t::READ_IOUT,
                                                                                  linearFormatToValueTest4RawReg);
static constexpr float linearFormatToValueTest4ExpectedResult = -5;
static_assert(linearFormatToValueTest4ActualResult == linearFormatToValueTest4ExpectedResult, "Test Fail");
/* -------------------- END STATIC TESTS -------------------- */

/* -------------------- STATIC PRINTING HELPER -------------------- */
// The below functions are for printing out constexpr variables are compiletime. For debugging.
template<class T, T x, class F>
void transparent(F f) {
  f();
}

template<bool B>
constexpr void my_assert() {
  static_assert(B, "oh no");
}

template<int X>
void f() {
  transparent<int, X + 7>([] {
    transparent<long, X*X*X>([] {
          my_assert<X+10==-89>();});});
}
static constexpr void test1() {
//  f<(int)linearFormatToValueTest1ActualResult>();
//  f<(int)linearFormatToValueTest1ExpectedResult>();
//  f<(int)linearFormatToValueTest2ActualResult>();
//  f<(int)linearFormatToValueTest2ExpectedResult>();
}
/* -------------------- END STATIC PRINTING HELPER -------------------- */
