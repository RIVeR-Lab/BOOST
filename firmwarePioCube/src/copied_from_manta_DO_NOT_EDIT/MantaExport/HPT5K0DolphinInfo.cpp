/*
 * HPT5K0DolphinInfo.cpp
 *
 *  Created on: Sep 6, 2022
 *      Author: DavidAntaki
 */

#include <MantaExport/HPT5K0DolphinInfo.h>
#include "HPT5K0Info.h"

constexpr std::array<HPT5K0DolphinInfo_t::StatusWordRegBit_t,
    static_cast<uint32_t>(HPT5K0DolphinInfo_t::StatusWordRegBitIndex_t::LAST_ITEM)> HPT5K0DolphinInfo_t::StatusWordRegBits;

bool HPT5K0DolphinInfo_t::isStatusWordBitSet(uint16_t regData, StatusWordRegBitIndex_t bit) {
  static_assert(HPT5K0Info_t::getCmd(psuCmdName_t::STATUS_WORD).regLen == sizeof(uint16_t), "Must have reglen of 2 bytes.");
  return regData & (0x1 << static_cast<uint32_t>(bit));
}
