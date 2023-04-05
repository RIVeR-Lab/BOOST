#include "BatteryManager.h"

bool BatteryManager::loopHook() {
  LOGDEBUG("%s", __func__);
  monitorMcuTemp();
  return true;
}

bool BatteryManager::logLoopHook() {
#if ENABLE_BATTMANAGER_LOGLOOP
  LOGEVENT("%s", __func__);
  int32_t t = readMcuTemp();
  LOGINFO("MCUTemp(C): %d", t);
  LOGINFO("BatteryVolt(mv): %d", readInBattVolt_mv());
#endif
  return true;
}

int32_t BatteryManager::readInBattVolt_mv() {
  int32_t VRef_mv = ADCClass::readVref();
  LOGDEBUG("VRef(mv): %d", VRef_mv);

  // This is the voltage at R2
  int32_t adc_reading_raw_mv =
      ADCClass::readVoltage(VRef_mv, batt_volt_monitor_pin);
  LOGDEBUG("ADCReadingRaw(mv): %d", adc_reading_raw_mv);

  int32_t batt_volt_mv = get_volt_divider_vin_mv(adc_reading_raw_mv);
  return batt_volt_mv;
}

int32_t BatteryManager::readMcuTemp() {
  int32_t VRef = ADCClass::readVref();
  LOGDEBUG("VRef: %d", VRef);
  uint32_t atemp = analogRead(ATEMP);
  LOGDEBUG("ATEMP: %d", atemp);
  int32_t mcuTemp = (__LL_ADC_CALC_TEMPERATURE(VRef, atemp, LL_ADC_RESOLUTION));
  return mcuTemp;
}

void BatteryManager::monitorMcuTemp() {
  int32_t mcuTemp = readMcuTemp();
  if (mcuTemp > MAX_MCU_TEMP_DEGC) {
    while (1) {
      LOGERROR(
          "!!!MAYDAY!!! MCU TEMPERATUER TOO HIGH!!! SHUTOFF MINIBOT!!!: %d",
          mcuTemp);
      ROSLOGERROR(
          "!!!MAYDAY!!! MCU TEMPERATUER TOO HIGH!!! SHUTOFF MINIBOT!!!: %d",
          mcuTemp);
      delay(500);
    }
  }
}

bool BatteryManager::isBattDetected() {
  int32_t batt_volt_mv = readInBattVolt_mv();
  if (batt_volt_mv > 0) {
    return true;
  }
  return false;
}

bool BatteryManager::getRosBattStateMsg(sensor_msgs::BatteryState &batt_msg) {
  batt_msg.voltage = getBattVolt();
  batt_msg.current = 0;
  batt_msg.charge = 0;
  batt_msg.capacity = 0;
  batt_msg.design_capacity = 0;
  batt_msg.percentage = 0;
  batt_msg.power_supply_status =
      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  batt_msg.power_supply_health =
      sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  batt_msg.power_supply_technology =
      sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
  batt_msg.present = isBattDetected();
  return true;
}