#ifndef _BATT_MANAGER_H
#define _BATT_MANAGER_H

#include "ADC.h"
#include "Arduino.h"
#include "FakeThread.h"
#include "utils/log.h"
#include "utils/macros.h"
#include "sensor_msgs/BatteryState.h"

class BatteryManager : public FakeThread {
public:
  BatteryManager(uint32_t _batt_volt_monitor_pin)
      : FakeThread(LOOP_DELAY_MS, LOG_LOOP_DELAY_MS),
        batt_volt_monitor_pin(_batt_volt_monitor_pin) {}

  bool init() {
    INIT_HEADER
    ADCClass::initAdc();
    pinMode(batt_volt_monitor_pin, INPUT);
    INIT_FOOTER
  }

  bool loopHook() override;
  bool logLoopHook() override;

  float getBattVolt() { return static_cast<float>(getBattVolt_mv() * 1000.0); }
  int32_t getBattVolt_mv() { return readInBattVolt_mv(); }
  bool getRosBattStateMsg(sensor_msgs::BatteryState &batt_msg);
  bool isBattDetected();

  // In degress C
  int32_t readMcuTemp();

private:
  static constexpr uint32_t LOOP_DELAY_MS = 100;
  static constexpr uint32_t LOG_LOOP_DELAY_MS = 500;
  uint32_t batt_volt_monitor_pin;

  float last_batt_volt = 0;
  static constexpr float MAX_6S_LIPO_VOLTAGE = (4.2 * 6.0);
  static constexpr int32_t VOLT_DIVIDER_R1 = 220000;
  static constexpr int32_t VOLT_DIVIDER_R2 = 33000;
  static constexpr float VOLTAGE_DIVIDER_RATIO =
      (MAX_6S_LIPO_VOLTAGE * (VOLT_DIVIDER_R1 + VOLT_DIVIDER_R2)) /
      (VOLT_DIVIDER_R2 * 1024.0);
  static int32_t get_volt_divider_vin_mv(int32_t vout) {
    return (vout * (VOLT_DIVIDER_R1 + VOLT_DIVIDER_R2)) / VOLT_DIVIDER_R2;
  }
  int32_t readInBattVolt_mv();

  static constexpr int32_t MAX_MCU_TEMP_DEGC = 45;
  void monitorMcuTemp();
};

#endif // _BATT_MANAGER_H