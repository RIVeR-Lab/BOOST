#ifndef _DAVID_IMU_H_
#define _DAVID_IMU_H_

#include "Adafruit_BNO055.h"
#include "FakeThread.h"
#include "utils/log.h"
#include <Adafruit_Sensor.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <utility/imumaths.h>
class BNO055Manager : public FakeThread {
public:
  BNO055Manager(Adafruit_BNO055 &_bno)
      : FakeThread(LOOP_DELAY_MS, LOG_LOOP_DELAY_MS), bno(_bno){};

  bool init() {
    bool success = true;
     LOGEVENT("Initializing...");

    success = success && bno.begin(OPERATION_MODE_NDOF);
    
    if (success) {
      bno.setExtCrystalUse(true);
    }

    if (!success) {
      LOGERROR("FAILED to init BNO055... Check your wiring or I2C ADDR!");
      initted = false;
    } else {
      LOGEVENT("Initialized SUCCESSFULLY");
      initted = true;
    }
    return success;
  }

  bool loopHook() override;
  bool logLoopHook() override;
  bool toRosImuMsg(sensor_msgs::Imu &imu_msg);
  bool readInAllImuData();

private:
  Adafruit_BNO055 &bno;
  static constexpr uint32_t LOOP_DELAY_MS = 10;
  static constexpr uint32_t LOG_LOOP_DELAY_MS = 500;
  bool initted = false;

  // Last readings
  sensors_event_t orientationDataEuler{};
  sensors_event_t angVelocityData{};
  sensors_event_t linearAccelData{};
  sensors_event_t magnetometerData{};
  sensors_event_t accelerometerData{};
  sensors_event_t gravityData{};
  imu::Quaternion orientationDataQuat{};

  void logAll();
  void printEvent(sensors_event_t *event);
};

#endif // _DAVID_IMU_H_