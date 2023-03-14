#ifndef _DAVID_IMU_H_
#define _DAVID_IMU_H_

#include "utils/log.h"
#include "Adafruit_BNO055.h"
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <sensor_msgs/Imu.h>
#include "FakeThread.h"
#include <geometry_msgs/Point.h>
class BNO055Manager : public FakeThread {
public:
  BNO055Manager(int32_t sensorID, uint8_t address, TwoWire &bus);

  bool init() {
    bool success = true;

    while (!bno.begin(OPERATION_MODE_NDOF)) {
      LOGERROR("FAILED to init BNO055... Check your wiring or I2C ADDR!");
      delay(1000);
    }
    bno.setExtCrystalUse(true);
    LOGEVENT("BNO055 inited SUCCESSFULLY");

    return success;
  }

  
  bool loopHook() override;
	bool toRosImuMsg(sensor_msgs::Imu &imu_msg);
  bool readInAllImuData();
  
private:
  Adafruit_BNO055 bno;
  static constexpr uint32_t LOOP_DELAY_MS = 10;

  // Last readings
  sensors_event_t orientationDataEuler{};
  sensors_event_t angVelocityData{};
  sensors_event_t linearAccelData{};
  sensors_event_t magnetometerData{};
  sensors_event_t accelerometerData{};
  sensors_event_t gravityData{};
  imu::Quaternion orientationDataQuat{};

  void printAll();
  void printEvent(sensors_event_t* event);
};

#endif // _DAVID_IMU_H_