#ifndef _IMU_H_
#define _IMU_H_

#include "utils/log.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <sensor_msgs/Imu.h>
#include "FakeThread.h"
#include <geometry_msgs/Point.h>

class IMU : public FakeThread {
public:
  IMU(int32_t sensorID, uint8_t address, TwoWire &bus)
      : FakeThread(LOOP_DELAY_MS), bno(sensorID, address, &bus) {}

  bool init() {
    bool success = true;

    if (!bno.begin(OPERATION_MODE_NDOF)) {
      while (1) {
        LOGERROR("FAILED to init BNO055... Check your wiring or I2C ADDR!");
        delay(1000);
      }
    }

    return success;
  }

  
  bool loopHook() override;
	bool getAllImuData(sensor_msgs::Imu &imu_msg);
  
private:
  Adafruit_BNO055 bno;
  /* Set the delay between fresh samples */
  uint16_t BNO055_SAMPLERATE_DELAY_MS = 1000;
  static constexpr uint32_t LOOP_DELAY_MS = 1000;

  void printAll();
  void printEvent(sensors_event_t* event);
};

#endif // _IMU_H_