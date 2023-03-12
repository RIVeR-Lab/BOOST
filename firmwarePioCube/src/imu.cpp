#include "imu.h"
#include "RealMain.h"

DavidImu::DavidImu(int32_t sensorID, uint8_t address, TwoWire &bus)
    : FakeThread(LOOP_DELAY_MS), bno(sensorID, address, &bus) {}

bool DavidImu::loopHook() {
  LOGEVENT("IMU::loopHook()");
  readInAllImuData();
#if PRINT_IMU_DATA
  printAll();
#endif
  return true;
}

/**
 * @brief Get all IMU data and store it in the imu_msg
 * VECTOR_EULER: Three axis orientation data based on a 360° sphere 100Hz
 *
 * getQuat() Four point quaternion output for more accurate data manipulation
 * 100Hz
 *
 * VECTOR_GYROSCOPE: Angular velocity data in rad/s 100Hz
 *
 * VECTOR_ACCELEROMETER: Three axis of acceleration (gravity + linear motion) in
 * m/s^2 100Hz
 *
 * VECTOR_MAGNETOMETER: Three axis of magnetic field sensing in micro Tesla (uT)
 * 20Hz
 *
 * VECTOR_LINEARACCEL: Three axis of linear acceleration data (acceleration
 * minus gravity) in m/s^2 100Hz
 *
 * VECTOR_GRAVITY: Three axis of gravitational acceleration (minus any movement)
 * in m/s^2 100hz
 *
 * getTemp() Ambient temperature in degrees celsius 1Hz
 *
 */
bool DavidImu::readInAllImuData() {
  bool success = true;
  bno.getEvent(&orientationDataEuler, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  orientationDataQuat = bno.getQuat();

  return success;
}

// Convert IMU data to a ROS IMU message.
bool DavidImu::toRosImuMsg(sensor_msgs::Imu &imu_msg) {
  bool success = true;

  imu_msg.orientation.x = orientationDataQuat.x();
  imu_msg.orientation.y = orientationDataQuat.y();
  imu_msg.orientation.z = orientationDataQuat.z();

  imu_msg.angular_velocity.x = angVelocityData.gyro.x;
  imu_msg.angular_velocity.y = angVelocityData.gyro.y;
  imu_msg.angular_velocity.z = angVelocityData.gyro.z;

  imu_msg.linear_acceleration.x = linearAccelData.acceleration.x;
  imu_msg.linear_acceleration.y = linearAccelData.acceleration.y;
  imu_msg.linear_acceleration.z = linearAccelData.acceleration.z;

  memset(imu_msg.orientation_covariance, 0.0,
         sizeof(imu_msg.orientation_covariance));
  memset(imu_msg.angular_velocity_covariance, 0.0,
         sizeof(imu_msg.angular_velocity_covariance));
  memset(imu_msg.linear_acceleration_covariance, 0.0,
         sizeof(imu_msg.linear_acceleration_covariance));

  imu_msg.header.stamp = realMain.rosHandler.nodeHandle.now();
  imu_msg.header.frame_id = "imu_link";
  imu_msg.header.seq = 0;

  return success;
}

// Taken from Adafruit Library
void DavidImu::printAll() {
  printEvent(&orientationDataEuler);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Console.println();
  Console.print(F(">temperature:"));
  Console.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Console.println();
  Console.print("Calibration: Sys=");
  Console.print(system);
  Console.print(" Gyro=");
  Console.print(gyro);
  Console.print(" Accel=");
  Console.print(accel);
  Console.print(" Mag=");
  Console.println(mag);

  Console.println("--");
}

// Taken from Adafruit Library
void DavidImu::printEvent(sensors_event_t *event) {
  float x = -1000000, y = -1000000,
        z = -1000000; // dumb values, easy to spot problem
  Console.printf(">time(ms):%d\r\n", millis());
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    Console.printf(">AcclX(m/s^2):%f\r\n", x);
    Console.printf(">AcclY(m/s^2):%f\r\n", y);
    Console.printf(">AcclZ(m/s^2):%f\r\n", z);
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    Console.printf(">OrientX(deg):%f\r\n", x);
    Console.printf(">OrientY(deg):%f\r\n", y);
    Console.printf(">OrientZ(deg):%f\r\n", z);
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
    Console.printf(">MagX(uT):%f\r\n", x);
    Console.printf(">MagY(uT):%f\r\n", y);
    Console.printf(">MagZ(uT):%f\r\n", z);
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    Console.printf(">GyroX(rad/s):%f\r\n", x);
    Console.printf(">GyroY(rad/s):%f\r\n", y);
    Console.printf(">GyroZ(rad/s):%f\r\n", z);
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    Console.printf(">LinearX(m/s^2):%f\r\n", x);
    Console.printf(">LinearY(m/s^2):%f\r\n", y);
    Console.printf(">LinearZ(m/s^2):%f\r\n", z);
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    Console.printf(">GravityX(m/s^2):%f\r\n", x);
    Console.printf(">GravityY(m/s^2):%f\r\n", y);
    Console.printf(">GravityZ(m/s^2):%f\r\n", z);
  } else {
    LOGERROR("Unknown BNO055 Event Type");
  }
}