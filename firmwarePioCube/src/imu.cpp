#include "imu.h"
#include "RealMain.h"

bool IMU::loopHook() {
  LOGEVENT("IMU::loopHook()");
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
bool IMU::getAllImuData(sensor_msgs::Imu &imu_msg) {
  bool success = true;

  sensors_event_t orientationDataEuler, angVelocityData, linearAccelData,
      magnetometerData, accelerometerData, gravityData;
  imu::Quaternion orientationDataQuat;
  bno.getEvent(&orientationDataEuler, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  orientationDataQuat = bno.getQuat();

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
void IMU::printAll() {
  // could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData, angVelocityData, linearAccelData,
      magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F(">temperature:"));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
}

// Taken from Adafruit Library
void IMU::printEvent(sensors_event_t *event) {
  float x = -1000000, y = -1000000,
        z = -1000000; // dumb values, easy to spot problem
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