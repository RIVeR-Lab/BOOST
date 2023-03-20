#include "RosManager.h"
#include "RealMain.h"

// TODO: Report firmware version

bool RosManager::init() {
  bool success = true;
  LOGEVENT("Initializing...");

  // SETUP HARDWARE AND ROS NODE
  nodeHandle.getHardware()->setBaud(rosSerialBaud);
  nodeHandle.getHardware()->setPort(
      const_cast<HardwareSerial *>(&nodeHardware));
  nodeHandle.initNode();

  // SETUP PUBLISHERS
  nodeHandle.advertise(chatter);

#if ENABLE_IMU
  nodeHandle.advertise(bno055_imu_pub);
#endif

#if ENABLE_ODOMETRY
  nodeHandle.advertise(encoder_left_pub);
  nodeHandle.advertise(encoder_right_pub);
#endif

#if ENABLE_GPS
  nodeHandle.advertise(gps_pub);
#endif

  // SETUP SUBSCRIBERS
  nodeHandle.subscribe(subDiffDrive);

  if(!success){
    LOGERROR("FAILED to initialize.");
  } else {
    LOGEVENT("Initialized SUCCESSFULLY.");
  }

  return success;
}

bool RosManager::loopHook() {
  bool success = true;
  // LOGEVENT("RosManager::loopHook()");
  // LOGINFO("chatter_msg size: %d", sizeof(str_msg));
  // LOGINFO("imu_msg size: %d", sizeof(bno055_imu_msg));

  // TODO: Check if ROS is connected
  // if(!nodeHandle.connected()){
  //     isRosConnected = false;
  // } else {
  isRosConnected = true;

  // TODO: Get rid of the duplicate code below.
  // Publish Chatter
  static uint32_t chatterLastPub = 0;
  if ((millis() - chatterLastPub) > 1000) {
    chatterLastPub = millis();
    str_msg.data = str_msg_data.c_str();
    chatter.publish(&str_msg);
  }

// Publish IMU data
#if ENABLE_IMU
  static uint32_t imuLastPub = 0;
  if ((millis() - imuLastPub) > 100) {
    imuLastPub = millis();
    success = success && realMain.imuManager.readInAllImuData();
    success = success && realMain.imuManager.toRosImuMsg(bno055_imu_msg);
    if (!success) {
      LOGERROR("Failed to read IMU data from BNO055.");
    } else {
      bno055_imu_pub.publish(&bno055_imu_msg);
    }
  }
#endif

// Publish Encoder data
#if ENABLE_ODOMETRY
  static uint32_t encoderLastPub = 0;
  if ((millis() - encoderLastPub) > 100) {
    encoderLastPub = millis();
    encoder_left_msg.data = realMain.encLeft.read();
    encoder_right_msg.data = realMain.encRight.read();
    encoder_left_pub.publish(&encoder_left_msg);
    encoder_right_pub.publish(&encoder_right_msg);
  }
#endif

// Publish GPS data
#if ENABLE_GPS
  static uint32_t gpsLastPub = 0;
  if ((millis() - gpsLastPub) > 100) {
    gpsLastPub = millis();
    gps_msg = realMain.gpsManager.toRosMsg();
    gps_pub.publish(&gps_msg);
  }
#endif

  // For subscribers
  nodeHandle.spinOnce();
  // }

  return success;
}

bool RosManager::logLoopHook() {
#if ENABLE_ROSMANAGER_LOGLOOP
  LOGEVENT("%s", __func__);
#endif
  return true;
}

void RosManager::subDiffDrive_cb(const geometry_msgs::Twist &msg) {
  LOGINFO("Linear: %f, %f, %f", msg.linear.x, msg.linear.y, msg.linear.z);
  LOGINFO("Angular: %f, %f, %f", msg.angular.x, msg.angular.y, msg.angular.z);
  realMain.drvManager.handlerNewTwistMsgRcv(msg);
}
