#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include "FakeThread.h"
#include "utils/log.h"
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h> 
#include "utils/macros.h"

class RosManager : public FakeThread {
public:
  RosManager(HardwareSerial &serialPort)
      : FakeThread(LOOP_DELAY_MS, LOG_LOOP_DELAY_MS), nodeHardware(serialPort),
        chatter("chatter", &str_msg),
        bno055_imu_pub("bno055_imu", &bno055_imu_msg),
        encoder_left_pub("encoder_left", &encoder_left_msg),
        encoder_right_pub("encoder_right", &encoder_right_msg),
        gps_pub("gps", &gps_msg),
        subDiffDrive("/cmd_vel", &subDiffDrive_cb)
        {}
  ~RosManager() {}

  static constexpr uint32_t rosSerialBaud = 57600;

  bool init() override;
  bool loopHook() override;
  bool logLoopHook() override;

  ros::NodeHandle nodeHandle;

private:
  const HardwareSerial &nodeHardware;
  bool isRosConnected = false;
  static constexpr uint32_t LOOP_DELAY_MS = 1;
  static constexpr uint32_t LOG_LOOP_DELAY_MS = 500;

  /* -------------------------- PUBLISHERS -------------------------- */
  // Chatter
  ros::Publisher chatter;
  std_msgs::String str_msg{};
  std::string str_msg_data = "Chatter Topic: MCU blink!";
  // IMUS
  ros::Publisher bno055_imu_pub;
  sensor_msgs::Imu bno055_imu_msg{};
  // Encoder
  ros::Publisher encoder_left_pub;
  ros::Publisher encoder_right_pub;
  std_msgs::Int32 encoder_left_msg{};
  std_msgs::Int32 encoder_right_msg{};
  // GPS
  ros::Publisher gps_pub;
  sensor_msgs::NavSatFix gps_msg{};
  /* -------------------------- END PUBLISHERS -------------------------- */

  /* -------------------------- SUBSCRIBERS -------------------------- */
  // For diff drive control
  ros::Subscriber<geometry_msgs::Twist> subDiffDrive;
  static void subDiffDrive_cb(const geometry_msgs::Twist &msg);
  /* -------------------------- END SUBSCRIBERS -------------------------- */
};

#endif // ROSHANDLER_H
