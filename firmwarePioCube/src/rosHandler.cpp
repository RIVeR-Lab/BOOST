#include "rosHandler.h"
#include "RealMain.h"

constexpr float RosHandler::WHEEL_BASE;
constexpr float RosHandler::WHEEL_RADIUS;
constexpr float RosHandler::PWM_MIN;
constexpr float RosHandler::PWM_MAX;
constexpr float RosHandler::MOTOR_RPM;
constexpr float RosHandler::MOTOR_MIN_RAD_PER_SEC;
constexpr float RosHandler::MOTOR_MAX_RAD_PER_SEC;



  bool RosHandler::init() {
    bool success = true;

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

    // SETUP SUBSCRIBERS
    nodeHandle.subscribe(subDiffDrive);

    if (!success) {
      LOGERROR("Failed to initialize ROS node.");
    } else {
      LOGINFO("Successfully initialized ROS node.");
    }

    return success;
  }

  bool RosHandler::loopHook() {
    bool success = true;
    LOGEVENT("RosHandler::loopHook()");
    LOGINFO("chatter_msg size: %d", sizeof(str_msg));
    LOGINFO("imu_msg size: %d", sizeof(bno055_imu_msg));

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
      success = success && realMain.imu.getAllImuData(bno055_imu_msg);
      if(!success){
        LOGERROR("Failed to read IMU data from BNO055.");
      } else {
        bno055_imu_pub.publish(&bno055_imu_msg);
      }
    }
    #endif

    // For subscribers
    nodeHandle.spinOnce();
    // }

    return success;
  }

  // Map x value from [0 .. 1] to [out_min .. out_max]
  float mapPwm(float x, float out_min, float out_max) {
    return x * (out_max - out_min) + out_min;
  }

  // Map x value from [in_min .. in_max] to [out_min .. out_max]
  float mapInOut(float x, float in_min, float in_max, float out_min,
                 float out_max) {
    // Point slope form
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  /**
   * @brief Converts unicycle model to differential drive model.
   * Taken from the following:
   * https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
   * v_r (rad/sec) = right wheel velocity
   * v_l (rad/sec) = left wheel velocity
   * v (m/sec) = linear velocity = twist.linear.x
   * w (rad/sec) = angular velocity = twist.angular.z
   * R (m/rad) = wheel radius
   * L (m/rad) = wheelbase (distance between wheels)
   *
   * ----> v_r = (2v + wL) / 2R
   * ----> v_l = (2v - wL) / 2R
   *
   * @param msg The unicycle model containing the linear and angular velocity.
   */
  void RosHandler::subDiffDrive_cb(const geometry_msgs::Twist &msg) {
    //   if (!_connected)
    //   {
    //     stop();
    //     return;
    //   }

    float v_r = ((2.0 * msg.linear.x) + (msg.angular.z * WHEEL_BASE)) /
                (2.0 * WHEEL_RADIUS);
    float v_l = ((2.0 * msg.linear.x) - (msg.angular.z * WHEEL_BASE)) /
                (2.0 * WHEEL_RADIUS);

    // Cap the values at the max/min RPMs values of the motors
    v_r = std::max(std::min(v_r, MOTOR_MAX_RAD_PER_SEC), MOTOR_MIN_RAD_PER_SEC);
    v_l = std::max(std::min(v_l, MOTOR_MAX_RAD_PER_SEC), MOTOR_MIN_RAD_PER_SEC);

    // Map rad/sec to PWM intensities
    uint32_t lPwm = static_cast<uint32_t>(
        mapInOut(fabs(v_l), 0, MOTOR_MAX_RAD_PER_SEC, PWM_MIN, PWM_MAX));
    uint32_t rPwm = static_cast<uint32_t>(
        mapInOut(fabs(v_r), 0, MOTOR_MAX_RAD_PER_SEC, PWM_MIN, PWM_MAX));

    char buf[128];
    sprintf(buf, "msg.linear.x: %f, msg.angular.z: %f lPwm: %ld, rPwm: %ld",
            msg.linear.x, msg.angular.z, lPwm, rPwm);
    // Serial2.printf()
    LOGEVENT(buf);

    // Set direction pins and PWM
    if (lPwm < 1) {
      analogWrite(L_WHEEL_FORW_PIN, 0);
      analogWrite(L_WHEEL_BACK_PIN, 0);
    } else if (v_l > 0) {
      analogWrite(L_WHEEL_BACK_PIN, 0);
      analogWrite(L_WHEEL_FORW_PIN, lPwm);
    } else {
      analogWrite(L_WHEEL_FORW_PIN, 0);
      analogWrite(L_WHEEL_BACK_PIN, lPwm);
    }

    if (rPwm < 1) {
      analogWrite(R_WHEEL_FORW_PIN, 0);
      analogWrite(R_WHEEL_BACK_PIN, 0);
    } else if (v_r > 0) {
      analogWrite(R_WHEEL_BACK_PIN, 0);
      analogWrite(R_WHEEL_FORW_PIN, rPwm);
    } else {
      analogWrite(R_WHEEL_FORW_PIN, 0);
      analogWrite(R_WHEEL_BACK_PIN, rPwm);
    }
  }
