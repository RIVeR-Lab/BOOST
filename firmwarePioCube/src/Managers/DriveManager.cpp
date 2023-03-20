#include "DriveManager.h"
#include "RealMain.h"

constexpr float DriveManager::WHEEL_BASE;
constexpr float DriveManager::WHEEL_RADIUS;
constexpr float DriveManager::PWM_MIN;
constexpr float DriveManager::PWM_MAX;
constexpr float DriveManager::MOTOR_RPM;
constexpr float DriveManager::MOTOR_MIN_RAD_PER_SEC;
constexpr float DriveManager::MOTOR_MAX_RAD_PER_SEC;

bool DriveManager::init() {
  bool success = true;
  LOGEVENT("Initializing...");
  success = success && mtrCtrl.init();
  success = success && lvlShifter.init();
  success = success && lvlShifter.enable();
  if(success) {
    LOGINFO("SUCCESSFULLY initialized.");
  } else {
    LOGERROR("FAILED to initialize.");
    // TODO: Change this do a macro
    realMain.rosManager.nodeHandle.logerror("DriveManager FAILED to initialize.");
  }
  return success;
}

bool DriveManager::loopHook() {
  // Don't do anything here.
  return true;
}

bool DriveManager::logLoopHook() {
  char buf[256];
  sprintf(
      buf,
      "msg.linear.x:%f:msg.angular.z:%f:vel_l:%f:vel_r:%f:pwm_l:%ld:pwm_r:%ld",
      lastTwistMsg.linear.x, lastTwistMsg.angular.z, lastCommandedVel_l,
      lastCommandedVel_r, lastCommandedPwm_l, lastCommandedPwm_r);
  LOGEVENT(buf);

  return true;
}

/**
 * @brief Handles when we receive a new twist message from ROS and we want to
 * update our robots driving based on the new messages.
 */
bool DriveManager::handlerNewTwistMsgRcv(const geometry_msgs::Twist msg) {
  bool success = true;
  float vel_l = 0;
  float vel_r = 0;
  // Convert the twist message to a velocity (rad/s) for each wheel.
  twistMsgToVelocity(msg, vel_l, vel_r);
  // Convert the velocity (rad/s) to a PWM value.
  uint32_t pwm_l = velocityToPwm(vel_l);
  uint32_t pwm_r = velocityToPwm(vel_r);
  // Check PWM ranges
  if (pwm_l > PWM_MAX || pwm_r > PWM_MAX || pwm_l < PWM_MIN ||
      pwm_r < PWM_MIN) {
    LOGERROR("PWM out of range: lPwm=%d, rPwm=%d", pwm_l, pwm_r);
    success = false;
  }

  // Set direction pins and PWM
  lvlShifter.enable();  // make sure this is enabled.
  if (success) {
    if (pwm_l < 1) {
      mtrCtrl.cmdMotor(L293N::MOTOR_LEFT, L293N::DIR_STOP, 0);
    } else if (vel_l > 0) {
      mtrCtrl.cmdMotor(L293N::MOTOR_LEFT, L293N::DIR_FORWARD, pwm_l);
    } else {
      mtrCtrl.cmdMotor(L293N::MOTOR_LEFT, L293N::DIR_BACKWARD, pwm_l);
    }

    if (pwm_r < 1) {
      mtrCtrl.cmdMotor(L293N::MOTOR_RIGHT, L293N::DIR_STOP, 0);
    } else if (vel_r > 0) {
      mtrCtrl.cmdMotor(L293N::MOTOR_RIGHT, L293N::DIR_FORWARD, pwm_r);
    } else {
      mtrCtrl.cmdMotor(L293N::MOTOR_RIGHT, L293N::DIR_BACKWARD, pwm_r);
    }

    // Save the last commanded values
    lastCommandedVel_l = vel_l;
    lastCommandedVel_r = vel_r;
    lastCommandedPwm_l = pwm_l;
    lastCommandedPwm_r = pwm_r;
  }
  return success;
}

/**
 * @brief Converts a twist message (unicycle model) to a velocity for each wheel
 * (differential drive model) in rad/s.
 *
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
bool DriveManager::twistMsgToVelocity(const geometry_msgs::Twist msg,
                                      float &vel_l, float &vel_r) {
  bool success = true;

  float v_r = ((2.0 * msg.linear.x) + (msg.angular.z * WHEEL_BASE)) /
              (2.0 * WHEEL_RADIUS);
  float v_l = ((2.0 * msg.linear.x) - (msg.angular.z * WHEEL_BASE)) /
              (2.0 * WHEEL_RADIUS);

  // Cap the values at the max/min RPMs values of the motors
  v_r = std::max(std::min(v_r, MOTOR_MAX_RAD_PER_SEC), MOTOR_MIN_RAD_PER_SEC);
  v_l = std::max(std::min(v_l, MOTOR_MAX_RAD_PER_SEC), MOTOR_MIN_RAD_PER_SEC);

  vel_l = v_l;
  vel_r = v_r;

  return success;
}

// Map rad/sec to PWM intensities
uint32_t DriveManager::velocityToPwm(float vel) {
  return static_cast<uint32_t>(
      utils::mapInOut(fabs(vel), 0, MOTOR_MAX_RAD_PER_SEC, PWM_MIN, PWM_MAX));
}