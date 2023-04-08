#ifndef _DRIVERMANAGER_H_
#define _DRIVERMANAGER_H_

#include "Arduino.h"
#include "FakeThread.h"
#include "L293N.h"
#include "TXB0104PWR.h"
#include "utils/utils.h"
#include "utils/log.h"
#include <geometry_msgs/Twist.h>
#include "utils/macros.h"

class RealMain;

class DriveManager : public FakeThread {
public:
  DriveManager(L293N &_mtrCtrl, TXB0104PWR &_lvlShifter)
      : FakeThread(LOOP_DELAY_MS, LOG_LOOP_DELAY_MS), mtrCtrl(_mtrCtrl),
        lvlShifter(_lvlShifter) {}
  ~DriveManager() {}

  bool init() override;

  bool handlerNewTwistMsgRcv(const geometry_msgs::Twist msg);

private:
  L293N &mtrCtrl;
  TXB0104PWR &lvlShifter;
  // We don't need to loop fast because we set motor speed when a new twist
  // message comes in.
  static constexpr uint32_t LOOP_DELAY_MS = 1000;
  static constexpr uint32_t LOG_LOOP_DELAY_MS = LOOP_DELAY_MS;

  static constexpr float WHEEL_BASE =
      (155.0 / 1000.0); // (meters per radian) (aka just meters between 2 wheels on the same
           // axle)
  static constexpr float WHEEL_RADIUS =
      (30.0 / 1000.0); // (meters per radian) (aka just the radius of the wheel)
  static constexpr float PWM_MIN = 0;
  static constexpr float PWM_MAX = 255;
  static constexpr float MOTOR_RPM = 60;
  static constexpr float MOTOR_MAX_RAD_PER_SEC =
      MOTOR_RPM * ((2.0 * M_PI) / 60.0); // rad/s
  static constexpr float MOTOR_MIN_RAD_PER_SEC =
      -MOTOR_MAX_RAD_PER_SEC; // rad/s

  // These are calculated from the ROS twist message.
  float lastCommandedVel_l = 0;
  float lastCommandedVel_r = 0;
  uint32_t lastCommandedPwm_l = 0;
  uint32_t lastCommandedPwm_r = 0;
  geometry_msgs::Twist lastTwistMsg{};

  bool loopHook() override;
  bool logLoopHook() override;
  bool twistMsgToVelocity(const geometry_msgs::Twist msg, float &vel_l,
                          float &vel_r);
  uint32_t velocityToPwm(float velocity);
};

#endif // _DRIVERMANAGER_H_