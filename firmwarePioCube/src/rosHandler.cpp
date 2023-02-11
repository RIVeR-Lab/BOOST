#include "rosHandler.h"

#define PWM_MIN 0
#define PWM_MAX 255

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

void RosHandler::subDiffDrive_cb(const geometry_msgs::Twist &msg)
{
//   if (!_connected)
//   {
//     stop();
//     return;
//   }

  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;
  // LOGEVENT("l: %f, r: %f", l, r);

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWM_MAX);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWM_MAX);
  // LOGEVENT("lPwm: %d, rPwm: %d", lPwm, rPwm);

  // Set direction pins and PWM
  if(l > 0)
  {
    analogWrite(L_WHEEL_FORW_PIN, lPwm);
  }
  else
  {
    analogWrite(L_WHEEL_BACK_PIN, lPwm);
  }

  if(r > 0)
  {
    analogWrite(R_WHEEL_FORW_PIN, rPwm);
  }
  else
  {
    analogWrite(R_WHEEL_BACK_PIN, rPwm);
  }
}
