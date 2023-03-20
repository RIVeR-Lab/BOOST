#include "L293N.h"

bool L293N::init() {
  bool success = true;
  LOGEVENT("Initializing...");
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  if (enableAPin != NOPIN) {
    pinMode(enableAPin, OUTPUT);
    success = success && enable(enableAPin, true);
  }
  if (enableBPin != NOPIN) {
    pinMode(enableBPin, OUTPUT);
    success = success && enable(enableBPin, true);
  }

  if (!success) {
    LOGERROR("FAILED to initialize.");
  } else {
    LOGINFO("SUCCESSFULLY initialized.");
  }

  return success;
}

bool L293N::enable(uint32_t pin, bool enable) {
  bool success = true;
  digitalWrite(pin, enable ? HIGH : LOW);
  return success;
}

bool L293N::setPwm(uint32_t pin, uint32_t pwm) {
  bool success = true;
  analogWrite(pin, pwm);
  return success;
}

/**
 * @motor: MOTOR_LEFT, MOTOR_RIGHT
 * @dir: DIRECTION_FORWARD, DIRECTION_BACKWARD, MOTOR_STOP
 * @pwm: 0-255, only used when dir is DIRECTION_FORWARD or DIRECTION_BACKWARD
 */
bool L293N::cmdMotor(e_motor_t motor, e_direction_t dir, uint32_t pwm) {
  bool success = true;
  uint32_t in1 = NOPIN;
  uint32_t in2 = NOPIN;
  switch (motor) {
  case MOTOR_LEFT:
    in1 = in1Pin;
    in2 = in2Pin;
    break;
  case MOTOR_RIGHT:
    in1 = in3Pin;
    in2 = in4Pin;
    break;
  default:
    success = false;
    break;
  }
  if (success) {
    if (dir == DIR_STOP) {
      success = success && setPwm(in1, 0);
      success = success && setPwm(in2, 0);
    } else {
      success = success && setPwm(in1, dir == DIR_FORWARD ? pwm : 0);
      success = success && setPwm(in2, dir == DIR_BACKWARD ? pwm : 0);
    }
  }

  if (!success) {
    LOGERROR("FAILED to command motor: motor=%d, dir=%d, pwm=%d", motor, dir,
             pwm);
  } else {
    LOGINFO("SUCCESSFULLY commanded motor: motor=%d, dir=%d, pwm=%d", motor,
            dir, pwm);
  }
  return success;
}