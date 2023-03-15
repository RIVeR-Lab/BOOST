#ifndef _L293N_H_
#define _L293N_H_

#include "Arduino.h"
#include "variant_nucleo_f446re_custom.h"
#include "utils/log.h"

class L293N {
public:
  L293N(uint32_t _in1Pin, uint32_t _in2Pin, uint32_t _in3Pin, uint32_t _in4Pin,
        uint32_t _enableAPin = NOPIN, uint32_t _enableBPin = NOPIN)
      : enableAPin(_enableAPin), enableBPin(_enableBPin), in1Pin(_in1Pin),
        in2Pin(_in2Pin), in3Pin(_in3Pin), in4Pin(_in4Pin) {}
  ~L293N() {}

  typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
  } e_motor_t;

  typedef enum {
    DIR_FORWARD = 0,
    DIR_BACKWARD = 1,
    DIR_STOP = 2,
  } e_direction_t;

  bool init();
  bool cmdMotor(e_motor_t motor, e_direction_t dir, uint32_t pwm);

private:
  const uint32_t enableAPin = NOPIN;
  const uint32_t enableBPin = NOPIN;
  const uint32_t in1Pin = NOPIN;
  const uint32_t in2Pin = NOPIN;
  const uint32_t in3Pin = NOPIN;
  const uint32_t in4Pin = NOPIN;

  bool enable(uint32_t pin, bool enable);
  /**
   * @pin: in1Pin, in2Pin, in3Pin, in4Pin
   * @pwm: 0-255
  */
  bool setPwm(uint32_t pin, uint32_t pwm);

};

#endif // _L293N_H_