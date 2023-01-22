#include "pwm.h"

void initPwm(){  
  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  
}

void setPwm(uint32_t freq, uint32_t duty){
  // Configure and start PWM
  // No callback required, idk why, there must be defaults somewhere.
  pwmTim->setPWM(channel, PWM1, freq, duty, NULL, NULL);
  pwmTim->resume(); // Start PWM
}