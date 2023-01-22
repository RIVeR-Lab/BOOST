#ifndef __PWM_H__
#define __PWM_H__

#include "Arduino.h"
#include "main.h"
#include "HardwareTimer.h"


extern void initPwm();
extern void setPwm(uint32_t freq, uint32_t duty);

// Automatically retrieve TIM instance and channel associated to pin
// TIM1 = PA8
static TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(PWM1, PinMap_PWM);
static const uint32_t channel = STM_PIN_CHANNEL(pinmap_function(PWM1, PinMap_PWM));

static HardwareTimer *pwmTim = new HardwareTimer(Instance);


#endif /* __PWM_H__ */

