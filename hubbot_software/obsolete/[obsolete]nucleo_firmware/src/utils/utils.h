#ifndef _UTILS_H_
#define _UTILS_H_
#include "Arduino.h"

namespace utils {
//-----------------------------------------------------------------------------
// pinModeAF - configure STM32 pin to given alternate-function
//-----------------------------------------------------------------------------
// Alternate values: see LL_GPIO_SetAFPin_8_15() and PeripheralPins.c
// example call: pinModeAF(PD12, GPIO_AF2_TIM4);
//-----------------------------------------------------------------------------
extern void pinModeAF(PinName ulPin, uint32_t alternate_function);

// Map x value from [0 .. 1] to [out_min .. out_max]
extern float mapPwm(float x, float out_min, float out_max);

// Map x value from [in_min .. in_max] to [out_min .. out_max]
extern float mapInOut(float x, float in_min, float in_max, float out_min,
                      float out_max);
};     // namespace utils
#endif // _UTILS_H_