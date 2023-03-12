#ifndef _UTILS_H_
#define _UTILS_H_
#include "Arduino.h"
//-----------------------------------------------------------------------------
// pinModeAF - configure STM32 pin to given alternate-function
//-----------------------------------------------------------------------------
// Alternate values: see LL_GPIO_SetAFPin_8_15() and PeripheralPins.c
// example call: pinModeAF(PD12, GPIO_AF2_TIM4);
//-----------------------------------------------------------------------------
extern void pinModeAF(PinName ulPin, uint32_t alternate_function);

#endif // _UTILS_H_