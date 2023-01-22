#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "Arduino.h"
#include "main.h"

extern void initAdc();
extern int32_t readVoltage(int32_t VRef, uint32_t pin);
extern int32_t readVref();

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

