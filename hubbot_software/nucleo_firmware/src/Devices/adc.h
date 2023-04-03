#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "Arduino.h"
#include "main.h"

#ifdef NUCLEO_F767ZI_CUSTOM
#include <stm32f7xx_ll_adc.h>
#endif

/* Analog read resolution */
#define LL_ADC_RESOLUTION_BITS 12
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 1024

extern void initAdc();
extern int32_t readVoltage(int32_t VRef, uint32_t pin);
extern int32_t readVref();

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

