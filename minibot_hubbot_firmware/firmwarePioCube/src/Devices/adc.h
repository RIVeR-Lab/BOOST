#ifndef __ADC_H__
#define __ADC_H__

#include "Arduino.h"

#ifdef NUCLEO_F767ZI_CUSTOM
#include <stm32f7xx_ll_adc.h>
#elif NUCLEO_F446RE_CUSTOM
#include <stm32f4xx_ll_adc.h>
#endif

/* Analog read resolution */
#define LL_ADC_RESOLUTION_BITS 12
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B

class ADCClass {
  public:
  ADCClass(){}
  ~ADCClass(){}

  static void initAdc() { analogReadResolution(LL_ADC_RESOLUTION_BITS); }

  /**
   * @return Vref in mV
   */
  static int32_t readVref() {
    return (
        __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION));
  }

  /**
   * @return ADC conversion data equivalent voltage value (unit: mVolt)
   */
  static int32_t readVoltage(int32_t VRef, uint32_t pin) {
    return (__LL_ADC_CALC_DATA_TO_VOLTAGE(VRef, analogRead(pin),
                                          LL_ADC_RESOLUTION));
  }
};

#endif /* __ADC_H__ */
