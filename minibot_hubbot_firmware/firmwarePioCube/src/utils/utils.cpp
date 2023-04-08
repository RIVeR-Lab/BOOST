#include "utils/utils.h"

void utils::pinModeAF(PinName ulPin, uint32_t alternate_function) {
  int pn = ulPin;

  if (STM_PIN(pn) < 8) {
    LL_GPIO_SetAFPin_0_7(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn),
                         alternate_function);
  } else {
    LL_GPIO_SetAFPin_8_15(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn),
                          alternate_function);
  }

  LL_GPIO_SetPinMode(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn),
                     LL_GPIO_MODE_ALTERNATE);
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float utils::mapPwm(float x, float out_min, float out_max) {
  return x * (out_max - out_min) + out_min;
}

// Map x value from [in_min .. in_max] to [out_min .. out_max]
float utils::mapInOut(float x, float in_min, float in_max, float out_min,
               float out_max) {
  // Point slope form
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
