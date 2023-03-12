#include "utils.h"

void pinModeAF(PinName ulPin, uint32_t alternate_function)
{
   int pn = ulPin;

   if (STM_PIN(pn) < 8) {
      LL_GPIO_SetAFPin_0_7(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), alternate_function);
   } else {
      LL_GPIO_SetAFPin_8_15(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), alternate_function);
   }

   LL_GPIO_SetPinMode(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), LL_GPIO_MODE_ALTERNATE);
}