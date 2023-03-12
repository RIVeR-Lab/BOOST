#ifndef _IMU_H_
#define _IMU_H_

#include "utils/log.h"
#include "FakeThread.h"
#include "STM32encoder.h"

// class EncoderManager : public FakeThread {
// public:
//   EncoderManager()
//       : FakeThread(LOOP_DELAY_MS) {}

//   bool init() {
//     bool success = true;

//     // l_encoder.attach();

//     LOGEVENT("Encoder initiated SUCCESSFULLY");
//     return success;
//   }

  
//   bool loopHook() override;
  
// private:
//   static constexpr uint32_t LOOP_DELAY_MS = 10;
//   static void l_encoder_callback(void) {

//   }
//   // STM32encoder l_encoder;
//   // STM32encoder r_encoder;
// };

#endif // _IMU_H_