#ifndef _IMU_H_
#define _IMU_H_

#include "utils/log.h"
#include "FakeThread.h"
#include "STM32encoder.h"



class Encoder : public FakeThread {
public:
  Encoder()
      : FakeThread(LOOP_DELAY_MS) {}

  bool init() {
    bool success = true;

    l_encoder.attach();

    LOGEVENT("Encder initiated SUCCESSFULLY");
    return success;
  }

  
  bool loopHook() override;
  
private:
  static constexpr uint32_t LOOP_DELAY_MS = 10;
  static void l_encoder_callback(void) {

  }
  STM32Encoder l_encoder;
  STM32Encoder r_encoder;
};

#endif // _IMU_H_